#include <set>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "imgproc/rastermask/cvmat.hpp"
#include "imgproc/scanconversion.hpp"

#include "./merge.hpp"
#include "../io.hpp"

namespace fs = boost::filesystem;

namespace vadstena { namespace vts { namespace merge {

namespace {

inline math::Matrix4 geo2grid(const math::Extents2& extents
                              , const math::Size2 &gridSize)
{
    math::Matrix4 trafo(boost::numeric::ublas::identity_matrix<double>(4));

    auto es(size(extents));

    // scales
    math::Size2f scale((gridSize.width - 1) / es.width
                       , (gridSize.height - 1) / es.height);

    // scale to grid
    trafo(0, 0) = scale.width;
    trafo(1, 1) = -scale.height;

    trafo(0, 3) = -extents.ll(0) * scale.width;
    trafo(1, 3) = extents.ur(1) * scale.height;

    return trafo;
}

} // namespace

Input::Input(Id id, const TileSet::Detail &owner, const TileId &tileId
             , const NodeInfo &nodeInfo)
    : id_(id), tileId_(tileId), owner_(&owner)
    , node_(owner.findMetaNode(tileId)), nodeInfo_(&nodeInfo)
    , sd2Coverage_(geo2grid(nodeInfo.node.extents, Mesh::coverageSize()))
{
}

bool Input::hasMesh() const
{
    return node_ && node_->geometry();
}

bool Input::hasAtlas() const
{
    return node_ && node_->internalTexture();
}

bool Input::hasNavtile() const
{
    return node_ && node_->navtile();
}

const Mesh& Input::mesh() const
{
    if (!mesh_) {
        mesh_ = owner_->getMesh(tileId_, node_);
    }

    return *mesh_;
}

const RawAtlas& Input::atlas() const
{
    if (!atlas_) {
        atlas_ = boost::in_place();
        owner_->getAtlas(tileId_, *atlas_, node_);
    }

    return *atlas_;
}

const opencv::NavTile& Input::navtile() const
{
    if (!navtile_) {
        navtile_ = boost::in_place();
        owner_->getNavTile(tileId_, *navtile_, node_);
    }

    return *navtile_;
}

const VerticesList& Input::coverageVertices() const
{
    if (!coverageVertices_) {
        geo::CsConvertor conv
            (registry::Registry::srs(nodeInfo_->node.srs).srsDef
             , registry::Registry::srs
             (nodeInfo_->referenceFrame->model.physicalSrs).srsDef);
        coverageVertices_ = convert(mesh(), conv, sd2Coverage_);
    }
    return *coverageVertices_;
}

Mesh& Output::forceMesh() {
    if (!mesh) {
        mesh = boost::in_place();
    }
    return *mesh;
}

RawAtlas& Output::forceAtlas() {
    if (!atlas) {
        atlas = boost::in_place();
    }
    return *atlas;
}

namespace {

void rasterize(const Mesh &mesh
               , const cv::Scalar &color
               , cv::Mat &coverage)
{
    mesh.coverageMask.forEachQuad([&](uint xstart, uint ystart, uint xsize
                                      , uint ysize, bool)
    {
        cv::Point2i start(xstart, ystart);
        cv::Point2i end(xstart + xsize - 1, ystart + ysize - 1);

        cv::rectangle(coverage, start, end, color, CV_FILLED, 4);
    }, Mesh::CoverageMask::Filter::white);
}

void clampMatPos(int &x, int &y, const cv::Mat &mat)
{
    if (x < 0) {
        x = 0;
    } else if (x >= mat.cols) {
        x = mat.cols-1;
    }

    if (y < 0) {
        y = 0;
    } else if (y >= mat.rows) {
        y = mat.rows-1;
    }
}

struct Coverage {
    typedef std::int16_t pixel_type;

    const TileId tileId;
    const Input::list &sources;
    cv::Mat coverage;
    bool hasHoles;
    std::vector<bool> indices;
    boost::optional<Input::Id> single;

    Coverage(const TileId &tileId, const Input::list &sources)
        : tileId(tileId), sources(sources), hasHoles(false)
        , indices(sources.back().id() + 1, false)
    {
        generateCoverage();
        analyze();
    }

    void getSources(Output &output) const {
        for (const auto &input : sources) {
            if (indices[input.id()]) {
                output.source.push_back(input);
            }
        }
    }

    const Input* getSingle() const {
        if (!single) { return nullptr; }
        auto fsources(std::find_if(sources.begin(), sources.end()
                                   , [&](const Input &input)
        {
            return input.id() == *single;
        }));
        if (fsources == sources.end()) { return nullptr; }
        return &*fsources;
    }

    bool covered(const Face &face, const math::Points3d &vertices
                 , Input::Id id)
    {
        cv::Point3f tri[3];
        {
            int index(0);
            for (const auto &i : face) {
                const auto &v(vertices[i]);
                tri[index++] = { float(v(0)), float(v(1)), float(v(2)) };
            }
        }

        std::vector<imgproc::Scanline> scanlines;
        imgproc::scanConvertTriangle(tri, 0, coverage.rows, scanlines);

        for (const auto &sl : scanlines) {
            bool covered(false);
            imgproc::processScanline(sl, 0, coverage.cols
                                     , [&](int x, int y, float)
            {
                // TODO: early exit
                if (coverage.at<pixel_type>(y, x) == id) {
                    covered = true;
                }
            });
            if (covered) { return true; }
        }

        // do one more check in case the triangle is thinner than one pixel
        for (int i = 0; i < 3; ++i) {
            if (check(tri[i].x, tri[i].y, id)) {
                return true;
            }
        }

        return false;
    }

    void dump(const fs::path &dump, const TileId &tileId) const {
        const cv::Vec3b colors[10] = {
            { 0, 0, 0 }
            , { 0, 0, 255 }
            , { 0, 255, 0 }
            , { 255, 0, 0 }
            , { 0, 255, 255 }
            , { 255, 0, 255 }
            , { 255, 255, 0 }
            , { 255, 255, 255 }
        };

        cv::Mat img(coverage.rows, coverage.cols, CV_8UC3);

        std::transform(coverage.begin<pixel_type>()
                       , coverage.end<pixel_type>()
                       , img.begin<cv::Vec3b>()
                       , [&](pixel_type v) { return colors[v + 1]; });

        const auto filename
            (dump / str(boost::format("coverage-%s.png") % tileId));
        create_directories(filename.parent_path());
        imwrite(filename.string(), img);
    }

private:
    void generateCoverage() {
        // prepare coverage map (set to invalid index)
        auto coverageSize(Mesh::coverageSize());
        coverage.create(coverageSize.height, coverageSize.width, CV_16S);
        coverage = cv::Scalar(-1);

        for (const auto &input : sources) {
            // TODO: handle proper rasterizing
            if (input.tileId().lod == tileId.lod) {
                rasterize(input.mesh(), input.id(), coverage);
            }
        }
    }

    void analyze() {
        for (auto j(0); j < coverage.rows; ++j) {
            for (auto i(0); i < coverage.cols; ++i) {
                auto v(coverage.at<pixel_type>(j, i));
                if (v == -1) {
                    hasHoles = true;
                } else {
                    indices[v] = true;
                }
            }
        }

        int count(0);
        for (Input::Id id(0), eid(indices.size()); id != eid; ++id) {
            if (!indices[id]) { continue; }
            if (!single) {
                single = id;
            }
            ++count;
        }
        if (count > 1) { single = boost::none; }

        // convert special negative value back to no value
        if (single && (single < 0)) { single = boost::none; }

        for (int i(0), e(indices.size()); i != e; ++i) {
            if (indices[i]) {
                LOG(info4) << "    " << i;
            }
        }
    }

    bool check(float x, float y, Input::Id id) const {
        int xx(std::round(x));
        int yy(std::round(y));
        if ((xx < 0) || (xx >= coverage.cols)) { return false; }
        if ((yy < 0) || (yy >= coverage.rows)) { return false; }
        return (coverage.at<pixel_type>(yy, xx) == id);
    }
};

class MeshAccumulator {
public:
    MeshAccumulator(const SubMesh &original)
        : original_(original)
        , vertexMap_(original.vertices.size(), -1)
        , tcMap_(original.tc.size(), -1)
    {}

    void addFace(int faceIndex) {
        const auto &of(original_.faces[faceIndex]);
        result_.faces.emplace_back
            (addVertex(of(0)), addVertex(of(1)), addVertex(of(2)));

        if (!original_.facesTc.empty()) {
            const auto &oftc(original_.facesTc[faceIndex]);
            result_.facesTc.emplace_back
                (addTc(oftc(0)), addTc(oftc(1)), addTc(oftc(2)));
        }
    }

    const SubMesh& get() const { return result_; }

    operator bool() const { return result_.vertices.size(); }

private:
    int addVertex(int i) {
        auto &m(vertexMap_[i]);
        if (m < 0) {
            // new vertex
            m = result_.vertices.size();
            result_.vertices.push_back(original_.vertices[i]);
            if (!original_.etc.empty()) {
                result_.etc.push_back(original_.etc[i]);
            }
            if (!original_.vertexUndulation.empty()) {
                result_.vertexUndulation.push_back
                    (original_.vertexUndulation[i]);
            }
        }
        return m;
    }

    int addTc(int i) {
        auto &m(tcMap_[i]);
        if (m < 0) {
            // new vertex
            m = result_.tc.size();
            result_.tc.push_back(original_.tc[i]);
        }
        return m;
    }

    const SubMesh &original_;
    SubMesh result_;
    std::vector<int> vertexMap_;
    std::vector<int> tcMap_;
};

} // namespace

Output mergeTile(const TileId &tileId, const Input::list &currentSource
                 , const Input::list &parentSource
                 , int quadrant)
{
    // build uniform source by merging current and parent sources
    // current data have precedence
    // only tiles with mesh are used
    Input::list source;
    {
        auto icurrentSource(currentSource.begin())
            , ecurrentSource(currentSource.end());
        auto iparentSource(parentSource.begin())
            , eparentSource(parentSource.end());

        // merge head and common content
        while ((icurrentSource != ecurrentSource)
               && (iparentSource != eparentSource))
        {
            const auto &ts(*icurrentSource);
            const auto &ps(*iparentSource);
            if (ts < ps) {
                if (ts.hasMesh()) { source.push_back(ts); }
                ++icurrentSource;
            } else if (ps < ts) {
                if (ps.hasMesh()) { source.push_back(ps); }
                ++iparentSource;
            } else {
                if (ts.hasMesh()) {
                    source.push_back(ts);
                } else if (ps.hasMesh()) {
                    source.push_back(ps);
                }
                ++icurrentSource;
                ++iparentSource;
            }
        }

        // copy tail (one or another)
        for (; icurrentSource != ecurrentSource; ++icurrentSource) {
            if (icurrentSource->hasMesh()) {
                source.push_back(*icurrentSource);
            }
        }

        for (; iparentSource != eparentSource; ++iparentSource) {
            if (iparentSource->hasMesh()) {
                source.push_back(*iparentSource);
            }
        }
    }

    // from here, all input tiles have geometry -> no need to check for mesh
    // presence

    // merge result
    Output result;

    if (source.empty()) { return result; }

    if ((source.size() == 1)) {
        // just copy one source
        const auto &input(source.back());
        result.mesh = input.mesh();
        if (input.hasAtlas()) { result.atlas = input.atlas(); }
        if (input.hasNavtile()) { result.navtile = input.navtile(); }
        return result;
    }

    // analyze coverage
    Coverage coverage(tileId, source);

    if (const auto *dumpDir = ::getenv("MERGE_MASK_DUMP_DIR")) {
        coverage.dump(dumpDir, tileId);
    }

    // get contributing tile sets
    coverage.getSources(result);

    if (coverage.single) {
        // single source
        if (*coverage.single < 0) {
            // nothing at all
            return result;
        }

        // just one source -> just copy
        if (const auto input = coverage.getSingle()) {
            result.mesh = input->mesh();
            if (input->hasAtlas()) { result.atlas = input->atlas(); }
            if (input->hasNavtile()) { result.navtile = input->navtile(); }
        }

        // OK
        return result;
    }

    // TODO: merge navtile based on navtile coverage

    // process all input tiles from result source (i.e. only those contributing
    // to the tile)
    for (const auto &input : result.source) {
        const auto &mesh(input.mesh());
        // get current mesh vertices converted to coverage coordinate system
        auto icoverageVertices(input.coverageVertices().begin());

        // traverse all submeshes
        for (int m(0), em(mesh.submeshes.size()); m != em; ++m) {
            const auto &sm(mesh.submeshes[m]);
            const auto &coverageVertices(*icoverageVertices++);

            MeshAccumulator ma(sm);

            for (int f(0), ef(sm.faces.size()); f != ef; ++f) {
                // only vertices
                if (coverage.covered
                    (sm.faces[f], coverageVertices, input.id()))
                {
                    ma.addFace(f);
                }
            }

            if (ma) {
                // there are some vertices -> put to output
                result.forceMesh().submeshes.push_back(ma.get());
                if (input.hasAtlas() && input.atlas().valid(m)) {
                    result.forceAtlas().add(input.atlas().get(m));
                }
            }
        }
    }

    return result;

    (void) quadrant;
}

} } } // namespace vadstena::merge::vts
