#include <cstdlib>
#include <string>
#include <iostream>
#include <algorithm>
#include <iterator>

#include <jpeglib.h>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"
#include "utility/streams.hpp"

#include "utility/buildsys.hpp"
#include "utility/gccversion.hpp"
#include "utility/progress.hpp"
#include "utility/streams.hpp"
#include "utility/openmp.hpp"

#include "math/transform.hpp"
#include "math/filters.hpp"

#include "imgproc/scanconversion.hpp"

#include "geo/csconvertor.hpp"
#include "geo/coordinates.hpp"

#include "vadstena-libs/utility.hpp"

// old stuff
#include "vadstena-libs/binmesh.hpp"
#include "vadstena-libs/vts0.hpp"
#include "vadstena-libs/vts0/tileset-advanced.hpp"
#include "vadstena-libs/vts0/io.hpp"

// new stuff
#include "vadstena-libs/vts.hpp"
#include "vadstena-libs/vts/encoder.hpp"
#include "vadstena-libs/vts/opencv/navtile.hpp"
#include "vadstena-libs/vts/io.hpp"

namespace po = boost::program_options;
namespace vs = vadstena::storage;
namespace vr = vadstena::registry;
namespace vts0 = vadstena::vts0;
namespace vts = vadstena::vts;
namespace va = vadstena;
namespace fs = boost::filesystem;
namespace ublas = boost::numeric::ublas;

namespace {

math::Size2 jpegSize(std::istream &is, const fs::path &path)
{
    char buf[1024];
    std::size_t size(sizeof(buf));

    {
        auto exc(utility::scopedStreamExceptions(is));

        // clear EOF bit
        is.exceptions(exc.state()
                      & ~(std::ios_base::failbit | std::ios_base::eofbit));
        size = is.read(buf, size).gcount();
        is.clear();
    }

    ::jpeg_decompress_struct cinfo;
    ::jpeg_error_mgr jerr;
    cinfo.err = ::jpeg_std_error(&jerr);
    ::jpeg_create_decompress(&cinfo);

    ::jpeg_mem_src(&cinfo, reinterpret_cast<unsigned char*>(buf), size);
    auto res(::jpeg_read_header(&cinfo, TRUE));

    if (res != JPEG_HEADER_OK) {
        LOGTHROW(err4, std::runtime_error)
            << "Unable to determine size of JPEG " << path << ".";
    }

    // fine
    return math::Size2(cinfo.image_width, cinfo.image_height);
}

struct Config {
    boost::optional<std::uint16_t> textureLayer;
};

class Vts02Vts : public va::UtilityBase
{
public:
    Vts02Vts()
        : UtilityBase("vts02vts", BUILD_TARGET_VERSION)
        , createMode_(vts::CreateMode::failIfExists)
    {
    }

private:
    virtual void configuration(po::options_description &cmdline
                               , po::options_description &config
                               , po::positional_options_description &pd)
        UTILITY_OVERRIDE;

    virtual void configure(const po::variables_map &vars)
        UTILITY_OVERRIDE;

    virtual bool help(std::ostream &out, const std::string &what) const
        UTILITY_OVERRIDE;

    virtual int run() UTILITY_OVERRIDE;

    fs::path input_;
    fs::path output_;

    vts::CreateMode createMode_;

    vts::StaticProperties properties_;

    Config config_;
};

void Vts02Vts::configuration(po::options_description &cmdline
                             , po::options_description &config
                             , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("input", po::value(&input_)->required()
         , "Path to input (vts0) tile set.")
        ("output", po::value(&output_)->required()
         , "Path to output (vts) tile set.")
        ("overwrite", "Existing tile set gets overwritten if set.")

        ("referenceFrame", po::value(&properties_.referenceFrame)->required()
         , "Reference frame.")

        ("textureLayer", po::value<std::string>()
         , "String/numeric id of bound layer to be used as external texture "
         "in generated meshes. Turns on generation of external texture "
         "coordinates if set.")
        ;

    registryConfiguration(cmdline);

    pd.add("input", 1);
    pd.add("output", 1);

    (void) config;
}

void Vts02Vts::configure(const po::variables_map &vars)
{
    registryConfigure(vars);

    createMode_ = (vars.count("overwrite")
                   ? vts::CreateMode::overwrite
                   : vts::CreateMode::failIfExists);

    if (vars.count("textureLayer")) {
        auto value(vars["textureLayer"].as<std::string>());

        vr::BoundLayer layer;
        try {
            layer = vr::Registry::boundLayer(boost::lexical_cast<int>(value));
        } catch (boost::bad_lexical_cast) {
            layer = vr::Registry::boundLayer(value);
        }

        if (layer.type != vr::BoundLayer::Type::raster) {
            throw po::validation_error
                (po::validation_error::invalid_option_value, "textureLayer");
        }
        config_.textureLayer = layer.numericId;
    }
}

bool Vts02Vts::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(vts02vts
usage
    vts02vts INPUT OUTPUT [OPTIONS]

)RAW";
    }
    return false;
}

typedef vts::opencv::NavTile NavTile;

class Encoder : public vts::Encoder {
public:
    Encoder(const boost::filesystem::path &path
            , const vts::StaticProperties &properties, vts::CreateMode mode
            , const vts0::TileSet::pointer &input
            , const Config &config)
        : vts::Encoder(path, properties, mode)
        , config_(config)
        , input_(input), aa_(input_->advancedApi())
        , ti_(aa_.tileIndex()), cti_(ti_)
    {
        cti_.makeFull().makeComplete();
        // set constraints: from zero to max LOD
        setConstraints(Constraints()
                       .setLodRange
                       (vs::LodRange(0, input->lodRange().max)));

        // generate navigation tiles from original metanodes' heightmaps
        hm2Navtile();
    }

private:
    virtual TileResult
    generate(const vts::TileId &tileId
             , const vr::ReferenceFrame::Division::Node &node
             , const math::Extents2 &divisionExtents) UTILITY_OVERRIDE;

    virtual void finish(vts::TileSet&) UTILITY_OVERRIDE {}

    void hm2Navtile();

    const Config config_;

    vts0::TileSet::pointer input_;
    vts0::TileSet::AdvancedApi aa_;
    const vts0::TileIndex &ti_;
    vts0::TileIndex cti_;

    typedef std::map<vts::TileId, NavTile::pointer> NavTiles;
    NavTiles navtiles_;
};

vts0::Mesh loadMesh(const vs::IStream::pointer &is)
{
    auto mesh(va::loadBinaryMesh(is->get()));
    is->close();
    return mesh;
}

class Atlas : public vts::Atlas {
public:
    Atlas(const vs::IStream::pointer &stream)
        : stream_(stream), area_(math::area(jpegSize(*stream, stream->name())))
    {}

private:
    virtual std::size_t size() const UTILITY_OVERRIDE { return 1; }

    virtual vts::multifile::Table serialize_impl(std::ostream &os) const
        UTILITY_OVERRIDE
    {
        stream_->get().seekg(0);
        auto start(os.tellp());
        os << stream_->get().rdbuf();
        auto end(os.tellp());

        vts::multifile::Table table;
        table.entries.emplace_back(start, end - start);
        return table;
    }

    virtual void deserialize_impl(std::istream&
                                  , const boost::filesystem::path&
                                  , const vts::multifile::Table&)
        UTILITY_OVERRIDE
    {
        LOGTHROW(err4, std::runtime_error)
            << "This atlas is serialize-only.";
    }

    virtual std::size_t area(std::size_t index) const {
        UTILITY_OVERRIDE
        if (index) {
            LOGTHROW(err4, std::runtime_error)
                << "This atlas has just one image+.";
        }

        return area_;
    }

    vs::IStream::pointer stream_;
    std::size_t area_;
};

inline vts0::TileId asVts(const vts::TileId tileId) {
    return vts0::TileId(tileId.lod, tileId.x, tileId.y);
}

inline vts::TileId asVts0(const vts0::TileId tileId) {
    return vts::TileId(tileId.lod, tileId.x, tileId.y);
}

class TextureNormalizer {
public:
    TextureNormalizer(const math::Extents2 &divisionExtents)
        : size_(size(divisionExtents))
        , origin_(divisionExtents.ll)
    {}

    math::Point2 operator()(const math::Point3 &p) const {
        // NB: origin is in the upper-left corner
        return { (p(0) - origin_(0)) / size_.width
                , 1.0 - ((p(1) - origin_(1)) / size_.height) };
    };

private:
    math::Size2f size_;
    math::Point2 origin_;
};

/** Constructs transformation matrix that maps everything in extents into a grid
 *  of defined size so the grid (0, 0) matches to upper-left extents corner and
 *  grid(gridSize.width - 1, gridSize.width - 1) matches lower-rightextents
 *  corner.
 *
 *  Optional offset allows us to map grid into larger grid of same coarsenes at
 *  given position is such grid.
 */
inline math::Matrix4 geo2grid(const math::Extents2& extents
                              , const math::Size2 &gridSize
                              , const math::Size2 &offset = math::Size2())
{
    math::Matrix4 trafo(ublas::identity_matrix<double>(4));

    auto es(size(extents));

    // scales
    math::Size2f scale((gridSize.width - 1) / es.width
                       , (gridSize.height - 1) / es.height);

    // scale to grid
    trafo(0, 0) = scale.width;
    trafo(1, 1) = -scale.height;

    // place zero to upper-left corner
    trafo(0, 3) = gridSize.width / 2.0 + offset.width;
    trafo(1, 3) = gridSize.height / 2.0 + offset.height;

    return trafo;
}

/** Rasterizes mesh to generate mesh mask and mesh heightmap.
 */
void rasterizeMesh(const vts::TileId &tileId, const math::Extents2 &extents
                   , const vts0::Mesh &mesh, vts::Mesh::CoverageMask &cm
                   , NavTile::pointer &navtile)
{
    (void) tileId;

    // do not change
    const int scale(2);

    const auto cms(vts::Mesh::coverageSize());

    // We need to create pane with 2x the size of mask/navtile; since
    // both are in grid registration and thus they have shared edges with
    // neighbouring tiles we must double the subtract 1 from both dimenstions at
    // each level
    math::Size2 sourceSize(2 * cms.width - 1, 2 * cms.height - 1);

    // since we are about to filter this down back to original size we have to
    // add filter radius to each direction; radius is 2x scale -> 4

    const int radius(2 * scale);
    const math::Size2 rasterSize(sourceSize.width + 2 * radius
                                 , sourceSize.height + 2 * radius);

    LOG(info2) << "rasterSize: " << rasterSize;
    // NB: sourceSize covers extents, raster size are a bit bigger

    // build trafo to map tile data into raster
    auto trafo(geo2grid(extents, sourceSize, math::Size2(radius, radius)));
    LOG(info2) << "trafo: " << trafo;

    // build heights and mask matrices
    cv::Mat heights(rasterSize.height, rasterSize.width, CV_64FC1);
    heights = cv::Scalar(0);
    cv::Mat mask(rasterSize.height, rasterSize.width, CV_8UC1);
    mask = cv::Scalar(0);

    // draw all faces into the heightmap and mask
    {
        std::vector<imgproc::Scanline> scanlines;
        cv::Point3f tri[3];
        for (const auto &face : mesh.facets) {
            for (int i : { 0, 1, 2 }) {
                auto p(transform(trafo, mesh.vertices[face.v[i]]));
                tri[i].x = p(0); tri[i].y = p(1); tri[i].z = p(2);
            }

            scanlines.clear();
            imgproc::scanConvertTriangle(tri, 0, heights.rows, scanlines);

            for (const auto &sl : scanlines) {
                imgproc::processScanline(sl, 0, heights.cols
                                         , [&](int x, int y, float z)
                {
                    // remember height
                    auto &height(heights.at<double>(y, x));
                    if (z > height) { height = z; }

                    // remember mask
                    mask.at<unsigned char>(y, x) = 255;
                });
            }
        }
    }

    // scale-down mask into cm
    for (int j(0), y(radius), ej(cm.size().height); j < ej; ++j, y += 2) {
        for (int i(0), x(radius), ei(cm.size().width); i < ei; ++i, x += 2) {
            // 3x3 submatrix at given location (nb, end is exclusive!)
            cv::Range yr(y - 1, y + 2);
            cv::Range xr(x - 1, x + 2);

            // we need to skip out-of mask values (otherwise we remove border
            // pixels in mask that is cut exactly to extents!)
            if (!j) { ++yr.start; } else if ((j + 1) == ej) {--yr.end; }
            if (!i) { ++xr.start; } else if ((i + 1) == ei) {--xr.end; }

            // calculate minimum in given submatrix
            double min;
            cv::minMaxIdx(cv::Mat(mask, yr, xr), &min, nullptr);
            // reset mask if minimum is zero
            if (!min) { cm.set(i, j, false); }
        }
    }

    if (navtile) { return; }
    navtile.reset(new NavTile);

    // use same mask as mesh mask
    navtile->coverageMask(cm);

    // TODO: pixel-grow data in heights using mask to have nice data in navtile

    // get kernel from low-pass filter
    auto kernel(math::LowPassFilter_t(radius, radius).getKernel());

    // temporary buffer for horizontal filtering
    cv::Mat buffer(rasterSize.height, cms.width, CV_64FC1);

    // horizontal filtering
    // for every row in output
    for (int j(0); j < buffer.rows; ++j) {
        // and for every column in buffer
        for (int i(0); i < buffer.cols; ++i) {
            // and for every item in kernel

            // calculate convolution
            int x(2 * i);
            double sum(0), weight(0);
            for (const auto f : kernel) {
                // if unmasked -> take into account
                if (mask.at<unsigned char>(j, x)) {
                    sum += f * heights.at<double>(j, x);
                    weight += f;
                }
                ++x;
            }

            // write filtered value
            if (weight) {
                buffer.at<double>(j, i) = sum / weight;
            }
        }
    }

    // output heightmap
    auto &hm(navtile->data());

    // vertical filtering
    // and for every column in output
    for (int i(0); i < hm.cols; ++i) {
        // and for every row in output
        for (int j(0); j < hm.rows; ++j) {
            // and for every item in kernel

            // calculate convolution
            int y(2 * j);
            double sum(0), weight(0);
            for (const auto f : kernel) {
                // if unmasked -> take into account
                if (mask.at<unsigned char>(y, 2 * i + radius)) {
                    sum += f * buffer.at<double>(y, i);
                    weight += f;
                }
                ++y;
            }

            // write filtered value
            if (weight) {
                hm.at<double>(j, i) = sum / weight;
            }
        }
    }
}

vts::Mesh::pointer
createMeshAndNavtile(const vts::TileId &tileId, const vts0::Mesh &m
                     , const geo::SrsDefinition &srcSrs
                     , const vr::Srs &dstSrs
                     , const math::Extents2 &divisionExtents
                     , bool externalTextureCoordinates
                     , boost::optional<std::uint16_t> textureLayer
                     , NavTile::pointer &navtile)
{
    // just one submesh
    auto mesh(std::make_shared<vts::Mesh>());
    mesh->submeshes.emplace_back();
    auto &sm(mesh->submeshes.back());

    // copy vertices
    geo::CsConvertor conv(srcSrs, dstSrs.srsDef);
    TextureNormalizer tn(divisionExtents);
    auto t2g(geo::local2geo(divisionExtents));
    for (const auto &v : m.vertices) {
        // convert v from local coordinates to division SRS and than to physical
        // SRS (the last transformation can be no-op)
        sm.vertices.push_back(conv(transform(t2g, v)));

        // generate external texture coordinates if instructed
        if (externalTextureCoordinates) { sm.etc.push_back(tn(v)); }
    }

    if (externalTextureCoordinates) {
        sm.textureLayer = textureLayer;
    }

    // copy texture coordinates
    std::transform(m.texcoords.begin(), m.texcoords.end()
                   , std::back_inserter(sm.tc)
                   , [&](const math::Point3 &p)
    {
        return math::Point2(p(0), p(1));
    });

    for (const auto &f : m.facets) {
        sm.faces.emplace_back(f.v[0], f.v[1], f.v[2]);
        sm.facesTc.emplace_back(f.t[0], f.t[1], f.t[2]);
    }

    // create mesh mask and navtile
    rasterizeMesh(tileId, divisionExtents, m, mesh->coverageMask, navtile);

    // done
    return mesh;
}

void Encoder::hm2Navtile()
{
    aa_.traverseTiles([&](const vts0::TileId &vts0Id)
    {
        // cannot go above root :)
        if (vts0Id.lod < 8) { return; }

        vts::TileId tileId(vts0Id.lod - 8, vts0Id.x >> 8, vts0Id.y >> 8);
        unsigned int x(vts0Id.x & 0xff);
        unsigned int y(vts0Id.y & 0xff);

        // get navtile and create if missing
        auto &nt(navtiles_[tileId]);
        if (!nt) {
            nt.reset(new NavTile);
            // clear mask
            nt->coverageMask.reset(false);
        }

        // set pixel at proper index to value read from center of tile's
        // heightmap
        nt->data().at<double>(y, x)
            = input_->getMetadata(vts0Id)
            .heightmap[vts0::TileMetadata::HMSize / 2]
            [vts0::TileMetadata::HMSize / 2];

        // set mask
        nt->coverageMask.set(x, y);
    });
}

Encoder::TileResult
Encoder::generate(const vts::TileId &tileId
                  , const vr::ReferenceFrame::Division::Node &node
                  , const math::Extents2 &divisionExtents)
{
    const auto nodeSrs(vr::Registry::srs(node.srs));
    geo::SrsDefinition spatialDivisionSrs(nodeSrs.srsDef);

    auto vts0Id(asVts(tileId));

    if (!cti_.exists(vts0Id)) {
        // neither this nor any child tile exists -> no data
        return TileResult::Result::noData;
    }

    // get generated navtile
    auto fnavtiles(navtiles_.find(tileId));

    NavTile::pointer navtile;
    if (fnavtiles != navtiles_.end()) { navtile = fnavtiles->second; }

    if (!ti_.exists(vts0Id)) {
        // check for generated navtile
        if (!navtile) {
            // nothing here, continue down
            return TileResult::Result::noDataYet;
        }

        // OK, set navtile
        vts::Encoder::TileResult result(TileResult::Result::data);
        result.tile.navtile = fnavtiles->second;
        return result;
    }

    // load mesh; NB: mesh is in space division srs, just convert to physical
    // and here we go
    vts0::Mesh mesh;
    vs::IStream::pointer atlasStream;

    UTILITY_OMP(critical)
    {
        // NB: access to tileset is not thread safe!
        mesh = loadMesh(aa_.input(vts0Id, vs::TileFile::mesh));
        atlasStream = aa_.input(vts0Id, vs::TileFile::atlas);
    }

    vts::Encoder::TileResult result(TileResult::Result::data);
    auto &tile(result.tile);

    // use original atlas (file is piped)
    tile.atlas = std::make_shared<Atlas>(atlasStream);

    // convert mesh from old one
    tile.mesh = createMeshAndNavtile(tileId, mesh, spatialDivisionSrs
                                     , physicalSrs(), divisionExtents
                                     , bool(node.boundLayerLod)
                                     , config_.textureLayer, navtile);

    // set navtile
    tile.navtile = navtile;

    return result;
}

int Vts02Vts::run()
{
    // open vts0 tileset
    auto input(vts0::openTileSet(input_));

    properties_.id = input->getProperties().id;

    // TODO: make checks here

    // run the encoder
    Encoder(output_, properties_, createMode_, input, config_).run();

    // all done
    LOG(info4) << "All done.";
    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    return Vts02Vts()(argc, argv);
}
