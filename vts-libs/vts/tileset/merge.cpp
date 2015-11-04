#include <set>

#include <opencv2/highgui/highgui.hpp>

#include "imgproc/rastermask/cvmat.hpp"

#include "./merge.hpp"
#include "../io.hpp"

namespace vadstena { namespace vts { namespace merge {

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

const opencv::Atlas& Input::atlas() const
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

namespace detail {
auto cvCoverageType(CV_16S);
typedef std::int16_t coverageType;
} // namespace detail

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

struct Contributors {
    const Input::list &sources;
    bool hasHoles;
    std::vector<bool> indices;
    boost::optional<Input::Id> single;

    Contributors(const Input::list &sources
                 , const cv::Mat &coverage)
        : sources(sources), hasHoles(false)
        , indices(sources.back().id() + 1, false)
    {
        analyze(coverage);
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

private:
    void analyze(const cv::Mat &coverage) {
        for (auto j(0); j < coverage.rows; ++j) {
            for (auto i(0); i < coverage.cols; ++i) {
                auto v(coverage.at<detail::coverageType>(j, i));
                if (v == -1) {
                    hasHoles = true;
                } else {
                    indices[v] = true;
                }
            }
        }

        for (Input::Id id(0), eid(indices.size()); id != eid; ++id) {
            if (!indices[id]) { continue; }
            if (single) {
                single = boost::none;
                break;
            }
            single = id;
        }
    }
};

} // namespace

Output mergeTile(const Input::list &currentSource
                 , const Input::list &parentSource
                 , int quadrant)
{
    // build uniform source by merging current and parent sources
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
                source.push_back(ts);
                ++icurrentSource;
            } else if (ps < ts) {
                source.push_back(ps);
                ++iparentSource;
            } else {
                source.push_back(ts);
                ++icurrentSource;
                ++iparentSource;
            }
        }

        // copy tail (one or another)
        source.insert(source.end(), icurrentSource, ecurrentSource);
        source.insert(source.end(), iparentSource, eparentSource);
    }

    for (const auto &s : source) {
        LOG(info4) << "source tile owner=" << s.id()
                   << ", tileId=" << s.tileId();
    }

    // merge result
    Output result;

    if (source.empty()) { return result; }

    if (source.size() == 1) {
        // just copy one source
        const auto &input(source.front());
        result.mesh = input.mesh();
        if (input.hasAtlas()) { result.atlas = input.atlas(); }
        if (input.hasNavtile()) { result.navtile = input.navtile(); }
        return result;
    }

    // prepare coverage map (set to invalid index)
    auto coverageSize(Mesh::coverageSize());
    cv::Mat coverage(coverageSize.height, coverageSize.width
                     , detail::cvCoverageType);
    coverage = cv::Scalar(-1);

    for (const auto &input : source) {
        if (!input.hasMesh()) { continue; }
        rasterize(input.mesh(), input.id(), coverage);
    }

    // analyze coverage
    Contributors contr(source, coverage);

    // get contributing tile sets
    contr.getSources(result);

    if (contr.single) {
        // single source
        if (*contr.single < 0) {
            // nothing at all
            return result;
        }

        // just one source -> just copy
        if (const auto input = contr.getSingle()) {
            result.mesh = input->mesh();
            if (input->hasAtlas()) { result.atlas = input->atlas(); }
            if (input->hasNavtile()) { result.navtile = input->navtile(); }
        }

        // OK
        return result;
    }

    // TODO: merge mesh based on coverage
    // TODO: merge navtile based on navtile coverage

    return result;

    (void) quadrant;
}

} } } // namespace vadstena::merge::vts
