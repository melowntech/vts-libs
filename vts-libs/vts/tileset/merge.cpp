#include "./merge.hpp"

namespace vadstena { namespace vts { namespace merge {

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
        atlas_ = {};
        owner_->getAtlas(tileId_, *atlas_, node_);
    }

    return *atlas_;
}

const opencv::NavTile& Input::navtile() const
{
    if (!navtile_) {
        navtile_ = {};
        owner_->getNavTile(tileId_, *navtile_, node_);
    }

    return *navtile_;
}

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
    }

    // TODO: merge
    Output res;

    return res;
    (void) quadrant;
}

} } } // namespace vadstena::merge::vts
