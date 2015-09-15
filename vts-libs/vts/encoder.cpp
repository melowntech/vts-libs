#include <boost/format.hpp>

#include "utility/openmp.hpp"

#include "../vts.hpp"

#include "./tileop.hpp"
#include "./io.hpp"
#include "./encoder.hpp"

namespace vadstena { namespace vts {

struct Encoder::Detail {
    Detail(Encoder *owner, const boost::filesystem::path &path
           , const StaticProperties &properties, CreateMode mode)
        : owner(owner), tileSet(createTileSet(path, properties, mode))
        , properties(tileSet.getProperties())
        , referenceFrame(tileSet.referenceFrame())
        , physicalSrs(storage::Registry::srs
                      (referenceFrame.model.physicalSrs).srsDef)
    {}

    void run()
    {
        UTILITY_OMP(parallel)
        UTILITY_OMP(single)
        process(TileId(), Constraints::all, &referenceFrame.root(), 0);

        // let the caller finish the tileset
        owner->finish(tileSet);

        // flush result
        tileSet.flush();
    }

    typedef storage::ReferenceFrame::Division::Node Node;

    void process(const TileId &tileId, int useConstraints
                 , const Node *node, Lod lodFromNode);

    Encoder *owner;
    TileSet tileSet;
    StaticProperties properties;
    storage::ReferenceFrame referenceFrame;
    geo::SrsDefinition physicalSrs;

    Constraints constraints;
};

void Encoder::Detail::process(const TileId &tileId, int useConstraints
                              , const Node *node, Lod lodFromNode)
{
    struct TIDGuard {
        TIDGuard(const std::string &id)
            : old(dbglog::thread_id())
        {
            dbglog::thread_id(id);
        }
        ~TIDGuard() { dbglog::thread_id(old); }

        const std::string old;
    };

    bool processTile(true);

    if ((useConstraints & Constraints::useLodRange)
        && constraints.lodRange)
    {
        if (tileId.lod < constraints.lodRange->min) {
            // no data yet -> go directly down
            // * equivalent to TileResult::noDataYet
            processTile = false;
        }
        if (tileId.lod > constraints.lodRange->max) {
            // nothing can live down here -> done
            // * equivalent to TileResult::noData
            return;
        }
    }

    TIDGuard tg(str(boost::format("tile:%d-%d-%d")
                    % (unsigned int)(tileId.lod) % tileId.x % tileId.y));

    // determine tile extents
    auto tc(tileCount(lodFromNode));
    auto rs(size(node->extents));
    math::Size2f ts(rs.width / tc, rs.height / tc);
    math::Extents2 divisionExtents
        (node->extents.ll(0) + tileId.x * ts.width
         , node->extents.ur(1) - (tileId.y + 1) * ts.height
         , node->extents.ll(0) + (tileId.x + 1) * ts.width
         , node->extents.ur(1) - tileId.y * ts.height);

    LOG(info3)
        << "Processing " << tileId << " (extents: "
        << std::fixed << divisionExtents << ").";

    if ((useConstraints & Constraints::useExtents)
        && constraints.extents) {
        if (!overlaps(*constraints.extents, divisionExtents)) {
            // nothing can live out here -> done
            // * equivalent to TileResult::noData
            return;
        }
    }

    if (processTile) {
        auto tile(owner->generate(tileId, *node, divisionExtents));
        switch (tile.result) {
        case TileResult::Result::data:
            UTILITY_OMP(critical)
            tileSet.setTile(tileId, tile.tile);

            // we hit a valid tile -> do not apply extents for children
            useConstraints &= ~Constraints::useExtents;
            break;

        case TileResult::Result::noDataYet:
            // fine, something could be down there
            return;

        case TileResult::Result::noData:
            // no data and nothing will ever be there
            return;
        }
    }

    // we can proces children -> go down
    for (auto child : children(tileId)) {
        // find node for this child
        // TODO: use manual/bisection information
        const auto *childNode
            (referenceFrame.find({child.lod, child.x, child.y}, std::nothrow));

        UTILITY_OMP(task)
        process(child, useConstraints
                , (childNode ? childNode : node)
                , (childNode ? 0 : lodFromNode + 1));
    }
}

Encoder::Encoder(const boost::filesystem::path &path
                 , const StaticProperties &properties, CreateMode mode)
    : detail_(std::make_shared<Detail>(this, path, properties, mode))
{}

StaticProperties Encoder::properties() const
{
    return detail_->properties;
}

const storage::ReferenceFrame& Encoder::referenceFrame() const
{
    return detail_->referenceFrame;
}

const geo::SrsDefinition& Encoder::physicalSrs() const
{
    return detail_->physicalSrs;
}

void Encoder::setConstraints(const Constraints &constraints)
{
    detail_->constraints = constraints;
}

void Encoder::run()
{
    detail_->run();
}

} } // namespace vadstena::vts
