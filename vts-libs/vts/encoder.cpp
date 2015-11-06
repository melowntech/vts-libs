#include <boost/format.hpp>

#include "utility/openmp.hpp"

#include "../vts.hpp"

#include "./tileop.hpp"
#include "./io.hpp"
#include "./encoder.hpp"

namespace vadstena { namespace vts {

namespace {
    struct ConstraintsFlag {
        typedef std::uint32_t type;
        enum : type {
            useLodRange = 0x01
            , useExtents = 0x02
            , clearExtentsOnHit = 0x04

            , all = (useLodRange | useExtents)
        };

        static type build(const Encoder::Constraints &c) {
            type flags(0);

            if (c.lodRange) { flags |= useLodRange; }
            if (c.extents) { flags |= useExtents; }
            if (c.useExtentsForFirstHit) { flags |= clearExtentsOnHit; }

            return flags;
        }

        static void clearExtents(type &flags, bool value) {
            if (value && (flags & clearExtentsOnHit)) {
                flags &= ~useExtents;
            }
        }
    };
} // namespace

struct Encoder::Detail {
    Detail(Encoder *owner, const boost::filesystem::path &path
           , const TileSetProperties &properties, CreateMode mode)
        : owner(owner), tileSet(createTileSet(path, properties, mode))
        , properties(tileSet.getProperties())
        , referenceFrame(tileSet.referenceFrame())
        , physicalSrs(registry::Registry::srs
                      (referenceFrame.model.physicalSrs))
    {}

    void run()
    {
        UTILITY_OMP(parallel)
        UTILITY_OMP(single)
        process(TileId(referenceFrame.division.rootLod, 0, 0)
                , ConstraintsFlag::build(constraints)
                , NodeInfo(referenceFrame));

        // let the caller finish the tileset
        owner->finish(tileSet);

        // flush result
        tileSet.flush();
    }

    typedef registry::ReferenceFrame::Division::Node Node;

    void process(const TileId &tileId, ConstraintsFlag::type useConstraints
                 , const NodeInfo &nodeInfo);

    Encoder *owner;
    TileSet tileSet;
    TileSetProperties properties;
    registry::ReferenceFrame referenceFrame;
    registry::Srs physicalSrs;

    Constraints constraints;
};

void Encoder::Detail::process(const TileId &tileId
                              , ConstraintsFlag::type useConstraints
                              , const NodeInfo &nodeInfo)
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

    if (useConstraints & ConstraintsFlag::useLodRange) {
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

    const auto &extents(nodeInfo.node.extents);
    if (useConstraints & ConstraintsFlag::useExtents) {
        if (!overlaps(*constraints.extents, extents)) {
            // nothing can live out here -> done
            // * equivalent to TileResult::noData
            return;
        }
    }

    if (processTile) {
        LOG(info2)
            << "Trying to generate " << tileId << " (extents: "
            << std::fixed << extents << ").";

        auto tile(owner->generate(tileId, nodeInfo));
        switch (tile.result) {
        case TileResult::Result::data:
            LOG(info3)
                << "Generated " << tileId << " (extents: "
                << std::fixed << extents << ").";

            UTILITY_OMP(critical)
            tileSet.setTile(tileId, tile.tile, nodeInfo);

            // we hit a tile with mesh -> do not apply extents constraints for
            // children if set
            ConstraintsFlag::clearExtents
                (useConstraints, bool(tile.tile.mesh));
            break;

        case TileResult::Result::noDataYet:
            // fine, something could be down there
            break;

        case TileResult::Result::noData:
            // no data and nothing will ever be there
            return;
        }
    } else {
        LOG(info3)
            << "Falling through " << tileId << " (extents: "
            << std::fixed << extents << ").";
    }

    // we can proces children -> go down
    for (auto child : children(tileId)) {
        // compute child node
        auto childNode(nodeInfo.child(child));

        UTILITY_OMP(task)
        process(child, useConstraints, childNode);
    }
}

Encoder::Encoder(const boost::filesystem::path &path
                 , const TileSetProperties &properties, CreateMode mode)
    : detail_(std::make_shared<Detail>(this, path, properties, mode))
{}

TileSetProperties Encoder::properties() const
{
    return detail_->properties;
}

const registry::ReferenceFrame& Encoder::referenceFrame() const
{
    return detail_->referenceFrame;
}

const registry::Srs& Encoder::physicalSrs() const
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
