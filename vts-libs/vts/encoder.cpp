#include <atomic>

#include <boost/format.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/openmp.hpp"

#include "../vts.hpp"

#include "./tileop.hpp"
#include "./io.hpp"
#include "./encoder.hpp"
#include "./csconvertor.hpp"
#include "../storage/error.hpp"

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

void Encoder::TileResult::fail(const char *what) const
{
    LOGTHROW(err1, std::runtime_error)
        << "Encoder: " << what << ".";
}

struct Encoder::Detail {
    Detail(Encoder *owner, const boost::filesystem::path &path
           , const TileSetProperties &properties, CreateMode mode)
        : owner(owner), tileSet(createTileSet(path, properties, mode))
        , properties(tileSet.getProperties())
        , referenceFrame(tileSet.referenceFrame())
        , physicalSrs(registry::Registry::srs
                      (referenceFrame.model.physicalSrs))
        , navigationSrs(registry::Registry::srs
                      (referenceFrame.model.navigationSrs))
        , generated_(0)
    {}

    TileSet run()
    {
        UTILITY_OMP(parallel)
        UTILITY_OMP(single)
        {
            owner->threadCount(omp_get_num_threads());
            process({}, ConstraintsFlag::build(constraints)
                    , NodeInfo(referenceFrame), {});
        }
        LOG(info3) << "VTS Encoder: generated. Finishing and flushing.";

        // let the caller finish the tileset
        owner->finish(tileSet);

        // flush result
        tileSet.flush();
        LOG(info3) << "VTS Encoder: done.";

        // done
        return tileSet;
    }

    void setConstraints(const Constraints &constraints);

    const math::Extents2& getExtents(const std::string &srs) const;

    typedef registry::ReferenceFrame::Division::Node Node;

    void process(const TileId &tileId, ConstraintsFlag::type useConstraints
                 , const NodeInfo &nodeInfo, const TileResult &parentTile);

    Encoder *owner;
    TileSet tileSet;
    TileSetProperties properties;
    registry::ReferenceFrame referenceFrame;
    const registry::Srs &physicalSrs;
    const registry::Srs &navigationSrs;

    Constraints constraints;

    typedef std::map<std::string, math::Extents2> SrsExtentsMap;
    SrsExtentsMap srsExtents;

    std::atomic<std::size_t> generated_;
};

void Encoder::Detail::setConstraints(const Constraints &c)
{
    constraints = c;
    if (!constraints.extents) { return; }

    for (const auto &srs : referenceFrame.division.srsList()) {
        srsExtents[srs] = CsConvertor(constraints.extents->srs, srs)
            (constraints.extents->extents);
    }
}

inline const math::Extents2&
Encoder::Detail::getExtents(const std::string &srs) const
{
    auto fsrsExtents(srsExtents.find(srs));
    if (fsrsExtents == srsExtents.end()) {
        LOGTHROW(err2, storage::Error)
            << "Inconsistency: no extents constraints for srs "
            << " <" << srs << ">.";
    }
    return fsrsExtents->second;
}

void Encoder::Detail::process(const TileId &tileId
                              , ConstraintsFlag::type useConstraints
                              , const NodeInfo &nodeInfo
                              , const TileResult &parentTile)
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

    if (constraints.validTree && !constraints.validTree->get(tileId)) {
        // tile not in valid tree -> stop
        return;
    }

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

    TIDGuard tg(str(boost::format("tile:%s") % tileId));

    const auto &extents(nodeInfo.node.extents);
    if (useConstraints & ConstraintsFlag::useExtents) {
        if (!overlaps(getExtents(nodeInfo.node.srs), extents)) {
            // nothing can live out here -> done
            // * equivalent to TileResult::noData
            return;
        }
    }

    TileResult tile;
    if (processTile) {
        LOG(info2)
            << "Trying to generate " << tileId << " (extents: "
            << std::fixed << extents << ").";

        tile = owner->generate(tileId, nodeInfo, parentTile);
        switch (auto result = tile.result()) {
        case TileResult::Result::tile:
        case TileResult::Result::source:
        {
            auto number(++generated_);

            LOG(info3)
                << "Generated tile #" << number << ": "
                << tileId << " (extents: " << std::fixed << extents << ").";

            bool hasMesh(false);
            if (result == TileResult::Result::tile) {
                const auto &t(tile.tile());

                UTILITY_OMP(critical)
                tileSet.setTile(tileId, t, nodeInfo);

                hasMesh = bool(t.mesh);
            } else {
                const auto &t(tile.source());

                UTILITY_OMP(critical)
                tileSet.setTile(tileId, t, nodeInfo);

                hasMesh = bool(t.mesh);
            }

            // we hit a tile with mesh -> do not apply extents constraints for
            // children if set
            ConstraintsFlag::clearExtents(useConstraints, hasMesh);
            break;
        }

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
        process(child, useConstraints, childNode, tile);
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

const std::string& Encoder::physicalSrsId() const
{
    return detail_->referenceFrame.model.physicalSrs;
}

const registry::Srs& Encoder::physicalSrs() const
{
    return detail_->physicalSrs;
}

const std::string& Encoder::navigationSrsId() const
{
    return detail_->referenceFrame.model.navigationSrs;
}

const registry::Srs& Encoder::navigationSrs() const
{
    return detail_->navigationSrs;
}

void Encoder::setConstraints(const Constraints &constraints)
{
    detail_->setConstraints(constraints);
}

TileSet Encoder::run()
{
    return detail_->run();
}

std::size_t Encoder::threadIndex() const
{
    return omp_get_thread_num();
}

} } // namespace vadstena::vts
