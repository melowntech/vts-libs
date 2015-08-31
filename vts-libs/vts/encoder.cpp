#include <boost/format.hpp>

#include "utility/openmp.hpp"

#include "../vts.hpp"

#include "tileop.hpp"
#include "io.hpp"
#include "encoder.hpp"

namespace vadstena { namespace vts {

Encoder::Encoder(const boost::filesystem::path &path
                 , const CreateProperties &properties, CreateMode mode)
    : tileSet_(createTileSet(path, properties, mode))
    , properties_(tileSet_->getProperties())
    , srs_(properties_.srs)
{}

void Encoder::run()
{
    UTILITY_OMP(parallel)
    UTILITY_OMP(single)
    process(TileId(), Constraints::all);

    // let the caller finish the tileset
    finish(*tileSet_);

    // flush result
    tileSet_->flush();
}

void Encoder::process(const TileId &tileId, int useConstraints)
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
        && constraints_.lodRange) {
        if (tileId.lod < constraints_.lodRange->min) {
            // no data yet -> go directly down
            // * equivalent to TileResult::noDataYet
            processTile = false;
        }
        if (tileId.lod > constraints_.lodRange->max) {
            // nothing can live down here -> done
            // * equivalent to TileResult::noData
            return;
        }
    }

    TIDGuard tg(str(boost::format("tile:%d-%d-%d")
                    % tileId.lod % tileId.x % tileId.y));

    const auto tileExtents(extents(properties_, tileId));
    LOG(info2) << "Processing tile " << tileId
               << " (extents: " << std::fixed << tileExtents << ").";

    if ((useConstraints & Constraints::useExtents)
        && constraints_.extents) {
        if (!overlaps(*constraints_.extents, tileExtents)) {
            // nothing can live out here -> done
            // * equivalent to TileResult::noData
            return;
        }
    }

    if (processTile) {
        Mesh mesh;
        Atlas atlas;
        TileMetadata metadata;

        switch (auto res = getTile(tileId, tileExtents, mesh, atlas, metadata))
        {
        case TileResult::data:
        case TileResult::dataWithMetadata:
            UTILITY_OMP(critical)
            tileSet_->setTile(tileId, mesh, atlas
                              , ((res == TileResult::dataWithMetadata)
                                 ? &metadata : nullptr));

            // we hit a valid tile -> do not apply extents for children
            useConstraints &= ~Constraints::useExtents;
            break;

        case TileResult::noDataYet:
            // fine, something could be down there
            return;

        case TileResult::noData:
            // no data and nothing will ever be there
            return;
        }
    }

    // we can proces children -> go down
    for (auto child : children(tileId)) {
        UTILITY_OMP(task)
        process(child, useConstraints);
    }
}

} } // namespace vadstena::vts
