#include <boost/format.hpp>

#include "utility/openmp.hpp"

#include "../vts0.hpp"

#include "tileop.hpp"
#include "io.hpp"
#include "encoder.hpp"

namespace vtslibs { namespace vts0 {

struct Encoder::Detail {
    Detail(Encoder *owner, const boost::filesystem::path &path
           , const CreateProperties &properties, CreateMode mode)
        : owner(owner), tileSet(createTileSet(path, properties, mode))
        , properties(tileSet->getProperties()), srs(this->properties.srs)
    {}

    void run()
    {
        UTILITY_OMP(parallel)
        UTILITY_OMP(single)
        process(TileId(), Constraints::all);

        // let the caller finish the tileset
        owner->finish(*tileSet);

        // flush result
        tileSet->flush();
    }

    void process(const TileId &tileId, int useConstraints);

    Encoder *owner;
    TileSet::pointer tileSet;
    Properties properties;
    geo::SrsDefinition srs;

    Constraints constraints;
};

void Encoder::Detail::process(const TileId &tileId, int useConstraints)
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
        && constraints.lodRange) {
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
                    % tileId.lod % tileId.x % tileId.y));

    const auto tileExtents(extents(properties, tileId));
    LOG(info2) << "Processing tile " << tileId
               << " (extents: " << std::fixed << tileExtents << ").";

    if ((useConstraints & Constraints::useExtents)
        && constraints.extents) {
        if (!overlaps(*constraints.extents, tileExtents)) {
            // nothing can live out here -> done
            // * equivalent to TileResult::noData
            return;
        }
    }

    if (processTile) {
        Mesh mesh;
        Atlas atlas;
        TileMetadata metadata;

        switch (auto res = owner->getTile
                (tileId, tileExtents, mesh, atlas, metadata))
        {
        case TileResult::data:
        case TileResult::dataWithMetadata:
            UTILITY_OMP(critical)
            tileSet->setTile(tileId, mesh, atlas
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

Encoder::Encoder(const boost::filesystem::path &path
                 , const CreateProperties &properties, CreateMode mode)
    : detail_(std::make_shared<Detail>(this, path, properties, mode))
{}

Properties Encoder::properties() const
{
    return detail_->properties;
}

geo::SrsDefinition Encoder::srs() const
{
    return detail_->srs;
}

void Encoder::setConstraints(const Constraints &constraints)
{
    detail_->constraints = constraints;
}

void Encoder::run()
{
    detail_->run();
}

} } // namespace vtslibs::vts0
