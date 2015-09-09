/**
 * \file vts/tileset.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile set access.
 */

#ifndef vadstena_libs_vts_tileset_detail_hpp_included_
#define vadstena_libs_vts_tileset_detail_hpp_included_

#include "../tileset.hpp"
#include "./driver.hpp"

namespace vadstena { namespace vts {

struct TileSet::Properties : StaticProperties {
    // driver options
    driver::Options driverOptions;

    Properties() {}
};

typedef std::map<TileId, MetaNode*> MetaNodes;

typedef std::map<TileId, MetaTile> MetaTiles;

/** Driver that implements physical aspects of tile set.
 */
struct TileSet::Detail
{
    bool readOnly;

    Driver::pointer driver;

    Properties savedProperties;  // properties as are on disk
    Properties properties;       // current properties
    bool propertiesChanged;      // marks whether properties have been changed

    bool metadataChanged;

    MetaNodes metaNodes;
    MetaTiles metaTiles;

    Detail(const Driver::pointer &driver);
    Detail(const Driver::pointer &driver, const StaticProperties &properties);

    void loadConfig();

    void saveConfig();

    void watch(utility::Runnable *runnable);

    void checkValidity() const;

    MetaNode* findMetaNode(const TileId &tileId);

    MetaNode* loadMetatile(const TileId &tileId) const;
};

inline void TileSet::Detail::checkValidity() const
{
    if (!driver) {
        LOGTHROW(err2, storage::NoSuchTileSet)
            << "Tile set <" << properties.id << "> was removed.";
    }
}

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_tileset_detail_hpp_included_
