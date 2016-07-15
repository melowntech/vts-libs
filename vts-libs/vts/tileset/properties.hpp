/**
 * \file vts/tileset/properties.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vadstena_libs_vts_tileset_properties_hpp_included_
#define vadstena_libs_vts_tileset_properties_hpp_included_

#include <map>

#include <boost/any.hpp>
#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

#include "../../storage/credits.hpp"
#include "../../registry.hpp"

#include "../mapconfig.hpp"

namespace vadstena { namespace vts {

typedef std::map<std::string, math::Extents2> SpatialDivisionExtents;

/** Tile set properties that must be specified during creation. They cannot be
 *  changed later.
 */
struct TileSetProperties {
    /** Unique set identifier.
     */
    std::string id;

    /** Tileset's reference frame
     */
    std::string referenceFrame;

    /** Set of credits.
     */
    registry::IdSet credits;

    /** Set of bound layers.
     */
    registry::IdSet boundLayers;

    /** Position.
     */
    registry::Position position;

    TileSetProperties() {}
};

struct FullTileSetProperties : TileSetProperties {
    /** Data version/revision. Should be increment anytime the data change.
     *  Used in template URL's to push through caches.
     */
    unsigned int revision;

    // driver options (interpreted by driver)
    boost::any driverOptions;

    /** Range of lods where are tiles with mesh and/or atlas.
     */
    storage::LodRange lodRange;

    /** Extents (inclusive) of tiles with mesh and/or atlas at lodRange.min
     */
    TileRange tileRange;

    /** Occupied extents for each spatial division SRS
     */
    SpatialDivisionExtents spatialDivisionExtents;

    FullTileSetProperties() : revision(0) {}

    FullTileSetProperties(const TileSetProperties &slice)
        : TileSetProperties(slice), revision(0)
    {}
};

struct ExtraTileSetProperties {
    /** Override position.
     */
    boost::optional<registry::Position> position;

    /** Texture layer to add (automatically added to bound layers).
     */
    boost::optional<std::string> textureLayer;

    /** ROI server(s) definition.
     */
    registry::Roi::list rois;

    /** Initial view.
     */
    registry::View view;

    /** Named views.
     */
    registry::View::map namedViews;

    /** Credits definition to include in the output.
     */
    registry::Credit::dict credits;

    /** Bound layers definition to include in the output.
     */
    registry::BoundLayer::dict boundLayers;

    /** Browser core options. Opaque structure.
     */
    std::shared_ptr<BrowserOptions> browserOptions;

    // TODO: freeLayers

    ExtraTileSetProperties() {}
};

MapConfig mapConfig(const FullTileSetProperties &properties
                    , const ExtraTileSetProperties &extra
                    = ExtraTileSetProperties()
                    , const boost::optional<boost::filesystem::path> &root
                    = boost::none);

MeshTilesConfig
meshTilesConfig(const FullTileSetProperties &properties
                , const ExtraTileSetProperties &extra
                = ExtraTileSetProperties()
                , const boost::optional<boost::filesystem::path> &root
                = boost::none);

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_tileset_properties_hpp_included_
