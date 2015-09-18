/**
 * \file vts/tileset/properties.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vadstena_libs_vts_tileset_properties_hpp_included_
#define vadstena_libs_vts_tileset_properties_hpp_included_

#include "../../storage/credits.hpp"
#include "../../registry.hpp"

#include "../basetypes.hpp"

namespace vadstena { namespace vts {

/** Tile set properties that must be specified during creation. They cannot be
 *  changed later.
 */
struct StaticProperties {
    /** Unique set identifier.
     */
    std::string id;

    /** Tileset's reference frame
     */
    std::string referenceFrame;

    /** Data version/revision. Should be increment anytime the data change.
     *  Used in template URL's to push through caches.
     */
    unsigned int revision;

    /** Set of credits.
     */
    registry::IdSet credits;

    /** Set of bound layers.
     */
    registry::IdSet boundLayers;

    /** Position.
     */
    registry::Position position;

    StaticProperties() : revision(0) {}
};

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_tileset_properties_hpp_included_
