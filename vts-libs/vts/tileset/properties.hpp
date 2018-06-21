/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \file vts/tileset/properties.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vtslibs_vts_tileset_properties_hpp_included_
#define vtslibs_vts_tileset_properties_hpp_included_

#include <map>

#include <boost/any.hpp>
#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

#include "../../storage/credits.hpp"
#include "../../registry.hpp"

#include "../mapconfig.hpp"

namespace vtslibs { namespace vts {

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

    /** Hint for merge - physically load only tiles at LODs same or above
     * for merging finer tiles use tile from this lod as source.
     * Comes handy when merging deep and large tilesets where fine LODs
     * do not bring any new information.
     */
    Lod mergeBottomLod;

    /** Nominal texelSize. Provided only for information.
     */
    boost::optional<double> nominalTexelSize;

    FullTileSetProperties() : revision(0), mergeBottomLod(0) {}

    FullTileSetProperties(const TileSetProperties &slice)
        : TileSetProperties(slice), revision(0), mergeBottomLod(0)
    {}

    template <typename T>
    T& getDriverOptions() {
        return boost::any_cast<T&>(driverOptions);
    }

    template <typename T>
    const T& getDriverOptions() const {
        return boost::any_cast<const T&>(driverOptions);
    }
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

    /** Free layers definition to include in the output.
     */
    registry::FreeLayer::dict freeLayers;

    /** Extra bodies.
     */
    registry::Body::dict bodies;

    /** Browser core options. Opaque structure.
     */
    boost::any browserOptions;

    ExtraTileSetProperties() {}
};

MapConfig mapConfig(const FullTileSetProperties &properties
                    , const registry::RegistryBase &localRegistry
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

} } // namespace vtslibs::vts

#endif // vtslibs_vts_tileset_properties_hpp_included_
