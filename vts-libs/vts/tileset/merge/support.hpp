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

#ifndef vtslibs_vts_merge_support_hpp_included_
#define vtslibs_vts_merge_support_hpp_included_

#include <utility>

#include <boost/optional.hpp>
#include <boost/utility/in_place_factory.hpp>

#include "math/transform.hpp"
#include "math/geometry_core.hpp"
#include "math/transform.hpp"

#include "imgproc/contours.hpp"

#include "../../basetypes.hpp"
#include "../../meshop.hpp"
#include "../../csconvertor.hpp"
#include "../merge.hpp"

namespace vtslibs { namespace vts { namespace merge {

Input::list filterSources(const Input::list &reference
                          , const Input::list &sources);

/** Returns mesh vertices (vector per submesh) converted to coverage space.
 */
Vertices3List inputCoverageVertices(const Input &input
                                    , const NodeInfo &nodeInfo
                                    , const CsConvertor &conv);

/** Geo coordinates to coverage mask mapping.
 * NB: result is in pixel system: pixel centers have integral indices
 */
math::Matrix4 geo2mask(const math::Extents2 &extents
                       , const math::Size2 &gridSize);

/** Coverage mask mapping to geo coordinates.
 * NB: result is in pixel system: pixel centers have integral indices
 */
math::Matrix4 mask2geo(const math::Extents2 &extents
                       , const math::Size2 &gridSize);

/** Maps external texture coordinates from parent tile into subtile.
 *  Relationship defined by tile id, parent is a root of the tree (i.e. tile id
 *  0-0-0).
 */
math::Matrix4 etcNCTrafo(const TileId &id);

/** Maps coverage coordinate into normalized external texture coordinates.
 */
math::Matrix4 coverage2EtcTrafo(const math::Size2 &gridSize);

math::Extents2 coverageExtents(double margin = .0);

/** Physical <-> SDS mask mesh coordinate system convertor.
 */
class SdMeshConvertor : public MeshVertexConvertor {
public:
    /** Create convertor
     *
     * \param input mesh operation input
     * \param nodeInfo curren node info
     * \param tileId local tile ID.
     */
    SdMeshConvertor(const NodeInfo &nodeInfo, const TileId &tileId = TileId());

    virtual math::Point3d vertex(const math::Point3d &v) const;

    virtual math::Point2d etc(const math::Point3d &v) const;

    virtual math::Point2d etc(const math::Point2d &v) const;

    // On-demand SdMeshConvertor instantiation.
    struct Lazy;

private:
    /** Linear transformation from local coverage coordinates to node's SD SRS.
     */
    math::Matrix4 geoTrafo_;

    /** Convertor between node's SD SRS and reference frame's physical SRS.
     */
    CsConvertor geoConv_;

    /** Converts external texture coordinates between fallback tile and current
     *  tile.
     */
    math::Matrix4 etcNCTrafo_;

    /** Converts between coverage coordinates and normalized external texture
     *  coordinates.
     */
    math::Matrix4 coverage2Texture_;
};

struct SdMeshConvertor::Lazy {
public:
    Lazy(const NodeInfo &nodeInfo, const TileId &tileId)
        : factory_(Factory(nodeInfo, tileId)), convertor_(nullptr)
    {}

    Lazy(const SdMeshConvertor &convertor)
        : convertor_(&convertor)
    {}

    operator const SdMeshConvertor&() const {
        if (!convertor_) {
            own_ = *factory_;
            convertor_ = &*own_;
        }
        return *convertor_;
    }

    const SdMeshConvertor& operator()() const { return *this; }

private:
    typedef decltype(boost::in_place
                     (std::declval<NodeInfo>(), std::declval<TileId>()))
        Factory;
    boost::optional<Factory> factory_;
    mutable boost::optional<SdMeshConvertor> own_;
    mutable const SdMeshConvertor* convertor_;
};

// inlines

inline math::Matrix4 geo2mask(const math::Extents2 &extents
                              , const math::Size2 &gridSize)
{
    math::Matrix4 trafo(boost::numeric::ublas::identity_matrix<double>(4));

    auto es(size(extents));

    // scales
    math::Size2f scale(gridSize.width / es.width
                       , gridSize.height / es.height);

    // scale to grid
    trafo(0, 0) = scale.width;
    trafo(1, 1) = -scale.height;

    // move to origin and also move pixel centers to integral indices
    trafo(0, 3) = -extents.ll(0) * scale.width - 0.5;
    trafo(1, 3) = extents.ur(1) * scale.height - 0.5;

    return trafo;
}

inline math::Matrix4 mask2geo(const math::Extents2 &extents
                              , const math::Size2 &gridSize)
{
    math::Matrix4 trafo(boost::numeric::ublas::identity_matrix<double>(4));

    auto es(size(extents));

    // scales
    math::Size2f scale(es.width / gridSize.width
                       , es.height / gridSize.height);

    // scale to grid
    trafo(0, 0) = scale.width;
    trafo(1, 1) = -scale.height;

    // move to origin
    trafo(0, 3) = extents.ll(0) + 0.5 * scale.width;
    trafo(1, 3) = extents.ur(1) - 0.5 * scale.height;

    return trafo;
}

inline math::Matrix4 etcNCTrafo(const TileId &id)
{
    math::Matrix4 trafo(boost::numeric::ublas::identity_matrix<double>(4));

    // LOD=0 -> identity
    if (!id.lod) { return trafo; }

    double tileCount(1 << id.lod);

    // number of tiles -> scale
    trafo(0, 0) = tileCount;
    trafo(1, 1) = tileCount;

    // NB: id.x is unsigned -> must cast to double first
    trafo(0, 3) = - double(id.x);
    trafo(1, 3) = (id.y + 1) - tileCount;

    return trafo;
}

inline math::Matrix4 coverage2EtcTrafo(const math::Size2 &gridSize)
{
    math::Matrix4 trafo(boost::numeric::ublas::identity_matrix<double>(4));

    // scale to normalized range (0, 1)
    trafo(0, 0) = 1.0 / gridSize.width;
    trafo(1, 1) = -1.0 / gridSize.height;

    // shift to proper orientation and cancel halfpixel offset
    trafo(0, 3) = 0.5 / gridSize.width;
    trafo(1, 3) = 1 - 0.5 / gridSize.height;

    return trafo;
}

inline math::Extents2 coverageExtents(double margin)
{
    const auto grid(Mesh::coverageSize());
    return math::Extents2(-.5 - margin
                          , -.5 - margin
                          , grid.width - .5 + margin
                          , grid.height - .5 + margin);
}

inline SdMeshConvertor::SdMeshConvertor(const NodeInfo &nodeInfo
                                        , const TileId &tileId)
    : geoTrafo_(Input::coverage2Sd(nodeInfo))
    , geoConv_(nodeInfo.srs()
               , nodeInfo.referenceFrame().model.physicalSrs)
    , etcNCTrafo_(etcNCTrafo(tileId))
    , coverage2Texture_(Input::coverage2Texture())
{}

inline math::Point3d SdMeshConvertor::vertex(const math::Point3d &v) const
{
    // point is in node SD SRS
    return geoConv_(transform(geoTrafo_, v));
}

inline math::Point2d SdMeshConvertor::etc(const math::Point3d &v) const
{
    // point is in projected space (i.e. in coverage raster)
    auto tmp(transform(coverage2Texture_, v));
    return math::Point2d(tmp(0), tmp(1));
}

inline math::Point2d SdMeshConvertor::etc(const math::Point2d &v) const
{
    // point is in the input's texture coordinates system
    return transform(etcNCTrafo_, v);
}

} } } // namespace vtslibs::vts::merge

#endif // vtslibs_vts_merge_support_hpp_included_
