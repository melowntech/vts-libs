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
#ifndef geo_vtslibs_vts_csconvert_hpp_included_
#define geo_vtslibs_vts_csconvert_hpp_included_

#include "geo/csconvertor.hpp"
#include "geo/verticaladjuster.hpp"

#include "../registry.hpp"

namespace vtslibs { namespace vts {

/** Coordinate system convertor.
 */
class CsConvertor {
public:
    /** Dummy.
     */
    CsConvertor() = default;

    /** Creates convertor between two SRS specified as keys to global SRS
     *  registry.
     *
     *  Can result in no-op convertor when srsIdFrom == srsIdTo.
     */
    CsConvertor(const std::string &srsIdFrom, const std::string &srsIdTo
                , const registry::Registry &reg = registry::system);

    /** Creates convertor between two SRS specified as keys to global SRS
    *  registry.
    *  Uses the specified proj context.
    *
    *  Can result in no-op convertor when srsIdFrom == srsIdTo.
    */
    CsConvertor(const std::string &srsIdFrom, const std::string &srsIdTo
        , const registry::Registry &reg, projCtx ctx);

    /** Combined convertor, never results in no-op convertor.
     */
    CsConvertor(const geo::SrsDefinition &srsFrom
                , const std::string &srsIdTo
                , const registry::Registry &reg = registry::system);

    /** Combined convertor, never results in no-op convertor.
     */
    CsConvertor(const std::string &srsIdFrom
                , const geo::SrsDefinition &srsTo
                , const registry::Registry &reg = registry::system);

#ifdef GEO_HAS_GDAL
    /** Combined convertor, never results in no-op convertor.
     */
    CsConvertor(const ::OGRSpatialReference &srsFrom
                , const std::string &srsIdTo
                , const registry::Registry &reg = registry::system);

    /** Combined convertor, never results in no-op convertor.
     */
    CsConvertor(const std::string &srsIdFrom
                , const ::OGRSpatialReference &srsTo
                , const registry::Registry &reg = registry::system);
#endif // GEO_HAS_GDAL

    /** Combined convertor, never results in no-op convertor.
     */
    CsConvertor(const registry::Srs &srsFrom
                , const std::string &srsIdTo
                , const registry::Registry &reg = registry::system);

#ifdef GEO_HAS_GDAL
    /** Returns inverse convertor (uses stored pointer to comples SRS
     *  definitions) -- swaps FROM and TO.
     *
     * \return inverse coordinate system convertor
     */
    CsConvertor inverse() const;
#endif // GEO_HAS_GDAL

    /** Converts point between FROM and TO srs.
     *
     *  Corectly unapplies FROM's vertical adjustment (if any), converts point
     *  into TO SRS and applies TO's vertical adjustment (if any).
     *
     *  If FROM and TO are same complex SRS then no coversion is made at all.
     *
     * \param p point in FROM-SRS
     * \return point in TO-SRS
     */
    math::Point3 operator()(const math::Point3 &p) const;

    /** Converts 2D point between FROM and TO srs.
     *
     *  Since Z-component is zero no adjustment is made.
     */
    math::Point2 operator()(const math::Point2 &p) const;

    /** Returns bounding box of all 8 corners converted to TO SRS.
     */
    math::Extents3 operator()(const math::Extents3 &e) const;

    /** Returns bounding box of all 4 corners converted to TO SRS.
     */
    math::Extents2 operator()(const math::Extents2 &e) const;

private:
    /** Helper constructor for conversion inversion.
     */
    CsConvertor(const boost::optional<geo::CsConvertor> &conv
                , const geo::VerticalAdjuster &srcAdjuster
                , const geo::VerticalAdjuster &dstAdjuster);

    /** Initialization helpers.
     */
    void init(const registry::Srs *srsFrom, const registry::Srs *srsTo);
    void init(const registry::Srs *srsFrom, const registry::Srs *srsTo,
        projCtx ctx);
    void init(const geo::SrsDefinition &srsFrom, const registry::Srs &srsTo);
    void init(const registry::Srs &srsFrom, const geo::SrsDefinition &srsTo);

#ifdef GEO_HAS_GDAL
    void init(const ::OGRSpatialReference &srsFrom
              , const registry::Srs &srsTo);
    void init(const registry::Srs &srsFrom
              , const ::OGRSpatialReference &srsTo);
#endif // GEO_HAS_GDAL

    /** Convertor between source and destination SRS's.
     */
    boost::optional<geo::CsConvertor> conv_;

    /** Vertical un/adjuster in source SRS.
     */
    geo::VerticalAdjuster srcAdjuster_;

    /** Vertical un/adjuster in destination SRS.
     */
    geo::VerticalAdjuster dstAdjuster_;
};

} } // namespace vtslibs::vts

#endif // geo_vtslibs_vts_csconvert_hpp_included_
