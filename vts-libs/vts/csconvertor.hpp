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

    /** Returns inverse convertor (uses stored pointer to comples SRS
     *  definitions) -- swaps FROM and TO.
     *
     * \return inverse coordinate system convertor
     */
    CsConvertor inverse() const;

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
    void init(const geo::SrsDefinition &srsFrom, const registry::Srs &srsTo);
    void init(const registry::Srs &srsFrom, const geo::SrsDefinition &srsTo);
    void init(const ::OGRSpatialReference &srsFrom
              , const registry::Srs &srsTo);
    void init(const registry::Srs &srsFrom
              , const ::OGRSpatialReference &srsTo);

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
