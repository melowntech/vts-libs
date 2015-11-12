#ifndef geo_vadstena_libs_vts_csconvert_hpp_included_
#define geo_vadstena_libs_vts_csconvert_hpp_included_

#include "geo/csconvertor.hpp"
#include "geo/verticaladjuster.hpp"

#include "../registry.hpp"

namespace vadstena { namespace vts {

/** Coordinate system convertor.
 */
class CsConvertor {
public:
    /** Creates convertor betwee two SRS specified as keys to global SRS
     *  registry.
     */
    CsConvertor(const std::string &srsIdFrom, const std::string &srsIdTo);

    /** Creates convertor betwee two SRS specified pointers to complex SRS
     *  definitons.
     */
    CsConvertor(const registry::Srs *srsFrom, const registry::Srs *srsTo);

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

private:
    /** Initialization helper.
     */
    void init();

    /** Source complex SRS.
     */
    const registry::Srs* srsFrom_;

    /** Destinatio complex SRS.
     */
    const registry::Srs* srsTo_;

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

} } // namespace vadstena::vts

#endif // geo_vadstena_libs_vts_csconvert_hpp_included_
