#include "./csconvertor.hpp"

namespace vtslibs { namespace vts {

CsConvertor::CsConvertor(const std::string &srsIdFrom
                         , const std::string &srsIdTo
                         , const registry::Registry &reg)
{
    init(&reg.srs(srsIdFrom), &reg.srs(srsIdTo));
}

CsConvertor::CsConvertor(const geo::SrsDefinition &srsFrom
                         , const std::string &srsIdTo
                         , const registry::Registry &reg)
{
    init(srsFrom, reg.srs(srsIdTo));
}

CsConvertor::CsConvertor(const std::string &srsIdFrom
                         , const geo::SrsDefinition &srsTo
                         , const registry::Registry &reg)
{
    init(reg.srs(srsIdFrom), srsTo);
}

CsConvertor::CsConvertor(const ::OGRSpatialReference &srsFrom
                         , const std::string &srsIdTo
                         , const registry::Registry &reg)
{
    init(srsFrom, reg.srs(srsIdTo));
}

CsConvertor::CsConvertor(const std::string &srsIdFrom
                         , const ::OGRSpatialReference &srsTo
                         , const registry::Registry &reg)
{
    init(reg.srs(srsIdFrom), srsTo);
}

CsConvertor::CsConvertor(const boost::optional<geo::CsConvertor> &conv
                         , const geo::VerticalAdjuster &srcAdjuster
                         , const geo::VerticalAdjuster &dstAdjuster)
    : conv_(conv), srcAdjuster_(srcAdjuster), dstAdjuster_(dstAdjuster)
{}

void CsConvertor::init(const registry::Srs *srsFrom
                       , const registry::Srs *srsTo)
{
    LOG(debug) << "CsConvert(\"" << srsFrom->srsDef << '"'
               << (srsFrom->adjustVertical() ? " [va]" : "")
               << ", \"" << srsTo->srsDef << '"'
               << (srsTo->adjustVertical() ? " [va]" : "")
               << ").";
    if (srsFrom != srsTo) {
        conv_ = boost::in_place(srsFrom->srsDef, srsTo->srsDef);

        if (srsFrom->adjustVertical()) {
            srcAdjuster_ = geo::VerticalAdjuster(srsFrom->srsDef);
        }

        if (srsTo->adjustVertical()) {
            dstAdjuster_ = geo::VerticalAdjuster(srsTo->srsDef);
        }
    }
}

void CsConvertor::init(const geo::SrsDefinition &srsFrom
                       , const registry::Srs &srsTo)
{
    LOG(debug) << "CsConvert(\"" << srsFrom.srs << "\", \""
               << srsTo.srsDef << '"'
               << (srsTo.adjustVertical() ? " [va]" : "")
               << ").";
    conv_ = boost::in_place(srsFrom, srsTo.srsDef);

    if (srsTo.adjustVertical()) {
        dstAdjuster_ = geo::VerticalAdjuster(srsTo.srsDef);
    }
}


void CsConvertor::init(const registry::Srs &srsFrom
                       , const geo::SrsDefinition &srsTo)
{
    LOG(debug) << "CsConvert(\"" << srsFrom.srsDef << '"'
               << (srsFrom.adjustVertical() ? " [va]" : "")
               << ", \""
               << srsTo.srs << "\").";
    conv_ = boost::in_place(srsFrom.srsDef, srsTo);

    if (srsFrom.adjustVertical()) {
        srcAdjuster_ = geo::VerticalAdjuster(srsFrom.srsDef);
    }
}

void CsConvertor::init(const ::OGRSpatialReference &srsFrom
                       , const registry::Srs &srsTo)
{
    conv_ = boost::in_place(srsFrom, srsTo.srsDef);

    if (srsTo.adjustVertical()) {
        dstAdjuster_ = geo::VerticalAdjuster(srsTo.srsDef);
    }
}

void CsConvertor::init(const registry::Srs &srsFrom
                       , const ::OGRSpatialReference &srsTo)
{
    conv_ = boost::in_place(srsFrom.srsDef, srsTo);

    if (srsFrom.adjustVertical()) {
        srcAdjuster_ = geo::VerticalAdjuster(srsFrom.srsDef);
    }
}

CsConvertor CsConvertor::inverse() const
{
    if (conv_) {
        // we have convertor, invert all
        return { conv_->inverse(), dstAdjuster_, srcAdjuster_ };
    }

    // no convertor -> no-op
    return { boost::none, {}, {} };
}

math::Point3 CsConvertor::operator()(const math::Point3 &p) const
{
    // no conversion needed if no convertor present
    if (!conv_) { return p; }

    // un-vert-adjusts -> converts -> vert-adjusts
    return dstAdjuster_((*conv_)(srcAdjuster_(p, true)));
}

math::Point2 CsConvertor::operator()(const math::Point2 &p) const
{
    // no conversion needed if no convertor present
    if (!conv_) { return p; }

    // TODO: should we check that both SRS are not cartesian?

    // since z-component is zero -> nothing to adjust
    return (*conv_)(p);
}

math::Extents3 CsConvertor::operator()(const math::Extents3 &e) const
{
    math::Extents3 out(math::InvalidExtents{});
    for (const auto &p : { bll(e), bul(e), bur(e), blr(e)
                           , tll(e), tul(e), tur(e), tlr(e) })
    {
        update(out, operator()(p));
    }
    return out;
}

math::Extents2 CsConvertor::operator()(const math::Extents2 &e) const
{
    math::Extents2 out(math::InvalidExtents{});
    for (const auto &p : { ll(e), ul(e), ur(e), lr(e) }) {
        update(out, operator()(p));
    }
    return out;
}

} } // namespace vtslibs::vts
