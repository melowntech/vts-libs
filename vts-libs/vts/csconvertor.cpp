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
#include "csconvertor.hpp"

namespace vtslibs { namespace vts {

CsConvertor::CsConvertor(const std::string &srsIdFrom
                         , const std::string &srsIdTo
                         , const registry::Registry &reg)
{
    init(&reg.srs(srsIdFrom), &reg.srs(srsIdTo));
}

CsConvertor::CsConvertor(const std::string &srsIdFrom,
    const std::string &srsIdTo,
    const registry::Registry &reg, projCtx ctx)
{
    init(&reg.srs(srsIdFrom), &reg.srs(srsIdTo), ctx);
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

#ifdef GEO_HAS_GDAL
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
#endif // GEO_HAS_GDAL

CsConvertor::CsConvertor(const boost::optional<geo::CsConvertor> &conv
                         , const geo::VerticalAdjuster &srcAdjuster
                         , const geo::VerticalAdjuster &dstAdjuster)
    : conv_(conv), srcAdjuster_(srcAdjuster), dstAdjuster_(dstAdjuster)
{}

CsConvertor::CsConvertor(const registry::Srs &srsFrom
                         , const std::string &srsIdTo
                         , const registry::Registry &reg)
{
    init(&srsFrom, &reg.srs(srsIdTo));
}

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

void CsConvertor::init(const registry::Srs *srsFrom,
    const registry::Srs *srsTo, projCtx ctx)
{
    LOG(debug) << "CsConvert(\"" << srsFrom->srsDef << '"'
        << (srsFrom->adjustVertical() ? " [va]" : "")
        << ", \"" << srsTo->srsDef << '"'
        << (srsTo->adjustVertical() ? " [va]" : "")
        << ").";
    if (srsFrom != srsTo) {
        conv_ = boost::in_place(srsFrom->srsDef, srsTo->srsDef, ctx);

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

#ifdef GEO_HAS_GDAL
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
#endif // GEO_HAS_GDAL

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
