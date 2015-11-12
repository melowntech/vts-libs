#include "./csconvertor.hpp"

namespace vadstena { namespace vts {

CsConvertor::CsConvertor(const std::string &srsIdFrom
                         , const std::string &srsTo)
    : srsFrom_(&registry::Registry::srs(srsIdFrom))
    , srsTo_(&registry::Registry::srs(srsTo))
{
    init();
}

CsConvertor::CsConvertor(const registry::Srs *srsFrom
                         , const registry::Srs *srsTo)
    : srsFrom_(srsFrom), srsTo_(srsTo)
{
    init();
}

void CsConvertor::init()
{
    if (srsFrom_ != srsTo_) {
        conv_ = boost::in_place(srsFrom_->srsDef, srsTo_->srsDef);
    }

    if (srsFrom_->adjustVertical()) {
        srcAdjuster_ = geo::VerticalAdjuster(srsFrom_->srsDef);
    }

    if (srsTo_->adjustVertical()) {
        dstAdjuster_ = geo::VerticalAdjuster(srsTo_->srsDef);
    }

}

CsConvertor CsConvertor::inverse() const
{
    return { srsTo_, srsFrom_ };
}

math::Point3 CsConvertor::operator()(const math::Point3 &p) const
{
    // no conversion needed if no convertor present
    if (!conv_) { return p; }

    // un-vert-adjusts -> converts -> vert-adjusts
    return dstAdjuster_((*conv_)(srcAdjuster_(p, true)));
}

} } // namespace vadstena::vts
