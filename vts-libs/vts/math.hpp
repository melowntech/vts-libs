#ifndef vadstena_libs_vts_math_hpp
#define vadstena_libs_vts_math_hpp

#include "math/geometry.hpp"

namespace vadstena { namespace vts {

double triangleArea(const math::Point3 &a, const math::Point3 &b,
                    const math::Point3 &c);

double triangleArea(const math::Point2 &a, const math::Point2 &b,
                    const math::Point2 &c);

class TextureNormalizer {
public:
    TextureNormalizer(const math::Extents2 &divisionExtents)
        : size_(size(divisionExtents))
        , origin_(divisionExtents.ll)
    {}

    math::Point2 operator()(const math::Point3 &p) const {
        // NB: origin is in the upper-left corner
        return { (p(0) - origin_(0)) / size_.width
                , (p(1) - origin_(1)) / size_.height };
    };

private:
    math::Size2f size_;
    math::Point2 origin_;
};

// inlines

inline double triangleArea(const math::Point3 &a, const math::Point3 &b,
                           const math::Point3 &c)
{
    return norm_2(math::crossProduct(b - a, c - a)) / 2.0;
}

inline double triangleArea(const math::Point2 &a, const math::Point2 &b,
                           const math::Point2 &c)
{
    return std::abs
        (math::crossProduct(math::Point2(b - a), math::Point2(c - a)))
        / 2.0;
}

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_math_hpp
