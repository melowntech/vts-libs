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
#ifndef vtslibs_vts_math_hpp
#define vtslibs_vts_math_hpp

#include "math/geometry.hpp"

namespace vtslibs { namespace vts {

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

} } // namespace vtslibs::vts

#endif // vtslibs_vts_math_hpp
