/**
 * Copyright (c) 2018 Melown Technologies SE
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

#ifndef vtslibs_vts_atmospheredensity_hpp_included_
#define vtslibs_vts_atmospheredensity_hpp_included_

#include <cstdint>
#include <vector>

#include "math/geometry_core.hpp"

namespace vtslibs { namespace vts {

struct AtmosphereTextureSpec {
    AtmosphereTextureSpec();

    int version;

    math::Size2 size;
    double thickness; // normalized to planet radius
    double verticalCoefficient;
    double normFactor;
    double integrationStep;

    /** Build and HTTP query argument from this spec.
     */
    std::string toQueryArg() const;

    /** Parse from an HTTP query argument.
     */
    static AtmosphereTextureSpec fromQueryArg(const std::string &arg);
};

struct AtmosphereTexture {
    enum class Format { rgba = 0, rgb = 1, gray4 = 2, gray3 = 3, float_ = 4 };

    Format format;
    math::Size2 size;
    std::uint32_t components;
    std::vector<unsigned char> data;

    AtmosphereTexture() : components() {}
};

AtmosphereTexture
generateAtmosphereTexture(const AtmosphereTextureSpec &spec
                          , AtmosphereTexture::Format format
                          = AtmosphereTexture::Format::rgba);

} } // namespace vtslibs::vts

#endif // vtslibs_vts_atmospheredensity_hpp_included_
