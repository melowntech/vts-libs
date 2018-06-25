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

#include <cassert>
#include <algorithm> // min, max

#include "dbglog/dbglog.hpp"

#include "../storage/error.hpp"
#include "atmospheredensitytexture.hpp"

namespace vtslibs { namespace vts {

AtmosphereTextureSpec::AtmosphereTextureSpec()
    : version(0), size(512, 512), thickness(0), verticalCoefficient(0),
    normFactor(0.2), integrationStep(0.0003)
{}

namespace
{

void encodeFloat(double v, unsigned char *target)
{
    if (v < 0 || v >= 1)
        LOGTHROW(err2, std::invalid_argument)
            << "invalid value for float encoding <"
            << v << ">";
    double enc[4] = { v, 256.0 * v, 256.0*256.0 * v, 256.0*256.0*256.0 * v };
    for (int i = 0; i < 4; i++)
        enc[i] -= std::floor(enc[i]); // frac
    double tmp[4];
    for (int i = 0; i < 3; i++)
        tmp[i] = enc[i + 1] / 256.0; // shift
    tmp[3] = 0;
    for (int i = 0; i < 4; i++)
        enc[i] -= tmp[i]; // subtract
    for (int i = 0; i < 4; i++)
        target[i] = enc[i] * 256.0;
}

template <class T>
T clamp(T v, T a, T b)
{
    return std::min(std::max(v, a), b);
}

AtmosphereTexture v0(const AtmosphereTextureSpec &spec)
{
    if (math::empty(spec.size))
        LOGTHROW(err1, std::invalid_argument)
            << "invalid resolution <" << spec.size << ">";
    if (spec.thickness <= 0)
        LOGTHROW(err1, std::invalid_argument)
            << "invalid thickness <"
            << spec.thickness << ">";
    if (spec.verticalCoefficient <= 0)
        LOGTHROW(err1, std::invalid_argument)
            << "invalid vertical coefficient <"
            << spec.verticalCoefficient << ">";
    if (spec.normFactor <= 0)
        LOGTHROW(err1, std::invalid_argument)
            << "invalid normalization factor <"
            << spec.normFactor << ">";
    if (spec.integrationStep <= 0)
        LOGTHROW(err1, std::invalid_argument)
            << "invalid integration step <"
            << spec.integrationStep << ">";

    double atmRad = 1.0 + spec.thickness;
    double atmRad2 = atmRad * atmRad;
    double invThickness = 1.0 / spec.thickness;
    double invWidth = 1.0 / spec.size.width;
    double invHeight = 1.0 / spec.size.height;

    AtmosphereTexture texture;
    texture.components = 4;
    texture.size = spec.size;
    texture.data.resize(spec.size.width * spec.size.height * 4);
    unsigned char *valsArray = (unsigned char*)texture.data.data();

    const std::uint32_t width(spec.size.width);
    const std::uint32_t height(spec.size.height);

    for (std::uint32_t xx = 0; xx < width; xx++)
    {
        double cosfi = 2 * xx * invWidth - 1;
        double fi = std::acos(cosfi);
        double sinfi = std::sin(fi);

        for (std::uint32_t yy = 0; yy < height; yy++)
        {
            double yyy = yy * invHeight;
            double r = 2 * spec.thickness * yyy - spec.thickness + 1;
            double t0 = cosfi * r;
            double y = sinfi * r;
            double y2 = y * y;
            double a = sqrt(atmRad2 - y2);
            double density = 0;
            for (double t = t0; t < a; t += spec.integrationStep)
            {
                double h = std::sqrt(t * t + y2);
                // h is distance of the point from origin
                h = (clamp(h, 1.0, atmRad) - 1) * invThickness;
                // h is 0..1 now
                double a = std::exp(-spec.verticalCoefficient * h);
                density += a;
            }
            density *= spec.integrationStep;
            encodeFloat(density * spec.normFactor,
                valsArray + ((yy * width + xx) * 4));
        }
    }

    return texture;
}

} // namespace

AtmosphereTexture generateAtmosphereTexture(const AtmosphereTextureSpec &spec)
{
    switch (spec.version) {
    case 0: return v0(spec);
    default: break;
    }

    LOGTHROW(err2, storage::VersionError)
        << "Atmosphere density texture: unsupported version <"
        << spec.version  << ">.";
    throw;
}


} } // namespace vtslibs::vts
