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
#include <sstream>

#include "dbglog/dbglog.hpp"

#include "utility/binaryio.hpp"
#include "utility/base64.hpp"

#include "../storage/error.hpp"
#include "atmospheredensitytexture.hpp"

namespace bin = utility::binaryio;

namespace vtslibs { namespace vts {

/** Do not change any defaults here.
 */
AtmosphereTextureSpec::AtmosphereTextureSpec()
    : version(0), size(512, 512), thickness(0), verticalCoefficient(0),
    normFactor(0.2), integrationStep(0.0003)
{}

namespace
{

template <AtmosphereTexture::Format format>
void encodeFloat(double v, unsigned char *target, int index, int planeSize)
{
    if (v < 0 || v >= 1) {
        LOGTHROW(err2, std::invalid_argument)
            << "invalid value for float encoding <"
            << v << ">";
    }

    if (format == AtmosphereTexture::Format::float_) {
        *reinterpret_cast<float*>(target + index * 4) = float(v);
        return;
    }

    double enc[4] = { v, 256.0 * v, 256.0*256.0 * v, 256.0*256.0*256.0 * v };
    for (int i = 0; i < 4; i++) {
        enc[i] -= std::floor(enc[i]); // frac
    }
    double tmp[4];
    for (int i = 0; i < 3; i++) {
        tmp[i] = enc[i + 1] / 256.0; // shift
    }
    tmp[3] = 0;
    for (int i = 0; i < 4; i++) {
        enc[i] -= tmp[i]; // subtract
    }

    switch (format) {
    case AtmosphereTexture::Format::rgba:
        target += index * 4;
        for (int i = 0; i < 4; i++) {
            target[i] = enc[i] * 256.0;
        }
        break;

    case AtmosphereTexture::Format::rgb:
        target += index * 3;
        for (int i = 0; i < 3; i++) {
            target[i] = enc[i] * 256.0;
        }
        break;

    case AtmosphereTexture::Format::gray3:
        for (int i = 0; i < 3; i++) {
            target[i * planeSize + index] = enc[i] * 256.0;
        }
        break;

    case AtmosphereTexture::Format::gray4:
        for (int i = 0; i < 4; i++) {
            target[i * planeSize + index] = enc[i] * 256.0;
        }
        break;

    default: break;
    }
}

template <class T>
T clamp(T v, T a, T b)
{
    return std::min(std::max(v, a), b);
}

template <AtmosphereTexture::Format format>
AtmosphereTexture v0(const AtmosphereTextureSpec &spec)
{
    double atmRad = 1.0 + spec.thickness;
    double atmRad2 = atmRad * atmRad;
    double invThickness = 1.0 / spec.thickness;
    double invWidth = 1.0 / spec.size.width;
    double invHeight = 1.0 / spec.size.height;

    AtmosphereTexture texture;
    texture.size = spec.size;
    auto byteSize(math::area(spec.size));
    auto planeSize(byteSize);
    switch (format) {
    case AtmosphereTexture::Format::rgba:
        texture.components = 4;
        byteSize *= 4;
        break;

    case AtmosphereTexture::Format::rgb:
        texture.components = 3;
        byteSize *= 3;
        break;

    case AtmosphereTexture::Format::gray4:
        texture.components = 1;
        texture.size.height *= 4;
        byteSize *= 4;
        break;

    case AtmosphereTexture::Format::gray3:
        texture.components = 1;
        texture.size.height *= 3;
        byteSize *= 3;
        break;

    case AtmosphereTexture::Format::float_:
        texture.components = 1;
        byteSize *= sizeof(float);
        break;
    }

    texture.data.resize(byteSize);
    auto *valsArray = (unsigned char*)texture.data.data();

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

            encodeFloat<format>(density * spec.normFactor
                                , valsArray, (yy * width + xx)
                                , planeSize);
        }
    }

    return texture;
}

AtmosphereTexture v0(const AtmosphereTextureSpec &spec
                     , AtmosphereTexture::Format format)
{
    if (math::empty(spec.size)) {
        LOGTHROW(err1, std::invalid_argument)
            << "invalid resolution <" << spec.size << ">";
    }
    if (spec.thickness <= 0) {
        LOGTHROW(err1, std::invalid_argument)
            << "invalid thickness <"
            << spec.thickness << ">";
    }
    if (spec.verticalCoefficient <= 0) {
        LOGTHROW(err1, std::invalid_argument)
            << "invalid vertical coefficient <"
            << spec.verticalCoefficient << ">";
    }
    if (spec.normFactor <= 0) {
        LOGTHROW(err1, std::invalid_argument)
            << "invalid normalization factor <"
            << spec.normFactor << ">";
    }
    if (spec.integrationStep <= 0) {
        LOGTHROW(err1, std::invalid_argument)
            << "invalid integration step <"
            << spec.integrationStep << ">";
    }

    switch (format) {
    case AtmosphereTexture::Format::rgba:
        return v0<AtmosphereTexture::Format::rgba>(spec);
    case AtmosphereTexture::Format::rgb:
        return v0<AtmosphereTexture::Format::rgb>(spec);
    case AtmosphereTexture::Format::gray3:
        return v0<AtmosphereTexture::Format::gray3>(spec);
    case AtmosphereTexture::Format::gray4:
        return v0<AtmosphereTexture::Format::gray4>(spec);
    case AtmosphereTexture::Format::float_:
        return v0<AtmosphereTexture::Format::float_>(spec);
    }
    throw;
}

} // namespace

AtmosphereTexture generateAtmosphereTexture(const AtmosphereTextureSpec &spec
                                            , AtmosphereTexture::Format format)
{
    switch (spec.version) {
    case 0: return v0(spec, format);
    default: break;
    }

    LOGTHROW(err2, storage::VersionError)
        << "Atmosphere density texture: unsupported version <"
        << spec.version  << ">.";
    throw;
}

std::string AtmosphereTextureSpec::toQueryArg() const
{
    std::ostringstream os(std::ios_base::out | std::ios_base::binary);

    bin::write<std::uint8_t>(os, 0);
    bin::write<std::uint16_t>(os, size.width);
    bin::write<std::uint16_t>(os, size.height);
    bin::write<float>(os, thickness);
    bin::write<float>(os, verticalCoefficient);
    bin::write<float>(os, normFactor);
    bin::write<float>(os, integrationStep);

    return utility::base64::encode(os.str());
}

namespace {

void parse0(AtmosphereTextureSpec &spec, std::istream &is, std::size_t size)
{
    if (size != (2 * sizeof(std::uint16_t) + 4 * sizeof(float))) {
        LOGTHROW(err1, std::invalid_argument)
            << "Unable to parse atmosphere parameters.";
    }

    spec.size.width = bin::read<std::uint16_t>(is);
    spec.size.height = bin::read<std::uint16_t>(is);
    spec.thickness = bin::read<float>(is);
    spec.verticalCoefficient = bin::read<float>(is);
    spec.normFactor = bin::read<float>(is);
    spec.integrationStep = bin::read<float>(is);
}

} // namespace

AtmosphereTextureSpec
AtmosphereTextureSpec::fromQueryArg(const std::string &arg)
{
    AtmosphereTextureSpec spec;

    auto decoded(utility::base64::decode(arg));

    try {
        std::istringstream is
            (decoded, std::ios_base::in | std::ios_base::binary);
        is.exceptions(std::ios::failbit);
        spec.version = bin::read<std::uint8_t>(is);

        switch (spec.version) {
        case 0: parse0(spec, is, decoded.size() - sizeof(std::uint8_t));
            break;

        default:
            LOGTHROW(err2, storage::VersionError)
                << "Atmosphere density texture: unsupported version <"
                << spec.version  << ">.";
        }
    } catch (...) {
        LOGTHROW(err1, std::invalid_argument)
            << "Unable to parse atmosphere parameters.";
    }

    return spec;
}


} } // namespace vtslibs::vts
