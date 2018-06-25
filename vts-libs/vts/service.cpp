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

#include "dbglog/dbglog.hpp"

#include "imgproc/png.hpp"

#include "../storage/error.hpp"
#include "../storage/sstreams.hpp"

#include "./service.hpp"
#include "./atmospheredensitytexture.hpp"

namespace vtslibs { namespace vts { namespace service {

namespace {

namespace constants {
    const std::string AtmosDensity("atmosdensity.png");
} // namespace constants

namespace ServiceFile {
    const unsigned int unknown = 0;
    const unsigned int atmosphereDensity = 1;
}

storage::IStream::pointer
atmosphereDensity(const std::string &filename, const std::string &query)
{
    (void) query;

    AtmosphereTextureSpec spec;
    const auto raw(generateAtmosphereTexture(spec));

    const auto format([&]() -> imgproc::png::RawFormat
    {
        switch (raw.components) {
        case 1: return imgproc::png::RawFormat::gray;
        case 3: return imgproc::png::RawFormat::rgb;
        case 4: return imgproc::png::RawFormat::rgba;
        }

        LOGTHROW(err1, storage::Error)
            << "Cannot serilize image with " << raw.components
            << " components to PNG.";
        throw;
    }());

    auto png(imgproc::png::serialize
             (raw.data.data(), raw.data.size()
              , raw.size, format, 9));

    return storage::memIStream
        ("image/png", std::move(png), std::time(nullptr), filename);
}

} // namespace

unsigned int match(const std::string &filename)
{
    if (filename == constants::AtmosDensity) {
        return ServiceFile::atmosphereDensity;
    }
    return ServiceFile::unknown;
}

storage::IStream::pointer generate(unsigned int type
                                   , const std::string &filename
                                   , const std::string &query)
{
    switch (type) {
    case ServiceFile::atmosphereDensity:
        return atmosphereDensity(filename, query);
    }

    LOGTHROW(err1, storage::Unimplemented)
        << "Unknown service file type <" << type << "> for file "
        << filename << ".";
    throw;
}

} } } // namespace vtslibs::vts::service
