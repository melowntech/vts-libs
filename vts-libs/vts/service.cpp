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

#include "utility/uri.hpp"
#include "utility/httpquery.hpp"

#include "imgproc/png.hpp"

#include "../storage/error.hpp"
#include "../storage/sstreams.hpp"

#include "service.hpp"
#include "atmospheredensitytexture.hpp"

namespace uq = utility::query;

namespace vtslibs { namespace vts { namespace service {

namespace {

namespace constants {
    const std::string AtmDensityName("atmdensity");
    const std::string AtmDensityFilename("atmdensity.png");
} // namespace constants

namespace ServiceFile {
    const unsigned int unknown = 0;
    const unsigned int atmphereDensity = 1;
}

storage::IStream::pointer
atmphereDensity(const std::string &filename, const std::string &query
                  , AtmosphereTexture::Format format)
{
    const auto args(uq::splitQuery(query));
    const auto def(utility::urlDecode(uq::asString(uq::find(args, "def"))));

    if (def.empty()) {
        LOGTHROW(err1, std::invalid_argument)
            << "Cannot find required query argument def=?.";
        throw;
    }

    const auto spec(AtmosphereTextureSpec::fromQueryArg(def));

    const auto texture(generateAtmosphereTexture(spec, format));

    switch (format) {
    case AtmosphereTexture::Format::float_: {
        std::vector<char> tmp(texture.data.begin(), texture.data.end());
        return storage::memIStream
            ("application/octet-stream", std::move(tmp)
             , std::time(nullptr), filename);

    }

    case AtmosphereTexture::Format::gray3:
    case AtmosphereTexture::Format::gray4:
    case AtmosphereTexture::Format::rgb:
    case AtmosphereTexture::Format::rgba: {
        const auto pngFormat([&]() -> imgproc::png::RawFormat
        {
            switch (texture.components) {
            case 1: return imgproc::png::RawFormat::gray;
            case 3: return imgproc::png::RawFormat::rgb;
            case 4: return imgproc::png::RawFormat::rgba;
            }

            LOGTHROW(err1, storage::Error)
                << "Cannot serialize image with " << texture.components
                << " components to PNG.";
            throw;
        }());

        auto png(imgproc::png::serialize
                 (texture.data.data(), texture.data.size()
                  , texture.size, pngFormat, 9));

        return storage::memIStream
            ("image/png", std::move(png), std::time(nullptr), filename);
    }
    }
    throw;
}

} // namespace

unsigned int match(const std::string &filename)
{
    if (filename == constants::AtmDensityFilename) {
        return ServiceFile::atmphereDensity;
    }
    return ServiceFile::unknown;
}

storage::IStream::pointer generate(unsigned int type
                                   , const std::string &filename
                                   , const std::string &query)
{
    switch (type) {
    case ServiceFile::atmphereDensity:
        return atmphereDensity
            (filename, query, AtmosphereTexture::Format::gray3);
    }

    LOGTHROW(err1, std::invalid_argument)
        << "Unknown service file type <" << type << "> for file "
        << filename << ".";
    throw;
}

void addLocal(MapConfig &mapConfig, const boost::filesystem::path &root)
{
    mapConfig.services.add
        (registry::Service
         (constants::AtmDensityName
          , (root / (constants::AtmDensityFilename + "?def={param(0)}"))
          .string()));
}

} } } // namespace vtslibs::vts::service
