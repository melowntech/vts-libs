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
#include "utility/buildsys.hpp"

#include "../registry.hpp"

namespace vtslibs { namespace registry {

namespace detail {
    boost::filesystem::path root;

    DataFile::dict dataFilesByFilename;
    DataFile::dict dataFilesByPath;
} // namespace detail

Registry system;

void init(const boost::filesystem::path &confRoot)
{
    detail::root = confRoot;

    system.srs = loadSrs(confRoot / "srs.json");
    system.referenceFrames
        = loadReferenceFrames(confRoot / "referenceframes.json");
    system.boundLayers = loadBoundLayers(confRoot / "boundlayers.json");
    system.credits = loadCredits(confRoot / "credits.json");
    system.bodies = loadBodies(confRoot / "bodies.json", std::nothrow);

    // grab geoid grid files from srs
    for (const auto &srs : system.srs) {
        if (!srs.second.geoidGrid) { continue; }
        const auto &path(srs.second.geoidGrid->definition);
        if (detail::dataFilesByPath.has(path)) { continue; }

        DataFile df(detail::root / path);
        detail::dataFilesByPath.set(path, df);
        detail::dataFilesByFilename.set
            (boost::filesystem::path(path).filename().string(), df);
    }
}

boost::filesystem::path confRoot()
{
    return detail::root;
}

const DataFile& dataFile(const std::string &path, DataFile::Key key)
{
    switch (key) {
    case DataFile::Key::filename:
        return detail::dataFilesByFilename.get(path);

    case DataFile::Key::path:
        return detail::dataFilesByPath.get(path);
    }
    throw; // never reached
}

const DataFile* dataFile(const std::string &path, DataFile::Key key
                         , std::nothrow_t)
{
    switch (key) {
    case DataFile::Key::filename:
        return detail::dataFilesByFilename.get(path, std::nothrow);

    case DataFile::Key::path:
        return detail::dataFilesByPath.get(path, std::nothrow);
    }
    throw; // never reached
}

} } // namespace vtslibs::registry
