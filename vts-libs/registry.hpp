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
/**
 * \file registry.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vtslibs_registry_hpp_included_
#define vtslibs_registry_hpp_included_

#include "registry/referenceframe.hpp"
#include "registry/freelayer.hpp"
#include "registry/datafile.hpp"
#include "registry/service.hpp"

namespace vtslibs { namespace registry {

/** Base registry: credits and bound layers only
 */
struct RegistryBase {
    BoundLayer::dict boundLayers;
    Credit::dict credits;

    RegistryBase() {};
};

/** Full registry
 */
struct Registry : RegistryBase {
    Srs::dict srs;
    ReferenceFrame::dict referenceFrames;
    Body::dict bodies;

    Registry() = default;
    Registry(const Registry&) = default;
    Registry(const RegistryBase &base) : RegistryBase(base) {}
};

/** System-wide registry
 */
extern Registry system;

void init(const boost::filesystem::path &confRoot);
boost::filesystem::path confRoot();

/** Returns default path to registry.
 *  NB: implemented in file generated from config.cpp.in template
 */
boost::filesystem::path defaultPath();

const DataFile& dataFile(const std::string &path, DataFile::Key key);
const DataFile* dataFile(const std::string &path, DataFile::Key key
                         , std::nothrow_t);

void load(RegistryBase &rb, const boost::filesystem::path &path);

void load(RegistryBase &rb, std::istream &in
          , const boost::filesystem::path &path
          = "UNKNOWN");

void save(const boost::filesystem::path &path, const RegistryBase &rb);

void save(std::ostream &out, const RegistryBase &rb);

bool operator==(const RegistryBase &l, const RegistryBase &r);
bool operator!=(const RegistryBase &l, const RegistryBase &r);

// inlines

inline bool operator==(const RegistryBase &l, const RegistryBase &r)
{
    return ((l.boundLayers == r.boundLayers)
            && (l.credits == r.credits));
}

inline bool operator!=(const RegistryBase &l, const RegistryBase &r)
{
    return !(l == r);
}

} } // namespace vtslibs::registry

#endif // vtslibs_registry_hpp_included_
