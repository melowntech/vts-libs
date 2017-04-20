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
#ifndef vtslibs_storage_resources_hpp_included_
#define vtslibs_storage_resources_hpp_included_

#include <cstddef>

namespace vtslibs { namespace storage {

struct Resources {
    std::size_t openFiles;
    std::size_t memory;

    Resources(std::size_t openFiles = 0, std::size_t memory = 0)
        : openFiles(openFiles), memory(memory)
    {}

    Resources& operator+=(const Resources &o);
    Resources& operator-=(const Resources &o);
    bool operator<(const Resources &o) const;
    bool operator>(const Resources &o) const;
};

// inlines

inline Resources& Resources::operator+=(const Resources &o)
{
    openFiles += o.openFiles;
    memory += o.memory;
    return *this;
}

inline Resources& Resources::operator-=(const Resources &o)
{
    openFiles -= o.openFiles;
    memory -= o.memory;
    return *this;
}

inline bool Resources::operator<(const Resources &o) const
{
    if (openFiles < o.openFiles) {
        return true;
    } else if (o.openFiles < openFiles) {
        return false;
    }
    return memory < o.memory;
}

inline bool Resources::operator>(const Resources &o) const
{
    if (openFiles > o.openFiles) {
        return true;
    } else if (o.openFiles > openFiles) {
        return false;
    }
    return memory > o.memory;
}

} } // namespace vtslibs::storage

#endif // vtslibs_storage_resources_hpp_included_

