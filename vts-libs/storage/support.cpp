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
#include <cstring>

#include "support.hpp"

namespace vtslibs { namespace storage {

namespace {

const char *find(const char *str, const char *end, const char *token)
{
    if (const char *f = std::strstr(str, token)) { return f; }
    return end;
}

bool replace(std::string &res, const std::string &name
             , const SupportFile::Vars *vars
             , const SupportFile::Vars *defaults)
{
    // try variables
    if (vars) {
        auto fvars(vars->find(name));
        if (fvars != vars->end()) {
            res.append(fvars->second);
            return true;
        }
    }

    // try defaults
    if (defaults) {
        auto fdefaults(defaults->find(name));
        if (fdefaults != defaults->end()) {
            res.append(fdefaults->second);
            return true;
        }
    }

    return false;
}

} // namespace

std::string SupportFile::expand(const Vars *vars, const Vars *defaults) const
{
    std::string res;

    const char* index(reinterpret_cast<const char*>(data));
    const char* end(index + size);

    while (index < end) {
        // find opening brace
        auto open(find(index, end, "{{{"));
        if (open != index) {
            // some verbatim text
            res.append(index, open);
        }

        // eof?
        if (open == end) { break; }

        // find close brace
        auto close(find(open + 1, end, "}}}"));
        if (close == end) {
            res.append(open, end);
            break;
        }

        // prepare index to next round
        index = close + 3;

        // we have something inside { braces } -> replace
        if (!replace(res, std::string(open + 3, close), vars, defaults)) {
            res.append(open, index);
        }
    }

    return res;
}

} } // namespace vtslibs::storage
