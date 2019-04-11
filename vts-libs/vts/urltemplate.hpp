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
#ifndef vts_libs_vts_urltemplate_hpp_included_
#define vts_libs_vts_urltemplate_hpp_included_

#include <string>
#include <functional>
#include <iostream>
#include <sstream>

#include <boost/optional.hpp>

#include "utility/enum-io.hpp"

#include "vts-libs/vts/nodeinfo.hpp"
#include "vts-libs/vts/mapconfig.hpp"

namespace vtslibs { namespace vts {

/** Wraps VTS URL template.
 */
class UrlTemplate {
public:
    UrlTemplate() = default;
    UrlTemplate(const UrlTemplate&) = default;
    UrlTemplate& operator=(const UrlTemplate&) = default;

    UrlTemplate(const std::string &str) { parse(str); }

    void parse(const std::string &str);
    void clear() { tokens_.clear(); }
    bool empty() const { return tokens_.empty(); }

    struct Vars {
        vts::TileId tileId;
        vts::TileId localId;
        std::string srs;
        unsigned int subMesh;
        std::string rf;

        std::vector<std::string> params;

        Vars(const vts::TileId &tileId, unsigned int subMesh = 0)
            : tileId(tileId), localId(tileId), subMesh(subMesh)
        {}

        Vars(const vts::TileId &tileId, const vts::TileId &localId
             , unsigned int subMesh = 0)
            : tileId(tileId), localId(localId), subMesh(subMesh)
        {}

        Vars(const vts::TileId &tileId, const vts::TileId &localId
             , const std::string &srs, unsigned int subMesh = 0)
            : tileId(tileId), localId(localId), srs(srs), subMesh(subMesh)
        {}

        Vars() : subMesh() {}

        Vars addSubmesh(int subMesh) {
            Vars out(*this);
            out.subMesh = subMesh;
            return out;
        }
    };

    std::string operator()(const Vars &vars) const;

    void expand(std::ostream &os, const Vars &vars) const;

    void dump(std::ostream &os) const;

    enum Variable {
        lod, x, y, loclod, locx, locy, sub, srs, rf
    };

    typedef std::function<void(std::ostream &os
                               , const Vars &vars)> Expander;

    struct Token {

        std::string value;
        boost::optional<Variable> variable;
        Expander expander;

        void expand(std::ostream &os, const Vars &vars) const;
        void dump(std::ostream &os) const;

        Token(const std::string &value) : value(value) {}
        Token(const Variable &variable) : variable(variable) {}
        Token(const std::string &value, const Expander &expander)
            : value(value), expander(expander)
        {}

        typedef std::vector<Token> list;
    };

private:
    Token::list tokens_;
};

// inlines

UTILITY_GENERATE_ENUM_IO(UrlTemplate::Variable,
                         ((lod))
                         ((x))
                         ((y))
                         ((loclod))
                         ((locx))
                         ((locy))
                         ((sub))
                         ((srs))
                         ((rf))
)

inline std::ostream& operator<<(std::ostream &os, const UrlTemplate &t)
{
    t.dump(os);
    return os;
}

inline std::string UrlTemplate::operator()(const Vars &vars) const
{
    std::ostringstream os;
    expand(os, vars);
    return os.str();
}

} } // namespace vtslibs::vts

#endif // vts_libs_vts_urltemplate_hpp_included_
