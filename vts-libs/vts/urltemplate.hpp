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

namespace vadstena { namespace vts {

/** Wraps VTS URL template.
 */
class UrlTemplate {
public:
    UrlTemplate() = default;

    // FIXME: make copyable/assignable when token copy problem is fixed
    UrlTemplate(const UrlTemplate&) = delete;
    UrlTemplate& operator=(const UrlTemplate&) = delete;

    UrlTemplate(const std::string &str) { parse(str); }

    void parse(const std::string &str);

    struct Vars {
        vts::TileId tileId;
        vts::TileId localId;
        unsigned int subMesh;

        Vars(const vts::TileId &tileId, unsigned int subMesh = 0)
            : tileId(tileId), localId(tileId), subMesh(subMesh)
        {}

        Vars(const vts::TileId &tileId, const vts::TileId &localId
             , unsigned int subMesh = 0)
            : tileId(tileId), localId(localId), subMesh(subMesh)
        {}

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
        lod, x, y, loclod, locx, locy, sub
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

} } // namespace vadstena::vts

#endif // vts_libs_vts_urltemplate_hpp_included_
