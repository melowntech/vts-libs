#include <cstring>

#include "./support.hpp"

namespace vadstena { namespace storage {

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

} } // namespace vadstena::storage
