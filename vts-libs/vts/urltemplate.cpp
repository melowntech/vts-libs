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
#include <atomic>
#include <limits>
#include <sstream>
#include <iomanip>

#include <boost/algorithm/string/split.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/io/ios_state.hpp>

#include "dbglog/dbglog.hpp"
#include "utility/streams.hpp"
#include "utility/uri.hpp"

#include "tileop.hpp"

#include "urltemplate.hpp"

namespace vtslibs { namespace vts {

namespace {

namespace ba = boost::algorithm;

inline unsigned int variableValue(UrlTemplate::Variable what
                                  , const UrlTemplate::Vars &vars)
{
    typedef UrlTemplate::Variable Variable;

    // expand variable
    switch (what) {
    case Variable::lod: return vars.tileId.lod;
    case Variable::x: return vars.tileId.x;
    case Variable::y: return vars.tileId.y;
    case Variable::loclod: return vars.localId.lod;
    case Variable::locx: return vars.localId.x;
    case Variable::locy: return vars.localId.y;
    case Variable::sub: return vars.subMesh;

    case Variable::rf:
    case Variable::srs:
        LOG(warn1) << "Using variable <" << what << "> in numeric context.";
        return std::numeric_limits<unsigned int>::max();
    }
    return 0;
}

struct Expand {
    UrlTemplate::Variable what;
    const UrlTemplate::Vars &vars;
};

inline std::ostream& operator<<(std::ostream &os, const Expand &e)
{
    typedef UrlTemplate::Variable Variable;

    // expand variable
    switch (e.what) {
    case Variable::lod: os << e.vars.tileId.lod; break;
    case Variable::x: os << e.vars.tileId.x; break;
    case Variable::y: os << e.vars.tileId.y; break;
    case Variable::loclod: os << e.vars.localId.lod; break;
    case Variable::locx: os << e.vars.localId.x; break;
    case Variable::locy: os << e.vars.localId.y; break;
    case Variable::sub: os << e.vars.subMesh; break;
    case Variable::srs: os << e.vars.srs; break;
    case Variable::rf: os << e.vars.rf; break;
    }
    return os;
}

inline std::string variableStringValue(UrlTemplate::Variable what
                                       , const UrlTemplate::Vars &vars)
{
    return boost::lexical_cast<std::string>(Expand{what, vars});
}

unsigned int hash(unsigned int x) {
    x = ((x >> 16) ^ x) * 0x45d9f3b;
    x = ((x >> 16) ^ x) * 0x45d9f3b;
    x = (x >> 16) ^ x;
    return x;
}

UrlTemplate::Expander makeAltExpander(const std::string &str
                                   , const std::string::size_type &start
                                   , const std::string::size_type &end)
{
    std::vector<std::string> alternatives;
    auto range(std::make_pair(str.data() + start, str.data() + end));
    ba::split(alternatives, range, ba::is_any_of(", ")
              , ba::token_compress_on);

    if (alternatives.empty()) { return {}; }

    return [alternatives](std::ostream &os,
            const UrlTemplate::Vars &vars)
    {
        unsigned int sum = vars.subMesh
                + vars.tileId.lod
                + vars.tileId.x
                + vars.tileId.y
                + vars.localId.lod
                + vars.localId.x
                + vars.localId.y;
        os << alternatives[hash(sum) % alternatives.size()];
    };
}

UrlTemplate::Expander makePpExpander(const std::string &str
                                  , const std::string::size_type &start
                                  , const std::string::size_type &end
                                  , bool xAxis)
{
    // split args
    std::vector<std::string> args;
    auto range(std::make_pair(str.data() + start, str.data() + end));
    ba::split(args, range, ba::is_any_of(", ")
              , ba::token_compress_on);

    if (args.size() != 2) { return {}; }

    try {
        auto lodVar(boost::lexical_cast<UrlTemplate::Variable>(args[0]));
        auto posVar(boost::lexical_cast<UrlTemplate::Variable>(args[1]));

        if (xAxis) {
            // ppx
            return [lodVar, posVar](std::ostream &os
                                           , const UrlTemplate::Vars &vars)
            {
                vts::Lod lod(variableValue(lodVar, vars));
                unsigned int x(variableValue(posVar, vars));

                boost::io::ios_base_all_saver ps(os);
                os << std::setw(7) << std::hex << std::setfill('0')
                   << (x << (28 - lod));
            };
        }

        // ppy
        return [lodVar, posVar](std::ostream &os
                                       , const UrlTemplate::Vars &vars)
        {
            vts::Lod lod(variableValue(lodVar, vars));
            unsigned int y(variableValue(posVar, vars));

            boost::io::ios_base_all_saver ps(os);
            os << std::setw(7) << std::hex << std::setfill('0')
               << ((1 << 28) - ((y + 1) << (28 - lod)));
        };

    } catch (const boost::bad_lexical_cast&) {}

    return {};
}

UrlTemplate::Expander makeMsDigitExpander(const std::string &str
                                       , const std::string::size_type &start
                                       , const std::string::size_type &end)
{
    // split args
    std::vector<std::string> args;
    auto range(std::make_pair(str.data() + start, str.data() + end));
    ba::split(args, range, ba::is_any_of(", ")
              , ba::token_compress_on);

    if (args.size() != 2) { return {}; }

    try {
        auto xVar(boost::lexical_cast<UrlTemplate::Variable>(args[0]));
        auto yVar(boost::lexical_cast<UrlTemplate::Variable>(args[1]));

        return [xVar, yVar](std::ostream &os, const UrlTemplate::Vars &vars)
        {
            unsigned int x(variableValue(xVar, vars));
            unsigned int y(variableValue(yVar, vars));
            os << (((y & 3) << 1) + (x & 1));
        };

    } catch (const boost::bad_lexical_cast&) {}

    return {};
}

UrlTemplate::Expander makeQuadExpander(const std::string &str
                                    , const std::string::size_type &start
                                    , const std::string::size_type &end)
{
    // split args
    std::vector<std::string> args;
    auto range(std::make_pair(str.data() + start, str.data() + end));
    ba::split(args, range, ba::is_any_of(", ")
              , ba::token_compress_on);

    if (args.size() != 3) { return {}; }

    try {
        auto lodVar(boost::lexical_cast<UrlTemplate::Variable>(args[0]));
        auto xVar(boost::lexical_cast<UrlTemplate::Variable>(args[1]));
        auto yVar(boost::lexical_cast<UrlTemplate::Variable>(args[2]));

        return [lodVar, xVar, yVar](std::ostream &os
                                    , const UrlTemplate::Vars &vars)
        {
            int lod(variableValue(lodVar, vars));
            unsigned int x(variableValue(xVar, vars));
            unsigned int y(variableValue(yVar, vars));

            for (int i(lod - 1); i >= 0; --i) {
                os << (((((y >> i) & 1) << 1) + ((x >> i) & 1)));
            }
        };

    } catch (const boost::bad_lexical_cast&) {}

    return {};
}

UrlTemplate::Expander makeSwitchExpander(const std::string &str
                                         , const std::string::size_type &start
                                         , const std::string::size_type &end)
{
    // split args
    std::vector<std::string> args;
    auto range(std::make_pair(str.data() + start, str.data() + end));
    ba::split(args, range, ba::is_any_of(", ")
              , ba::token_compress_on);

    if (args.size() < 1) { return {}; }

    typedef std::map<std::string, std::string> Mapping;
    Mapping mapping;
    std::string default_;

    for (std::size_t i(1), e(args.size()); i != e; ++i) {
        const auto &arg(args[i]);

        auto colon(arg.find(':'));
        if (colon == std::string::npos) {
            // as-is
            mapping.insert(Mapping::value_type(arg, arg));
            continue;
        }

        Mapping::value_type pair(arg.substr(0, colon), arg.substr(colon + 1));
        if (pair.first == "*") {
            // default value
            default_ = std::move(pair.second);
            continue;
        }

        // store mapping
        mapping.insert(std::move(pair));
    }

    try {
        auto var(boost::lexical_cast<UrlTemplate::Variable>(args[0]));

        return [var, mapping, default_](std::ostream &os
                                        , const UrlTemplate::Vars &vars)
        {
            const auto value(variableStringValue(var, vars));

            const auto fmapping(mapping.find(value));
            if (fmapping == mapping.end()) {
                // not found, use default
                os << ((default_ == "*") ? value : default_);
            } else {
                os << ((fmapping->second == "*") ? value : fmapping->second);
            }
        };

    } catch (const boost::bad_lexical_cast&) {}

    return {};
}

UrlTemplate::Expander makeParamExpander(const std::string &str
                                        , const std::string::size_type &start
                                        , const std::string::size_type &end)
{
    // split args
    std::vector<std::string> args;
    auto range(std::make_pair(str.data() + start, str.data() + end));
    ba::split(args, range, ba::is_any_of(", ")
              , ba::token_compress_on);

    if (args.size() != 1) { return {}; }

    try {
        auto index(boost::lexical_cast<unsigned int>(args[0]));

        return [index](std::ostream &os
                       , const UrlTemplate::Vars &vars)
        {
            if (index >= vars.params.size()) { return; }
            os << utility::urlEncode(vars.params[index]);
        };

    } catch (const boost::bad_lexical_cast&) {}

    return {};
}

UrlTemplate::Expander parseFunction(const std::string &str
                                 , const std::string::size_type &start
                                 , const std::string::size_type &end)
{
    auto open(str.find('(', start));
    if (open >= end) {
        return {};
    }

    auto close(str.find(')', open));
    if (close >= end) {
        return {};
    }

    auto name(str.substr(start, open - start));

    if (name == "alt") {
        return makeAltExpander(str, open + 1, close);
    }

    if (name == "ppx") {
        return makePpExpander(str, open + 1, close, true);
    }

    if (name == "ppy") {
        return makePpExpander(str, open + 1, close, false);
    }

    if (name == "ms_digit") {
        return makeMsDigitExpander(str, open + 1, close);
    }

    if (name == "quad") {
        return makeQuadExpander(str, open + 1, close);
    }

    if (name == "switch") {
        return makeSwitchExpander(str, open + 1, close);
    }

    if (name == "param") {
        return makeParamExpander(str, open + 1, close);
    }

    return {};
}

} // namespace

void UrlTemplate::parse(const std::string &str)
{
    tokens_.clear();

    std::string::size_type index(0);
    std::string::size_type end(str.size());
    while (index < end) {
        // find opening brace
        auto open(str.find('{', index));
        if (open != index) {
            // some verbatim text
            tokens_.emplace_back(str.substr(index, (open - index)));
        }

        // eof?
        if (open == std::string::npos) { break; }

        // find close brace
        auto close(str.find('}', open + 1));
        if (close == std::string::npos) {
            // unclosed, consume everything
            LOG(warn2)
                << "Invalid expansion string <" << str
                << ">: unmatched '{'.";
            tokens_.emplace_back(str.substr(open));
            break;
        }

        // we have something inside { braces }

        // try a variable
        Variable result = Variable();
        if (boost::conversion::try_lexical_convert(
            str.substr(open + 1, close - open - 1), result)) {
            tokens_.emplace_back(result);
        } else {
            // not a variable; try a function
            if (auto func = parseFunction(str, open + 1, close)) {
                tokens_.emplace_back(str.substr(open + 1, close - open - 1)
                                     , func);
            } else {
                // emplace the name of the substituent, excluding the braces
                tokens_.emplace_back(str.substr(open + 1, close - open - 1));

                // ignore viewspec, otherwise issue a warning
                if (tokens_.back().value != "viewspec") {
                    LOG(warn2)
                        << "Unprocessed directive <" << tokens_.back().value
                        << ">.";
                }
            }
        }

        // next round
        index = close + 1;
    }
}

void UrlTemplate::expand(std::ostream &os, const Vars &vars) const
{
    for (const auto &token : tokens_) {
        token.expand(os, vars);
    }
}

void UrlTemplate::Token::expand(std::ostream &os, const Vars &vars)
    const
{
    if (expander) {
        // expander callback
        expander(os, vars);
        return;
    }

    if (variable) {
        // expand variable
        os << Expand{*variable, vars};
        return;
    }

    // plain value
    os << value;
}

void UrlTemplate::dump(std::ostream &os) const
{
    for (const auto &token : tokens_) {
        token.dump(os);
    }
}

void UrlTemplate::Token::dump(std::ostream &os)
    const
{
    if (expander) {
        os << "[expander:" << value << "]";
        return;
    }

    if (variable) {
        os << "[var:" << *variable << "]";
        return;
    }

    // plain value
    os << "[text:" << value << "]";
}

} } // namespace vtslibs::vts
