#include <atomic>
#include <sstream>
#include <iomanip>

#include <boost/algorithm/string/split.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/io/ios_state.hpp>

#include "dbglog/dbglog.hpp"
#include "utility/streams.hpp"

#include "./tileop.hpp"

#include "./urltemplate.hpp"

namespace vtslibs { namespace vts {

namespace {

namespace ba = boost::algorithm;

inline unsigned int variableValue(UrlTemplate::Variable what
                                  , const UrlTemplate::Vars &vars)
{
    typedef UrlTemplate::Variable Variable;

    // expand variable
    switch (what) {
    case Variable::lod: return vars.tileId.lod; break;
    case Variable::x: return vars.tileId.x; break;
    case Variable::y: return vars.tileId.y; break;
    case Variable::loclod: return vars.localId.lod; break;
    case Variable::locx: return vars.localId.x; break;
    case Variable::locy: return vars.localId.y; break;
    case Variable::sub: return vars.subMesh; break;
    }
    return 0;
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

    class Counter {
    public:
        Counter() : counter_(0) {}
        Counter(const Counter&) : counter_(0) {}

        unsigned int next() const { return ++counter_; }

    private:
        mutable std::atomic<unsigned int> counter_;
    };

    Counter counter;
    return [alternatives, counter](std::ostream &os, const UrlTemplate::Vars&)
    {
        os << alternatives[counter.next() % alternatives.size()];
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
            return [lodVar, posVar, xAxis](std::ostream &os
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
        return [lodVar, posVar, xAxis](std::ostream &os
                                       , const UrlTemplate::Vars &vars)
        {
            vts::Lod lod(variableValue(lodVar, vars));
            unsigned int y(variableValue(posVar, vars));

            boost::io::ios_base_all_saver ps(os);
            os << std::setw(7) << std::hex << std::setfill('0')
               << ((1 << 28) - ((y + 1) << (28 - lod)));
        };

    } catch (boost::bad_lexical_cast) {}

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

    } catch (boost::bad_lexical_cast) {}

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

    } catch (boost::bad_lexical_cast) {}

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

    return {};
}

} // namespace

UrlTemplate::UrlTemplate(const std::string &str)
{
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
        try {
            // try as a variable
            tokens_.emplace_back(boost::lexical_cast<Variable>
                                 (str.substr(open + 1, close - open - 1)));
        } catch (boost::bad_lexical_cast) {
            // not a variable; try a function
            if (auto func = parseFunction(str, open + 1, close)) {
                tokens_.emplace_back(str.substr(open + 1, close - open - 1)
                                     , func);
            } else {
                tokens_.emplace_back(str.substr(open, close - open + 1));
                LOG(warn2)
                    << "Unprocessed directive <" << tokens_.back().value
                    << ">.";
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
        os << variableValue(*variable, vars);
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
