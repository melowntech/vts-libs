#include <boost/algorithm/string/split.hpp>

#include "dbglog/dbglog.hpp"

#include "./po.hpp"

namespace ba = boost::algorithm;
namespace po = boost::program_options;

namespace vtslibs { namespace registry {

CreditIds creditsConfigure(const po::variables_map &vars)
{
    CreditIds creditIds;

    if (!vars.count("credits")) { return creditIds; }

    std::vector<std::string> parts;
    for (const auto &value
             : ba::split(parts, vars["credits"].as<std::string>()
                         , ba::is_any_of(",")))
    {
        int numericCredit(-1);

        try {
            numericCredit = boost::lexical_cast<int>(value);
        } catch (const boost::bad_lexical_cast&) {
            creditIds.insert(system.credits(value).numericId);
        }

        if (const auto *credit = system.credits(numericCredit, std::nothrow)) {
            creditIds.insert(credit->numericId);
            continue;
        }

        LOG(warn2) << "Numeric credit id <"
                   << numericCredit << "> not found registry, using anyway.";
        creditIds.insert(numericCredit);
    }

    return creditIds;
}

} } // namespace vtslibs::registry
