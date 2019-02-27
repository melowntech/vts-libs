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
#include <boost/algorithm/string/split.hpp>

#include "dbglog/dbglog.hpp"

#include "po.hpp"

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
