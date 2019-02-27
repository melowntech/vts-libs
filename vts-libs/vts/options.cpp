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

#include <boost/program_options.hpp>

#include "options.hpp"

namespace po = boost::program_options;

namespace vtslibs { namespace vts {

void OpenOptions::configuration(po::options_description &od
                                , const std::string &prefix)
{
    od.add_options()
        ((prefix + "io.retries").c_str()
         , po::value(&ioRetries_)->default_value(ioRetries_)
         , "Max number of retries on I/O operation "
         "(-1 means infinity retries).")
        ((prefix + "io.retryDelay").c_str()
         , po::value(&ioRetryDelay_)->default_value(ioRetryDelay_)
         , "Delay between individual reties. Zero means immediate retry!")
        ((prefix + "io.wait").c_str()
         , po::value(&ioWait_)->default_value(ioWait_)
         , "Timeout for I/O operations [in ms] "
         "(-1 means infinity retries).")
        ((prefix + "cname").c_str()
         , po::value<std::vector<std::string>>()
         , "CName mimicking for hostnames in remote tileset URLs. "
         "Format: srcHostname:dstHosname.")
        ;
}

void OpenOptions::configure(const po::variables_map &vars
                            , const std::string &prefix)
{
    const auto name(prefix + "cname");
    if (vars.count(name)) {
        for (const auto &def : vars[name].as<std::vector<std::string>>()) {
            const auto colon(def.find(':'));
            if ((colon == std::string::npos)
                || (colon + 1) == def.size())
            {
                throw po::validation_error
                    (po::validation_error::invalid_option_value
                     , name);
            }

            cnames_[def.substr(0, colon)] = def.substr(colon + 1);
        }
    }
}

std::ostream& OpenOptions::dump(std::ostream &os, const std::string &prefix)
    const
{
    os << prefix << "io.retries = " << ioRetries_ << '\n'
       << prefix << "io.retryDelay = " << ioRetryDelay_ << '\n'
       << prefix << "io.wait = " << ioWait_ << '\n';

    for (const auto &item : cnames_) {
        os << prefix << "cname = " << item.first
           << " -> " << item.second << '\n';
    }

    if (resourceFetcher_) {
        os << prefix << "io.fetcher = " << resourceFetcher_.get() << '\n';
    }

    return os;
}

} } // namespace vtslibs::vts
