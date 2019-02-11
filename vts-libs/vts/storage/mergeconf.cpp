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
#include <boost/algorithm/string/predicate.hpp>

#include "dbglog/dbglog.hpp"

#include "../../storage/error.hpp"

#include "mergeconf.hpp"

namespace po = boost::program_options;
namespace ba = boost::algorithm;

namespace vtslibs { namespace vts {

namespace {

MergeConf loadMergeConf(std::istream &f, const boost::filesystem::path &path)
{
    MergeConf mc;

    try {
        // add options here
        po::options_description od;

        // first parse round: collect names
        auto parsed(po::parse_config_file(f, od, true));

        // store parses values in variables map
        po::variables_map vm;
        po::store(parsed, vm);

        for (const auto &opt : parsed.options) {
            if (!opt.unregistered) { continue; }
            for (auto iot(opt.original_tokens.begin())
                     , eot(opt.original_tokens.end());
                 iot != eot; ++iot)
            {
                auto key(*iot++);
                if (ba::starts_with(key, "cname.")) {
                    mc.cnames[key.substr(6)] = *iot;
                } else {
                    LOGTHROW(err2, vtslibs::storage::BadFileFormat)
                        << "Unknown option <" << key
                        << "> in merge configuration file " << path << ".";
                }
            }
        }

        po::notify(vm);
    } catch (const po::error &e) {
        LOGTHROW(err2, vtslibs::storage::BadFileFormat)
            << "Cannot parse in merge configuration file " << path
            << ": <" << e.what() << ">.";
    } catch (const std::ios_base::failure &e) {
        LOGTHROW(err2, vtslibs::storage::BadFileFormat)
            << "Cannot parse in merge configuration file " << path
            << ": <" << e.what() << ">.";
    }

    return mc;
}

} // namespace

MergeConf loadMergeConf(const boost::filesystem::path &path
                        , bool ignoreNoexistent)
{
    LOG(info1) << "Loading merge configuration from " << path  << ".";
    std::ifstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    try {
        f.open(path.string(), std::ios_base::in);
        f.peek();
    } catch (const std::exception &e) {
        if (ignoreNoexistent) { return {}; }

        LOGTHROW(err1, vtslibs::storage::NoSuchStorage)
            << "Unable to load merge configuration from " << path << ".";
    }
    f.exceptions(std::ifstream::badbit);
    auto conf(loadMergeConf(f, path));

    f.close();
    return conf;
}

} } // namespace vtslibs::vts
