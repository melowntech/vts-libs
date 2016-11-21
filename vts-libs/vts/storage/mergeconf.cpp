#include <boost/program_options.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include "dbglog/dbglog.hpp"

#include "../../storage/error.hpp"

#include "./mergeconf.hpp"

namespace po = boost::program_options;
namespace ba = boost::algorithm;

namespace vadstena { namespace vts {

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
                    LOGTHROW(err2, vadstena::storage::BadFileFormat)
                        << "Unknown option <" << key
                        << "> in merge configuration file " << path << ".";
                }
            }
        }

        po::notify(vm);
    } catch (const po::error &e) {
        LOGTHROW(err2, vadstena::storage::BadFileFormat)
            << "Cannot parse in merge configuration file " << path
            << ": <" << e.what() << ">.";
    } catch (const std::ios_base::failure &e) {
        LOGTHROW(err2, vadstena::storage::BadFileFormat)
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

        LOGTHROW(err1, vadstena::storage::NoSuchStorage)
            << "Unable to load merge configuration from " << path << ".";
    }
    f.exceptions(std::ifstream::badbit);
    auto conf(loadMergeConf(f, path));

    f.close();
    return conf;
}

} } // namespace vadstena::vts
