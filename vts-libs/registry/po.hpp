#ifndef vtslibs_registry_po_hpp_included_
#define vtslibs_registry_po_hpp_included_

#include <string>
#include <vector>

#include <boost/filesystem/path.hpp>
#include <boost/program_options.hpp>

#include "utility/buildsys.hpp"

#include "../registry.hpp"

namespace vtslibs { namespace registry {

inline void registryConfiguration
(boost::program_options::options_description &od
 , const boost::filesystem::path &defaultPath)
{
    od.add_options()
        ("registry", boost::program_options::value<boost::filesystem::path>()
         ->required()->default_value(defaultPath)
         , "Path to registry directory, i.e. where reference "
         "frames etc are to be found.")
        ;
}

inline void
registryConfigure(const boost::program_options::variables_map &vars)
{
    init(vars["registry"].as<boost::filesystem::path>());
}

inline void
creditsConfiguration(boost::program_options::options_description &od)
{
    od.add_options()
        ("credits", boost::program_options::value<std::string>()
         , "Comma-separated list of string/numeric credit id.");
}

CreditIds creditsConfigure(const boost::program_options::variables_map &vars);

} } // namespace vtslibs::registry

#endif // vtslibs_registry_po_hpp_included_
