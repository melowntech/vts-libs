#ifndef vtslibs_registry_po_hpp_included_
#define vtslibs_registry_po_hpp_included_

#include <string>
#include <vector>

#include <boost/filesystem/path.hpp>
#include <boost/program_options.hpp>

#include "utility/buildsys.hpp"

#include "../registry.hpp"

namespace vtslibs { namespace registry {

void registryConfiguration
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

void registryConfigure(const boost::program_options::variables_map &vars)
{
    init(vars["registry"].as<boost::filesystem::path>());
}

} } // namespace vtslibs::registry

#endif // vtslibs_registry_po_hpp_included_
