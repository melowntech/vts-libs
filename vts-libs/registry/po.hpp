#ifndef vadstena_libs_registry_po_hpp_included_
#define vadstena_libs_registry_po_hpp_included_

#include <string>
#include <vector>

#include <boost/filesystem/path.hpp>
#include <boost/program_options.hpp>

#include "utility/buildsys.hpp"

#include "../registry.hpp"

namespace vadstena { namespace registry {

void registryConfiguration
(boost::program_options::options_description &od
 , const std::string &defaultPath)
{
    od.add_options()
        ("registry", boost::program_options::value<boost::filesystem::path>()
         ->required()->default_value
         (utility::buildsys::installPath(defaultPath))
         , "Path to registry directory, i.e. where reference "
         "frames etc are to be found.")
        ;
}

void registryConfigure(const boost::program_options::variables_map &vars)
{
    Registry::init(vars["registry"].as<boost::filesystem::path>());
}

} } // namespace vadstena::registry

#endif // vadstena_libs_registry_po_hpp_included_
