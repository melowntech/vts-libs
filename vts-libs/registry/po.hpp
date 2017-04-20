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
