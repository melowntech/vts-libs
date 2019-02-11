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

#ifndef vtslibs_vts_urltemplate_po_hpp_included_
#define vtslibs_vts_urltemplate_po_hpp_included_

#include <boost/program_options.hpp>


#include "../registry.hpp"
#include "urltemplate.hpp"
#include "nodeinfo.hpp"
#include "tileop.hpp"

namespace vtslibs { namespace vts {

inline void
configuration(UrlTemplate::Vars &v
              , boost::program_options::options_description &od
              , const std::string &prefix = "")
{
    od.add_options()
        ((prefix + "tileId").c_str()
         , boost::program_options::value(&v.tileId)->default_value(TileId())
         , "Tile ID.")
        ((prefix + "localId").c_str()
         , boost::program_options::value(&v.localId)
         , "Local tile ID. If no value is provided: defaults to tileId "
         "unless reference frame is provided to compute local tile ID.")
        ((prefix + "srs").c_str()
         , boost::program_options::value(&v.srs)
         , "Spatial division SRS of tile. Computed from tileId if "
         "reference frame is available.")
        ((prefix + "sub").c_str()
         , boost::program_options::value(&v.subMesh)->default_value(0)
         , "Submesh index.")
        ((prefix + "param").c_str()
         , boost::program_options::value(&v.params)
         , "Generic string parameters, fed to {param(index)}.")
        ((prefix + "referenceFrame").c_str()
         , boost::program_options::value<std::string>()
         , "Reference frame to compute missing information.")
        ;
}

inline void
configure(UrlTemplate::Vars &v
          , const boost::program_options::variables_map &vars
          , const std::string &prefix = "")
{
    if (vars.count((prefix + "referenceFrame"))) {
        const auto referenceFrame(registry::system.referenceFrames
                                  (vars[prefix + "referenceFrame"]
                                   .as<std::string>()));

        NodeInfo ni(referenceFrame, v.tileId);

        if (!vars.count(prefix + "localId")) { v.localId = local(ni); }
        if (!vars.count(prefix + "srs")) { v.srs = ni.srs(); }

    } else {
        if (!vars.count(prefix + "localId")) { v.localId = v.tileId; }
    }
}

} } // namespace vtslibs::vts

#endif // vtslibs_vts_urltemplate_po_hpp_included_
