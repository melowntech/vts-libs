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
#ifndef vtslibs_vts_options_po_hpp_included_
#define vtslibs_vts_options_po_hpp_included_

#include <string>
#include <vector>
#include <boost/program_options.hpp>

#include "utility/streams.hpp"

#include "options.hpp"
#include "options.hpp"

namespace vtslibs { namespace vts {

inline void configuration(boost::program_options::options_description &od
                          , MergeOptions &mo)
{
    namespace po = boost::program_options;

    const auto cs(Mesh::coverageSize());

    od.add_options()
        ("glue.mode", po::value(&mo.glueMode)
         ->default_value(mo.glueMode)
         , utility::concat
         ("Glue tile generation mode, one of "
          , enumerationString(mo.glueMode), ".").c_str())

        ("glue.skirt.mode", po::value(&mo.skirtMode)
         ->default_value(mo.skirtMode)
         , utility::concat
         ("Skirt mode, one of "
          , enumerationString(mo.skirtMode), ".").c_str())
        ("glue.skirt.scale", po::value(&mo.skirtScale)
         ->default_value(mo.skirtScale)
         , "Scaling factor for computer skirt length")

        ("glue.safetyMargin", po::value(&mo.safetyMargin)
         ->default_value(mo.safetyMargin)
         , utility::concat
         ("Safety margin added around tile when generating new tile. "
          "Value is in pixels, where 1 pixel is [1/", cs.width
          , ", 1/", cs.height, "] of tile SDS extents size. "
          "Should not be touched in production environment."
          ).c_str())

        ("glue.mode.coverageContour.simplification"
         , po::value(&mo.contourSimplification)
         ->default_value(mo.contourSimplification)
         , utility::concat
         ("Contour simplification algorithm for glue.mode=coverageContour, "
          " one of "
          , enumerationString(mo.contourSimplification)
          , ". Should not be touched in production environment.").c_str())

        ("glue.mode.coverageContour.simplification.rdp.maxError"
         , po::value(&mo.rdpMaxError)
         ->default_value(mo.rdpMaxError)
         , "Maximum simplified segment error setting for "
         "glue.mode.coverageContour=rdp. In pixels. "
         "Should not be touched in production environment.")
        ;
}

inline void configure(const boost::program_options::variables_map &vars
                      , MergeOptions &mo)
{
    namespace po = boost::program_options;

    if (mo.safetyMargin < 0) {
        throw po::invalid_option_value("glue.safetyMargin");
    }

    (void) vars;
}

} } // namespace vtslibs::vts

#endif // vtslibs_vts_options_po_hpp_included_
