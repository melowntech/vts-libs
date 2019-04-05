/**
 * Copyright (c) 2019 Melown Technologies SE
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

/**
 * \file registry/extensions.hpp
 * \author Vaclav Blazek <vaclav.blazek@melown.com>
 */

#ifndef vtslibs_registry_extensions_wmts_hpp_included_
#define vtslibs_registry_extensions_wmts_hpp_included_

#include <iosfwd>

#include <boost/optional.hpp>
#include <boost/any.hpp>

#include "utility/enum-io.hpp"

#include "../referenceframe.hpp"

namespace vtslibs { namespace registry { namespace extensions {

/** Following extensions are automatically de/serialized by reference frame.
 */

/** OSGeo Tile Map Service extension -- how to map reference frame to be used
 * for TMS.
 *
 *  http://wiki.osgeo.org/wiki/Tile_Map_Service_Specification
 */
struct Wmts {
    /** Time Map Service profile (global-geodetic, global-mercator, local)
     */
    enum class Profile;

    /** SRS identifier of root node(s) with content.
     *
     *  If there are multiple nodes with given SRS then all nodes are included
     *  in the output, i.e. extents, tile ranges etc. are union of all nodes.
     *
     * Optinal, may be omitted if there is only one root node in the reference
     * frame definition.
     */
    boost::optional<std::string> content;

    /** Extents SRS. Defaults to content. All extents reporting and tiling is
     *  performed in this SRS.
     */
    boost::optional<std::string> extentsSrs;

    /** Reported projection (i.e. SRS). Free form string.
     */
    std::string projection;

    /** Well known scale set, if known.
     */
    boost::optional<std::string> wellKnownScaleSet;

    static constexpr char key[] = "wmts";
};

void load(Wmts &wmts, std::istream &is);
void save(const Wmts &wmts, std::ostream &os);

} } } // namespace vtslibs::registry::extensions

#endif // vtslibs_registry_extensions_tms_wmts_included_
