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

#ifndef vtslibs_registry_extensions_tms_hpp_included_
#define vtslibs_registry_extensions_tms_hpp_included_

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
struct Tms {
    /** Time Map Service profile (global-geodetic, global-mercator, local)
     */
    enum class Profile;

    /** Root node ID. TMS node 0-0-0 is mapped to this rootId.
     *  Optional in serialized form, defaults to 0-0-0.
     */
    registry::ReferenceFrame::Division::Node::Id rootId;

    /** Flip Y coordinate (Y => 2^lod - 1 - Y) before mapping from TMS to node
     * ID.
     *
     * Optional in serialized form, defaults to true since TMS Y coordinate
     * grows up and VTS Y coordinate grows down.
     */
    bool flipY;

    /** Time Map Service profile.
     */
    Profile profile;

    /** Physical SRS used by TMS, may be different than reference frame's
     * physical SRS.
     */
    boost::optional<std::string> physicalSrs;

    /** Reported projection (i.e. SRS). Free form string.
     */
    std::string projection;

    Tms();

    static constexpr char key[] = "tms";
};

void load(Tms &tms, std::istream &is);
void save(const Tms &tms, std::ostream &os);

UTILITY_GENERATE_ENUM(Tms::Profile,
                      ((none))
                      ((globalGeodetic)("global-geodetic"))
                      ((globalMercator)("global-mercator"))
                      ((local)("local"))
                      )

inline Tms::Tms() : profile(Profile::none) {}

} } } // namespace vtslibs::registry::extensions

#endif // vtslibs_registry_extensions_tms_hpp_included_
