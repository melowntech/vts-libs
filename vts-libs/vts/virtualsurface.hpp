#ifndef vtslibs_vts_virtualsurface_hpp_included_
#define vtslibs_vts_virtualsurface_hpp_included_

#include <string>

#include "math/geometry_core.hpp"

#include "./basetypes.hpp"

namespace vtslibs { namespace vts {

struct VirtualSurface {
    static constexpr char TilesetMappingPath[] = "tileset.map";
    static constexpr char TilesetMappingContentType[]
    = "application/octet-stream";

    typedef TilesetIdList Id;
    Id id;
    std::string path;

    typedef std::map<Id, VirtualSurface> map;
    typedef std::vector<VirtualSurface> list;
    typedef std::vector<Id> Ids;

    VirtualSurface() {}
    VirtualSurface(const Id &id) : id(id) {}
    VirtualSurface(const Id &id, const std::string &path)
        : id(id), path(path) {}

    /** Returns true if virtual surface references given tileset
     */
    bool references(const std::string &tilesetId) const;
};

} } // namespace vtslibs::vts

#endif // vtslibs_vts_virtualsurface_hpp_included_
