#ifndef vadstena_libs_vts_basetypes_hpp_included_
#define vadstena_libs_vts_basetypes_hpp_included_

#include <string>

#include "math/geometry_core.hpp"

#include "utility/enum-io.hpp"

#include "../storage/lod.hpp"
#include "../storage/range.hpp"
#include "../registry.hpp"

namespace vadstena { namespace vts {

using storage::Lod;
using storage::Range;
using storage::LodRange;

using registry::TileRange;

typedef registry::ReferenceFrame::Division::Node RFNode;

/** Tile identifier (index in 3D space): LOD + tile index from upper-left corner
 *  in tile grid.
 *
 *  Works as an empty wrapper around reference frame node's ID
 */
typedef RFNode::Id TileId;

struct Child : TileId {
    unsigned int index;

    Child(const TileId &tileId = TileId(), unsigned int index = 0)
        : TileId(tileId), index(index)
    {}

    Child(Lod lod, unsigned int x, unsigned int y, unsigned int index)
        : TileId(lod, x, y), index(index)
    {}
};

typedef std::array<Child, 4> Children;

/** Open mode
 */
enum class OpenMode {
    readOnly     //!< only getters are allowed
    , readWrite  //!< both getters and setters are allowed
};

enum class CreateMode {
    failIfExists //!< creation fails if tile set/storage already exists
    , overwrite  //!< existing tile set/storage is replace with new one
};

typedef std::string TilesetId;
typedef std::vector<TilesetId> TilesetIdList;
typedef std::set<TilesetId> TilesetIdSet;

struct Glue {
    typedef TilesetIdList Id;
    Id id;
    std::string path;

    typedef std::map<Id, Glue> map;
    typedef std::vector<Glue> list;
    typedef std::vector<Id> Ids;

    Glue() {}
    Glue(const Id &id, const std::string &path) : id(id), path(path) {}

    /** Returns true if glue references given tileset
     */
    bool references(const std::string &tilesetId) const;
};

/** Tileset with its glues.
 */
struct TileSetGlues {
    struct EnhancedGlue : Glue {
        template <typename ...Args>
        EnhancedGlue(Args &&...args) : Glue(std::forward<Args>(args)...) {}

        typedef std::vector<int> Indices;
        Indices indices;

        typedef std::vector<EnhancedGlue> list;
    };

    TilesetId tilesetId;

    EnhancedGlue::list glues;

    TileSetGlues(const TilesetId &tilesetId, const Glue::list glues)
        : tilesetId(tilesetId), glues(glues.begin(), glues.end()) {}

    typedef std::vector<TileSetGlues> list;
};

/** Calculates glue priority order for given set of tilesets.
 *
 * Tile set order is given by order of input list!
 *
 * \param mapping between tilesets and their glues.
 */
TileSetGlues::list glueOrder(const TileSetGlues::list &in);

// inline stuff

UTILITY_GENERATE_ENUM_IO(OpenMode,
    ((readOnly))
    ((readWrite))
)

UTILITY_GENERATE_ENUM_IO(CreateMode,
    ((failIfExists))
    ((overwrite))
)

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_basetypes_hpp_included_
