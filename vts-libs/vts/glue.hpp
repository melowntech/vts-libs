#ifndef vadstena_libs_vts_glue_hpp_included_
#define vadstena_libs_vts_glue_hpp_included_

#include <string>

#include "math/geometry_core.hpp"

#include "utility/enum-io.hpp"

#include "./basetypes.hpp"

namespace vadstena { namespace vts {

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

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_glue_hpp_included_
