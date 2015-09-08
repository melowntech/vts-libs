/**
 * \file vts/tileset.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile set access.
 */

#ifndef vadstena_libs_vts_tileset_hpp_included_
#define vadstena_libs_vts_tileset_hpp_included_

#include <memory>
#include <cmath>
#include <list>
#include <string>
#include <array>

#include <boost/optional.hpp>
#include <boost/noncopyable.hpp>
#include <boost/filesystem/path.hpp>

#include "utility/runnable.hpp"

#include "../storage/lod.hpp"
#include "../storage/range.hpp"

#include "./basetypes.hpp"
#include "./mesh.hpp"
#include "./metatile.hpp"
#include "./atlas.hpp"

namespace vadstena { namespace vts {

/** Driver that implements physical aspects of tile set.
 */
class Driver;

/** TileSet interface.
 */
class TileSet {
public:
    ~TileSet();

    Mesh mesh(const TileId &tileId) const;
    void mesh(const TileId &tileId, const Mesh &mesh) const;

    Atlas atlas(const TileId &tileId) const;
    void atlas(const TileId &tileId, const Atlas &atlas) const;

    MetaNode metaNode(const TileId &tileId) const;
    void metaNode(const TileId &tileId, const MetaNode node) const;

    bool exists(const TileId &tileId) const;

    void flush();

    /** Starts new transaction. (R/W)
     */
    void begin(utility::Runnable *runnable = nullptr);

    /** Commits pending transaction. Calls flush to ensure any changes are
     *  propagated to the backing store.
     *  Commit fails if there is no transaction in progress.
     */
    void commit();

    /** Rolls back pending transaction.
     *  Rollback fails if there is no transaction in progress.
     */
    void rollback();

    /** Starts watching runnable without entering a transaction.
     */
    void watch(utility::Runnable *runnable);

    /** Check for pending transaction.
     */
    bool inTx() const;

    /** Is the tile set empty (i.e. has it no tile?)
     */
    bool empty() const;

    /** Remove storage.
     */
    void drop();

    /** Returns lod range covered by tiles.
     */
    LodRange lodRange() const;

    /** Internals. Public to ease library developers' life, not to allow users
     *  to put their dirty hands in the tileset's guts!
     */
    struct Detail;

private:
    TileSet(const std::shared_ptr<Driver> &driver);

    std::unique_ptr<Detail> detail_;
    Detail& detail() { return *detail_; }
    const Detail& detail() const { return *detail_; }

public:
    /** Needed to instantiate.
     */
    class Factory; friend class Factory;

    struct Accessor; friend class Accessor;
};

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_tileset_hpp_included_
