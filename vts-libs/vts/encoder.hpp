#ifndef vadstena_libs_vts_encoder_hpp_included_
#define vadstena_libs_vts_encoder_hpp_included_

#include <memory>

#include <boost/optional.hpp>
#include <boost/noncopyable.hpp>

#include "geo/srsdef.hpp"

#include "./tileset.hpp"
#include "./atlas.hpp"

namespace vadstena { namespace vts {

class Encoder : boost::noncopyable {
public:
    Encoder(const boost::filesystem::path &path
            , const TileSetProperties &properties, CreateMode mode);

    virtual ~Encoder() {}

    /** Starts encoding process from root tile.
     *
     *  Traverses tile tree in depth-first manner.  For each tile a call to
     *  generate() member function is made.
     *
     *  Subtree traversal is stopped when generate() returns (result ==
     *  noData).  Constraints are applied during traversal to filter out tiles
     *  that doesn't need to or cannot be generated.
     */
    TileSet run();

    /** Tree traversal algorithm constraints. See bellow.
     */
    struct Constraints;

    /** Type returned by generate(). Tile data + operation result.
     */
    struct TileResult {
        enum class Result {
            /** No data in this tile but there can be some data in the lower
             *  levels of the tree.
             */
            noDataYet

            /** No data here and nothing down there.
             */
            , noData

            /** Valid tile data.
             */
            , tile

            /** Valid tile source.
             */
            , source
       };

        TileResult(Result result = Result::noData) : result_(result) {}

        /** Generated tile. Throws if source has been called before.
         */
        Tile& tile();

        /** Generated tile source. result==Result::data. Throws if tile has
         * been called before.
         */
        TileSource& source();

        /** Generated tile.
         */
        const Tile& tile() const;

        /** Generated tile source.
         */
        const TileSource& source() const;

        /** Switches to no-data-yet state.
         */
        TileResult noDataYet() { result_ = Result::noDataYet; return *this; }

        /** Switches to no-data state.
         */
        TileResult noData() { result_ = Result::noData; return *this; }

        Result result() const { return result_; }

    private:
        void fail(const char *what) const;

        /** Result of generate() operation.
         */
        Result result_;

        boost::optional<Tile> tile_;
        boost::optional<TileSource> source_;
    };

protected:
    TileSetProperties properties() const;
    const registry::ReferenceFrame& referenceFrame() const;
    void setConstraints(const Constraints &constraints);

    const std::string& physicalSrsId() const;
    const registry::Srs& physicalSrs() const;

    const std::string& navigationSrsId() const;
    const registry::Srs& navigationSrs() const;

private:
    /** Called from run to generate mesh, atlas and navtile for every tile in
     *  the tree that satisfies constraints.
     *
     *  Thread safety info: this function iscalled in parallel via OpenMP when
     *  OpenMP support is compiled in. Therefore you have to wrap thread-unsafe
     *  operation in an OpenMP critical block, preferably using our UTILITY_OMP
     *  helper macro that takes into account whether OpenMP is compiled in or
     *  not.
     *
     *  Example:
     *      UTILITY_OMP(critical)
     *      {
     *          something;
     *          thread;
     *          unsafe;
     *      }
     */
    virtual TileResult
    generate(const TileId &tileId, const NodeInfo &nodeInfo) = 0;


    /** Called from run after whole tree is processed.
     */
    virtual void finish(TileSet &tileSet) = 0;

    // internals (pimpl)
    struct Detail;
    friend struct Detail;
    std::shared_ptr<Detail> detail_;
};

/** Tree traversal constraints.
 */
struct Encoder::Constraints {
    /** Generate will be called only for tiles having LOD in given range.
     */
    boost::optional<LodRange> lodRange;

    /** Extents: extents and their SRS.
     */
    struct Extents {
        math::Extents2 extents;
        std::string srs;

        Extents(const math::Extents2 &extents, const std::string &srs)
            : extents(extents), srs(srs)
        {}
    };

    /** Generate will be called only for tiles overlapping with given extents.
     */
    boost::optional<Extents> extents;

    /** Given extents are used to filter tiles until first valid tile is
     *  generated in given subtree if true. On by default.
     */
    bool useExtentsForFirstHit;

    /** Index with tree to descend, combined with all other constrains if
     *  nonnull.
     *  NB: this must be complete tree from the root!
     */
    const TileIndex *validTree;

    Constraints& setLodRange(const boost::optional<LodRange> &value);

    Constraints& setExtents(const boost::optional<Extents> &value);

    Constraints& setValidTree(const TileIndex *value);

    Constraints() : useExtentsForFirstHit(true), validTree(nullptr) {}
};

inline Encoder::Constraints&
Encoder::Constraints::setLodRange(const boost::optional<LodRange> &value)
{
    lodRange = value;
    return *this;
}

inline Encoder::Constraints&
Encoder::Constraints::setExtents(const boost::optional<Extents> &value)
{
    extents = value;
    return *this;
}

inline Encoder::Constraints&
Encoder::Constraints::setValidTree(const TileIndex *value)
{
    validTree = value;
    return *this;
}

inline const Tile& Encoder::TileResult::tile() const
{
    if (!tile_) { fail("no tile data"); }
    return *tile_;
}

inline const TileSource& Encoder::TileResult::source() const
{
    if (!source_) { fail("no tile source"); }
    return *source_;
}

inline Tile& Encoder::TileResult::tile()
{
    if (tile_) { return *tile_; }
    if (source_) {
        fail("cannot create tile data since there is already tile source");
    }
    tile_ = boost::in_place();
    result_ = Result::tile;
    return *tile_;
}

inline TileSource& Encoder::TileResult::source()
{
    if (source_) { return *source_; }
    if (tile_) {
        fail("cannot create tile source since there are already tile data");
    }
    source_ = boost::in_place();
    result_ = Result::source;
    return *source_;
}

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_encoder_hpp_included_
