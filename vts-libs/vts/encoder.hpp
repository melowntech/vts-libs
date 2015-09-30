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
    void run();

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
            , data
        };

        /** Result of generate() operation.
         */
        Result result;

        /** Tile data. Valid only when (result == data).
         */
        Tile tile;

        TileResult(Result result = Result::noData) : result(result) {}
    };

protected:
    TileSetProperties properties() const;
    const registry::ReferenceFrame& referenceFrame() const;
    void setConstraints(const Constraints &constraints);
    const registry::Srs& physicalSrs() const;

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
    generate(const TileId &tileId
             , const registry::ReferenceFrame::Division::Node &node
             , const math::Extents2 &divisionExtents) = 0;


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

    /** Generate will be called only for tiles overlapping with given extents.
     */
    boost::optional<math::Extents2> extents;

    /** Given extents are used to filter tiles until first valid tile is
     *  generated in given subtree if true. On by default.
     */
    bool useExtentsForFirstHit;

    Constraints& setLodRange(const boost::optional<LodRange> &value);

    Constraints& setExtents(const boost::optional<math::Extents2> &value);

    Constraints() : useExtentsForFirstHit(true) {}
};

inline Encoder::Constraints&
Encoder::Constraints::setLodRange(const boost::optional<LodRange> &value)
{
    lodRange = value;
    return *this;
}

inline Encoder::Constraints&
Encoder::Constraints::setExtents(const boost::optional<math::Extents2> &value)
{
    extents = value;
    return *this;
}

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_encoder_hpp_included_
