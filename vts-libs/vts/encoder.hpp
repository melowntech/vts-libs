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
#ifndef vtslibs_vts_encoder_hpp_included_
#define vtslibs_vts_encoder_hpp_included_

#include <memory>

#include <boost/any.hpp>
#include <boost/optional.hpp>
#include <boost/noncopyable.hpp>

#include "dbglog/dbglog.hpp"

#include "geo/srsdef.hpp"

#include "tileset.hpp"
#include "atlas.hpp"

namespace vtslibs { namespace vts {

class Encoder : boost::noncopyable {
public:
    /** Encoder options.
     */
    class Options {
    public:
        bool flush() const { return flush_; };
        Options& flush(bool value) { flush_ = value; return *this; };

        dbglog::level level() const { return level_; };
        Options& level(dbglog::level value) { level_ = value; return *this; };

        Options() : flush_(true), level_(dbglog::info3) {}

    private:
        bool flush_;
        dbglog::level level_;
    };

    /** Creates encoder for new tileset.
     */
    Encoder(const boost::filesystem::path &path
            , const TileSetProperties &properties, CreateMode mode
            , const Options &options = Options());

    /** Creates encoder for existing tileset.
     */
    Encoder(TileSet &tileset, const Options &options = Options());

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

            /** Influence marker.
             */
            , influenced
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
        TileResult& noDataYet() { result_ = Result::noDataYet; return *this; }

        /** Switches to no-data state.
         */
        TileResult& noData() { result_ = Result::noData; return *this; }

        /** Switches to influenced state.
         */
        TileResult& influenced() {
            result_ = Result::influenced; return *this;
        }

        Result result() const { return result_; }

        /** Are there any userdata.
         */
        bool hasUserData() const { return !userData_.empty(); }

        /** Returns pointer to userdata. Can be NULL.
         */
        template <typename T>
        const T* userData() const {
            return boost::any_cast<T>(&userData_);
        }

        /** Returns reference to stored userdata or to a default if not set.
         */
        template <typename T>
        const T& userDataWithDefault(const T &defaultValue) const {
            if (const auto *value = boost::any_cast<T>(&userData_)) {
                return *value;
            }
            return defaultValue;
        }

        /** Sets userdata and returns reference to them.
         */
        template <typename T>
        T& userData(T &&userData) {
            userData_ = std::move(userData);
            return boost::any_cast<T&>(userData_);
        }

        bool hasMesh() const;
        bool hasAtlas() const;
        bool hasNavtile() const;

    private:
        void fail(const char *what) const;

        /** Result of generate() operation.
         */
        Result result_;

        boost::optional<Tile> tile_;
        boost::optional<TileSource> source_;

        boost::any userData_;
    };

protected:
    TileSetProperties properties() const;
    const registry::ReferenceFrame& referenceFrame() const;
    void setConstraints(const Constraints &constraints);

    const std::string& physicalSrsId() const;
    const registry::Srs& physicalSrs() const;

    const std::string& navigationSrsId() const;
    const registry::Srs& navigationSrs() const;

    std::size_t threadIndex() const;

    /** Notifies encoder about estimated number of tiles to be generated.
     *
     *  Number is used in logging.
     *
     *  \param count estimated number of tiles to generate, 0 turn off
     */
    void setEstimatedTileCount(std::size_t count = 0);

    /** Updates estimated tile count by given number.
     */
    void updateEstimatedTileCount(int diff);

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
    generate(const TileId &tileId, const NodeInfo &nodeInfo
             , const TileResult &parentTile) = 0;

    /** Called from run after whole tree is processed.
     */
    virtual void finish(TileSet &tileSet) = 0;

    /** Called before right before first call to generate() to report number of
     *  threads.
     */
    virtual void threadCount(std::size_t) {};

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
     *  In conflict with extentsGenerator
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

    /** These nodes will be invalidated in the descend tree before launching
     *  encoding algorithm.
     */
    TileId::list invalidNodes;

    typedef std::function<math::Extents2(const std::string&)> ExtentsGenerator;

    /** Alternative to single extents: function to generate extents in all
     *  spatial division SRS. In conflict with extents.
     */
    ExtentsGenerator extentsGenerator;

    Constraints& setLodRange(const boost::optional<LodRange> &value);

    Constraints& setExtents(const boost::optional<Extents> &value);

    Constraints& setValidTree(const TileIndex *value);

    Constraints& setExtentsGenerator(const ExtentsGenerator &value);

    Constraints& setInvalidNodes(const TileId::list &value);

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

inline Encoder::Constraints&
Encoder::Constraints::setExtentsGenerator
(const Encoder::Constraints::ExtentsGenerator &value)
{
    extentsGenerator = value;
    return *this;
}

inline Encoder::Constraints&
Encoder::Constraints::setInvalidNodes(const TileId::list &value)
{
    invalidNodes = value;
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

inline bool Encoder::TileResult::hasMesh() const
{
    if (tile_) { return bool(tile_->mesh); }
    if (source_) { return bool(source_->mesh); }
    return false;
}

inline bool Encoder::TileResult::hasAtlas() const
{
    if (tile_) { return bool(tile_->atlas); }
    if (source_) { return bool(source_->atlas); }
    return false;
}

inline bool Encoder::TileResult::hasNavtile() const
{
    if (tile_) { return bool(tile_->navtile); }
    if (source_) { return bool(source_->navtile); }
    return false;
}

} } // namespace vtslibs::vts

#endif // vtslibs_vts_encoder_hpp_included_
