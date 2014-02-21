#ifndef vadstena_libs_tilestorage_metatile_hpp_included_
#define vadstena_libs_tilestorage_metatile_hpp_included_

#include <cstdint>
#include <iosfwd>
#include <functional>

#include "./basetypes.hpp"

namespace vadstena { namespace tilestorage {

namespace detail {
/** Invalid pixel size to mark tiles with no data
 */
constexpr float invalidPixelSize = std::numeric_limits<float>::max();

} // namespace detail


/** Extra metadata for one tile.
 */
struct TileMetadata {
    enum { HMSize = 5 };
    float heightmap[HMSize][HMSize];
};

//! Meta-data for one tile
struct MetaNode : TileMetadata
{
    // NB: pulls in HMSize and heightmap!

    float zmin, zmax;
    float pixelSize[2][2];

    MetaNode()
        : zmin(std::numeric_limits<float>::infinity())
        , zmax(-std::numeric_limits<float>::infinity())
    {
        invalidate();
    }

    void calcParams(const geometry::Obj &mesh, const math::Size2 &atlasSize);

    /** Make node invalid (i.e. having no real data).
     */
    void invalidate() {
        pixelSize[0][0] = pixelSize[0][1] = detail::invalidPixelSize;
        pixelSize[1][0] = pixelSize[1][1] = detail::invalidPixelSize;
    }

    /** Does tile exist?
     */
    bool exists() const {
        return pixelSize[0][0] < detail::invalidPixelSize;
    }

    void dump(std::ostream &f) const;
    void load(std::istream &f);
};

// metatile IO

typedef std::function<void(const TileId &tileId, const MetaNode &node
                           , std::uint8_t childFlags)> MetaNodeLoader;

void loadMetatile(std::istream &f, long baseTileSize, const TileId &tileId
                  , const MetaNodeLoader &loader);

struct MetaNodeSaver
{
    typedef std::function<void(std::ostream &os)> MetaTileSaver;

    virtual void saveTile(const TileId &metaId, const MetaTileSaver &saver)
        const = 0;
    virtual MetaNode* getNode(const TileId &tileId) const = 0;
    virtual ~MetaNodeSaver() {}
};

typedef std::function<MetaNode*(const TileId &tileId)> MetaNodeGetter;

void saveMetatile(long baseTileSize, const TileId &foat
                  , const LodLevels &metaLevels
                  , const MetaNodeSaver &saver);


// inline method implementation

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
dump(std::basic_ostream<CharT, Traits> &os
     , const MetaNode &n
     , const std::string &prefix = std::string())
{
    os << prefix << "z = " << n.zmin << ", " << n.zmax << '\n'
       << prefix << "pixelSize = "
       << n.pixelSize[0][0] << ", " << n.pixelSize[0][1]
       << ", " << n.pixelSize[1][0] << ", " << n.pixelSize[1][1] << '\n'
        ;

    return os;
}

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_metatile_hpp_included_
