#ifndef vadstena_libs_vts_metatile_hpp_included_
#define vadstena_libs_vts_metatile_hpp_included_

#include <cstdint>
#include <iosfwd>
#include <functional>

#include <boost/optional.hpp>

#include "./basetypes.hpp"

namespace vadstena { namespace vts {

namespace detail {
/** Invalid pixel size to mark tiles with no data
 */
constexpr float invalidPixelSize = std::numeric_limits<float>::max();
constexpr float invalidGsd = -1;
constexpr float invalidCoarseness = -1;

} // namespace detail


/** Extra metadata for one tile.
 */
struct TileMetadata {
    enum { HMSize = 5 };
    typedef float Heightmap[HMSize][HMSize];
    Heightmap heightmap;
    float coarseness;
    float gsd;

    struct Mask { enum {             // mask bitfields
          coarseness = 0x01
        , gsd = 0x02
    }; };

    typedef int MaskType;

    // NB: update initializer when HMSize changes!
    TileMetadata()
        : heightmap{{0.f}, {0.f}, {0.f}, {0.f}, {0.f}}
        , coarseness(detail::invalidCoarseness)
        , gsd(detail::invalidGsd)
    {}
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

    void dump(std::ostream &f, const unsigned int version) const;
    void load(std::istream &f, const unsigned int version);
};

// metatile IO

typedef std::function<void(const TileId &tileId, const MetaNode &node
                           , std::uint8_t childFlags)> MetaNodeLoader;

typedef std::function<void(const TileId &tileId)> MetaNodeNotify;

void loadMetatile(std::istream &f, const TileId &tileId
                  , const MetaNodeLoader &loader
                  , const MetaNodeNotify &notify = MetaNodeNotify());

struct MetaNodeSaver
{
    typedef std::function<void(std::ostream &os)> MetaTileSaver;

    virtual void saveTile(const TileId &metaId, const MetaTileSaver &saver)
        const = 0;
    virtual const MetaNode* getNode(const TileId &tileId) const = 0;
    virtual ~MetaNodeSaver() {}
};

typedef std::function<MetaNode*(const TileId &tileId)> MetaNodeGetter;

void saveMetatile(const TileId &foat
                  , const LodLevels &metaLevels
                  , const MetaNodeSaver &saver);


// inline method implementation

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
dump(std::basic_ostream<CharT, Traits> &os
     , const TileMetadata &m
     , const std::string &prefix = std::string())
{
    os << prefix << "gsd: " << m.gsd;
    os << prefix << "\ncoarseness: " << m.coarseness << "\n";
    for (int j(0); j < TileMetadata::HMSize; ++j) {
        os << prefix << "heightmap[" << j << "]: ";
        for (int i(0); i < TileMetadata::HMSize; ++i) {
            if (i) { os << ", "; }
            os << m.heightmap[j][i];
        }
        os << "\n";
    }

    return os;
}

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

    dump(os, static_cast<const TileMetadata&>(n), prefix);

    return os;
}

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_metatile_hpp_included_
