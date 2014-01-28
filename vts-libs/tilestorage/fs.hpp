#ifndef vadstena_libs_tilestorage_fs_hpp_included_
#define vadstena_libs_tilestorage_fs_hpp_included_

#include <map>

#include <boost/filesystem.hpp>

#include "utility/gccversion.hpp"

#include "../tilestorage.hpp"

namespace vadstena { namespace tilestorage {

namespace detail {
    typedef std::map<TileId, MetaNode> MetaNodeMap;
}

class FileSystemStorage : public Storage {
public:
    ~FileSystemStorage();

    /** Get tile content.
     * \param tileId idetifier of tile to return.
     * \return read tile
     * \throws Error if tile with given tileId is not found
     */
    virtual Tile getTile(const TileId &tileId) override;

    /** Set new tile content.
     * \param tileId idetifier of tile write.
     * \param mesh new tile's mesh
     * \param atlas new tile's atlas
     */
    virtual void setTile(const TileId &tileId, const Mesh &mesh
                         , const Atlas &atlas) override;

    /** Get tile's metadata.
     * \param tileId idetifier of metatile to return.
     * \return read metadata
     * \throws Error if metadate with given tileId is not found
     */
    virtual MetaNode getMetaData(const TileId &tileId) override;

    /** Set tile's metadata.
     * \param tileId idetifier of tile to write metadata to.
     * \param meta new tile's metadata
     */
    virtual void setMetaData(const TileId &tileId, const MetaNode &meta)
        override;

    /** Query for tile's existence.
     * \param tileId identifier of queried tile
     * \return true if given tile exists, false otherwise
     *
     ** NB: Should be optimized.
     */
    virtual bool tileExists(const TileId &tileId) override;

    /** Get storage propeties.
     * \return storage properties
     */
    virtual Properties getProperties() override;

    /** Set new storage propeties.
     * \param new storage properties
     */
    virtual void setProperties(const Properties &properties) override;

    virtual void flush() override;

private:
    /** Grant access to factory.
     */
    friend class Storage::Factory;

    FileSystemStorage(const std::string &root, OpenMode mode);

    FileSystemStorage(const std::string &root, const Properties &properties
                      , CreateMode mode);

    boost::filesystem::path root_;
};

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_fs_hpp_included_
