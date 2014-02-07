/**
 * \file tilestorage/storage.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Store.
 */

#ifndef vadstena_libs_tilestorage_storage_hpp_included_
#define vadstena_libs_tilestorage_storage_hpp_included_

#include <memory>
#include <map>

#include <boost/noncopyable.hpp>
#include <boost/filesystem/path.hpp>

#include "./types.hpp"

namespace vadstena { namespace tilestorage {

/** Tile set descriptor.
 */
struct TileSetDescriptor {
    Locator locator;

    typedef std::map<std::string, TileSetDescriptor> map;

    TileSetDescriptor() {}
};

/** Storage properties.
 */
struct StorageProperties {
    /** Set of input tile set descriptors.
     */
    TileSetDescriptor::map inputSets;

    /** Output tile sets.
     */
    TileSetDescriptor outputSet;

    StorageProperties() {}
};

class Storage : boost::noncopyable
{
public:
    typedef std::shared_ptr<Storage> pointer;

    void addTileSet(const Locator &locator);

    void removeTileSet(const std::string &id);

    /** Needed to instantiate.
     */
    class Factory;
    friend class Factory;

private:
    Storage(const boost::filesystem::path &root, bool readOnly);

    struct Detail;
    std::unique_ptr<Detail> detail_;
    Detail& detail() { return *detail_; }
    const Detail& detail() const { return *detail_; }
};

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_storage_hpp_included_
