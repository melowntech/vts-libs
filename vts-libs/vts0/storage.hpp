/**
 * \file vts/storage.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Store.
 */

#ifndef vadstena_libs_vts0_storage_hpp_included_
#define vadstena_libs_vts0_storage_hpp_included_

#include <memory>
#include <map>

#include <boost/noncopyable.hpp>
#include <boost/filesystem/path.hpp>

#include "utility/runnable.hpp"

#include "./properties.hpp"

namespace vadstena { namespace vts0 {

/** Tile set descriptor.
 */
struct TileSetDescriptor {
    boost::filesystem::path path;

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

/** Storage create properties.
 */
struct StorageCreateProperties {
    /** Output tile set create properties.
     */
    CreateProperties createProperties;

    StorageCreateProperties() {}

    StorageCreateProperties(const CreateProperties &createProperties)
        : createProperties(createProperties)
    {}
};

class Storage : boost::noncopyable
{
public:
    typedef std::shared_ptr<Storage> pointer;

    StorageProperties getProperties() const;

    void addTileSet(const boost::filesystem::path &path
                    , utility::Runnable *runnable = nullptr);

    void addTileSets(const std::vector<boost::filesystem::path> &paths
                     , utility::Runnable *runnable = nullptr);

    void rebuildOutput();

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


// inline stuff

inline void Storage::addTileSet(const boost::filesystem::path &path
                                , utility::Runnable *runnable)
{
    return addTileSets({path}, runnable);
}

} } // namespace vadstena::vts0

#endif // vadstena_libs_vts0_storage_hpp_included_
