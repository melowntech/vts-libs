#ifndef vadstena_libs_tilestorage_driver_flat_hpp_included_
#define vadstena_libs_tilestorage_driver_flat_hpp_included_

#include <set>

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

#include "jsoncpp/json.hpp"

#include "../driver.hpp"

namespace vadstena { namespace tilestorage {

class FlatDriver : public Driver {
public:
    /** Creates new storage and sets properties. Existing storage is overwritten
     *  only if mode == CreateMode::overwrite.
     */
    FlatDriver(const boost::filesystem::path &root
               , const CreateProperties &properties
               , CreateMode mode);

    /** Opens storage.
     */
    FlatDriver(const boost::filesystem::path &root
               , OpenMode mode);

    virtual ~FlatDriver();

    VADSTENA_TILESTORAGE_DRIVER_FACTORY("flat", FlatDriver);

private:
    virtual Properties loadProperties_impl() override;

    virtual void saveProperties_impl(const Properties &properties) override;

    virtual std::shared_ptr<Driver::OStream>
    metatileOutput_impl(const TileId tileId) override;

    virtual std::shared_ptr<Driver::IStream>
    metatileInput_impl(const TileId tileId) override;

    virtual std::shared_ptr<Driver::OStream> tileIndexOutput_impl() override;

    virtual std::shared_ptr<Driver::IStream> tileIndexInput_impl() override;

    virtual void saveMesh_impl(const TileId tileId, const Mesh &mesh) override;

    virtual Mesh loadMesh_impl(const TileId tileId) override;

    virtual void saveAtlas_impl(const TileId tileId, const Atlas &atlas
                                , short textureQuality) override;

    virtual Atlas loadAtlas_impl(const TileId tileId) override;

    virtual void begin_impl() override;

    virtual void commit_impl() override;

    virtual void rollback_impl() override;

    boost::filesystem::path readPath(const boost::filesystem::path &path);

    boost::filesystem::path writePath(const boost::filesystem::path &path);

    /** Backing root.
     */
    const boost::filesystem::path root_;

    /** temporary files backing root.
     */
    const boost::filesystem::path tmp_;

    /** Parsed config as JSON tree.
     */
    Json::Value config_;

    /** File in pending transaction.
     */
    boost::optional<std::set<boost::filesystem::path> > txFiles_;
};

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_driver_flat_hpp_included_
