#ifndef vadstena_libs_tilestorage_driver_hpp_included_
#define vadstena_libs_tilestorage_driver_hpp_included_

#include <iostream>

#include "../tilestorage.hpp"

namespace vadstena { namespace tilestorage {

class Driver : boost::noncopyable {
public:
    typedef std::shared_ptr<Driver> pointer;

    virtual ~Driver() {};

    Properties loadProperties();

    void saveProperties(const Properties &properties);

    std::shared_ptr<std::ostream> metatileOutput(const TileId tileId);

    std::shared_ptr<std::istream> metatileInput(const TileId tileId);

    std::shared_ptr<std::ostream> tileIndexOutput();

    std::shared_ptr<std::istream> tileIndexInput();

    void saveMesh(const TileId tileId, const Mesh &mesh);

    Mesh loadMesh(const TileId tileId);

    void saveAtlas(const TileId tileId, const Atlas &atlas
                   , short textureQuality);

    Atlas loadAtlas(const TileId tileId);

    bool readOnly() const { return readOnly_; }

    void wannaWrite(const std::string &what) const;

protected:
    Driver(bool readOnly) : readOnly_(readOnly) {}

private:
    virtual Properties loadProperties_impl() = 0;

    virtual void saveProperties_impl(const Properties &properties) = 0;

    virtual std::shared_ptr<std::ostream>
    metatileOutput_impl(const TileId tileId) = 0;

    virtual std::shared_ptr<std::istream>
    metatileInput_impl(const TileId tileId) = 0;

    virtual std::shared_ptr<std::ostream> tileIndexOutput_impl() = 0;

    virtual std::shared_ptr<std::istream> tileIndexInput_impl() = 0;

    virtual void saveMesh_impl(const TileId tileId, const Mesh &mesh) = 0;

    virtual Mesh loadMesh_impl(const TileId tileId) = 0;

    virtual void saveAtlas_impl(const TileId tileId, const Atlas &atlas
                                , short textureQuality) = 0;

    virtual Atlas loadAtlas_impl(const TileId tileId) = 0;

    bool readOnly_;
};

// inline stuff

inline Properties Driver::loadProperties() {
    return loadProperties_impl();
}

inline void Driver::saveProperties(const Properties &properties)
{
    return saveProperties_impl(properties);
}

inline std::shared_ptr<std::ostream> Driver::metatileOutput(const TileId tileId)
{
    return metatileOutput_impl(tileId);
}

inline std::shared_ptr<std::istream> Driver::metatileInput(const TileId tileId)
{
    return metatileInput_impl(tileId);
}

inline std::shared_ptr<std::ostream> Driver::tileIndexOutput()
{
    return tileIndexOutput_impl();
}

inline std::shared_ptr<std::istream> Driver::tileIndexInput()
{
    return tileIndexInput_impl();
}

inline void Driver::saveMesh(const TileId tileId, const Mesh &mesh)
{
    return saveMesh_impl(tileId, mesh);
}

inline Mesh Driver::loadMesh(const TileId tileId)
{
    return loadMesh_impl(tileId);
}

inline void Driver::saveAtlas(const TileId tileId, const Atlas &atlas
                              , short textureQuality)
{
    return saveAtlas_impl(tileId, atlas, textureQuality);
}

inline Atlas Driver::loadAtlas(const TileId tileId)
{
    return loadAtlas_impl(tileId);
}

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_driver_hpp_included_
