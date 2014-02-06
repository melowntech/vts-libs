#ifndef vadstena_libs_tilestorage_driver_hpp_included_
#define vadstena_libs_tilestorage_driver_hpp_included_

#include <iostream>

#include "dbglog/dbglog.hpp"
#include "utility/gccversion.hpp"

#include "./tileset.hpp"

namespace vadstena { namespace tilestorage {

class Driver : boost::noncopyable {
public:
    struct OStream;
    struct IStream;

    typedef std::shared_ptr<Driver> pointer;

    virtual ~Driver() {};

    Properties loadProperties();

    void saveProperties(const Properties &properties);

    std::shared_ptr<OStream> metatileOutput(const TileId tileId);

    std::shared_ptr<IStream> metatileInput(const TileId tileId);

    std::shared_ptr<OStream> tileIndexOutput();

    std::shared_ptr<IStream> tileIndexInput();

    void saveMesh(const TileId tileId, const Mesh &mesh);

    Mesh loadMesh(const TileId tileId);

    void saveAtlas(const TileId tileId, const Atlas &atlas
                   , short textureQuality);

    Atlas loadAtlas(const TileId tileId);

    bool readOnly() const { return readOnly_; }

    void wannaWrite(const std::string &what) const;

    class Factory;

    template <typename DriverClass> static void registerDriver();

    static Driver::pointer create(const std::string type
                                  , const std::string location
                                  , const CreateProperties &properties
                                  , CreateMode mode);

    static Driver::pointer open(const std::string type
                                , const std::string location
                                , OpenMode mode);

protected:
    Driver(bool readOnly) : readOnly_(readOnly) {}

private:
    virtual Properties loadProperties_impl() = 0;

    virtual void saveProperties_impl(const Properties &properties) = 0;

    virtual std::shared_ptr<OStream> metatileOutput_impl(const TileId tileId)
        = 0;

    virtual std::shared_ptr<IStream> metatileInput_impl(const TileId tileId)
        = 0;

    virtual std::shared_ptr<OStream> tileIndexOutput_impl() = 0;

    virtual std::shared_ptr<IStream> tileIndexInput_impl() = 0;

    virtual void saveMesh_impl(const TileId tileId, const Mesh &mesh) = 0;

    virtual Mesh loadMesh_impl(const TileId tileId) = 0;

    virtual void saveAtlas_impl(const TileId tileId, const Atlas &atlas
                                , short textureQuality) = 0;

    virtual Atlas loadAtlas_impl(const TileId tileId) = 0;

    static void registerDriver(const std::shared_ptr<Factory> &factory);

    bool readOnly_;
};

class Driver::OStream {
public:
    OStream() {}
    virtual ~OStream() {}
    virtual operator std::ostream&() = 0;
    virtual void close() = 0;
};

class Driver::IStream {
public:
    IStream() {}
    virtual ~IStream() {}
    virtual operator std::istream&() = 0;
    virtual void close() = 0;
};

class Driver::Factory {
public:
    typedef std::shared_ptr<Factory> pointer;
    Factory(const std::string &type) : type(type) {}

    virtual ~Factory() {}

    virtual Driver::pointer create(const std::string location
                                   , const CreateProperties &properties
                                   , CreateMode mode) const = 0;

    virtual Driver::pointer open(const std::string location
                                 , OpenMode mode) const = 0;

    const std::string type;
};

#define VADSTENA_TILESTORAGE_DRIVER_FACTORY(DRIVER_TYPE, DRIVER_CLASS)  \
    class Factory : public Driver::Factory {                            \
    public:                                                             \
        Factory() : Driver::Factory(DRIVER_TYPE) {}                     \
                                                                        \
        virtual Driver::pointer create(const std::string location       \
                                       , const CreateProperties &properties \
                                       , CreateMode mode) const override \
        {                                                               \
            return std::make_shared<DRIVER_CLASS>(location, properties, mode); \
        }                                                               \
                                                                        \
        virtual Driver::pointer open(const std::string location         \
                                       , OpenMode mode) const override  \
        {                                                               \
            return std::make_shared<DRIVER_CLASS>(location, mode);      \
        }                                                               \
    }

// inline stuff

template <typename DriverClass>
void Driver::registerDriver()
{
    registerDriver(std::make_shared<typename DriverClass::Factory>());
}

inline Properties Driver::loadProperties() {
    return loadProperties_impl();
}

inline void Driver::saveProperties(const Properties &properties)
{
    return saveProperties_impl(properties);
}

inline std::shared_ptr<Driver::OStream>
Driver::metatileOutput(const TileId tileId)
{
    return metatileOutput_impl(tileId);
}

inline std::shared_ptr<Driver::IStream>
Driver::metatileInput(const TileId tileId)
{
    return metatileInput_impl(tileId);
}

inline std::shared_ptr<Driver::OStream> Driver::tileIndexOutput()
{
    return tileIndexOutput_impl();
}

inline std::shared_ptr<Driver::IStream> Driver::tileIndexInput()
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
