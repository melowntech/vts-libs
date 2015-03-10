#ifndef vadstena_libs_tilestorage_driver_hpp_included_
#define vadstena_libs_tilestorage_driver_hpp_included_

#include <map>

#include "./types.hpp"
#include "./properties.hpp"
#include "./streams.hpp"

namespace vadstena { namespace tilestorage {

class Driver : boost::noncopyable {
public:
    typedef std::shared_ptr<Driver> pointer;

    virtual ~Driver() {};

    OStream::pointer output(File type);

    IStream::pointer input(File type) const;

    OStream::pointer output(const TileId tileId, TileFile type);

    IStream::pointer input(const TileId tileId, TileFile type) const;

    void remove(const TileId tileId, TileFile type);

    bool readOnly() const { return readOnly_; }

    void wannaWrite(const std::string &what) const;

    void begin();

    void commit();

    void rollback();

    void flush();

    /** Drop storage.
     */
    void drop();

    /** Update driver stuff. Currenlty, only embedded browser is updated.
     */
    void update();

    DriverProperties properties() const;

    class Factory;

    template <typename DriverClass> static void registerDriver();

    static Driver::pointer create(Locator locator
                                  , CreateMode mode
                                  , const StaticProperties &properties);

    static Driver::pointer open(Locator locator, OpenMode mode);

    static std::map<std::string, std::string> listSupportedDrivers();

protected:
    Driver(bool readOnly) : readOnly_(readOnly) {}

private:
    virtual OStream::pointer output_impl(const File type) = 0;

    virtual IStream::pointer input_impl(File type) const = 0;

    virtual OStream::pointer
    output_impl(const TileId tileId, TileFile type) = 0;

    virtual IStream::pointer
    input_impl(const TileId tileId, TileFile type) const = 0;

    virtual void remove_impl(const TileId tileId, TileFile type) = 0;

    virtual void begin_impl() = 0;

    virtual void commit_impl() = 0;

    virtual void rollback_impl() = 0;

    virtual void flush_impl() {};

    virtual void drop_impl() = 0;

    virtual void update_impl() = 0;

    virtual DriverProperties properties_impl() const = 0;

    static void registerDriver(const std::shared_ptr<Factory> &factory);

    static std::string detectType(const std::string &location);

    bool readOnly_;
};

class Driver::Factory {
public:
    typedef std::shared_ptr<Factory> pointer;
    Factory(const std::string &type) : type(type) {}

    virtual ~Factory() {}

    virtual Driver::pointer create(const std::string location
                                   , CreateMode mode
                                   , const StaticProperties &properties)
        const = 0;

    virtual Driver::pointer open(const std::string location
                                 , OpenMode mode) const = 0;

    virtual std::string help() const = 0;

    /** Returns type of tileset at given location if location makes sense.
     */
    virtual std::string detectType(const std::string &location)
        const = 0;

    const std::string type;
};

// inline stuff

template <typename DriverClass>
void Driver::registerDriver()
{
    registerDriver(std::make_shared<typename DriverClass::Factory>());
}

inline OStream::pointer Driver::output(File type)
{
    return output_impl(type);
}

inline IStream::pointer Driver::input(File type) const
{
    return input_impl(type);
}

inline OStream::pointer Driver::output(const TileId tileId, TileFile type)
{
    return output_impl(tileId, type);
}

inline IStream::pointer Driver::input(const TileId tileId, TileFile type) const
{
    return input_impl(tileId, type);
}

inline void Driver::remove(const TileId tileId, TileFile type)
{
    return remove_impl(tileId, type);
}

inline void Driver::begin()
{
    return begin_impl();
}

inline void Driver::commit()
{
    return commit_impl();
}

inline void Driver::rollback()
{
    return rollback_impl();
}

inline void Driver::flush()
{
    return flush_impl();
}

inline void Driver::drop()
{
    return drop_impl();
}

inline void Driver::update()
{
    return update_impl();
}

inline DriverProperties Driver::properties() const
{
    return properties_impl();
}

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_driver_hpp_included_

