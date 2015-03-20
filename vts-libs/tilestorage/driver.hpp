#ifndef vadstena_libs_tilestorage_driver_hpp_included_
#define vadstena_libs_tilestorage_driver_hpp_included_

#include <map>

#include "utility/runnable.hpp"

#include "./basetypes.hpp"
#include "./properties.hpp"
#include "./streams.hpp"

namespace vadstena { namespace tilestorage {

class Driver : boost::noncopyable {
public:
    typedef std::shared_ptr<Driver> pointer;

    struct CreateProperties {
        StaticProperties properties;
        bool cloned;

        CreateProperties(const StaticProperties &properties
                         , bool cloned = false)
            : properties(properties), cloned(cloned) {}

        StaticProperties* operator->() { return &properties; }
        const StaticProperties* operator->() const { return &properties; }
    };

    struct Resources {
        std::size_t openFiles;
        std::size_t memory;

        Resources(std::size_t openFiles = 0, std::size_t memory = 0)
            : openFiles(openFiles), memory(memory)
        {}

        Resources& operator+=(Resources &o);
        Resources& operator-=(Resources &o);
        bool operator<(Resources &o) const;
    };

    virtual ~Driver() {};

    OStream::pointer output(File type);

    IStream::pointer input(File type) const;

    OStream::pointer output(const TileId tileId, TileFile type);

    IStream::pointer input(const TileId tileId, TileFile type) const;

    void remove(const TileId tileId, TileFile type);

    FileStat stat(File type) const;

    FileStat stat(const TileId tileId, TileFile type) const;

    Resources resources() const;

    bool readOnly() const { return readOnly_; }

    void wannaWrite(const std::string &what) const;

    void begin(utility::Runnable *runnable = nullptr);

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
                                  , const CreateProperties &properties);

    static Driver::pointer open(Locator locator, OpenMode mode);

    static std::map<std::string, std::string> listSupportedDrivers();

protected:
    Driver(bool readOnly) : readOnly_(readOnly), runnable_() {}

private:
    virtual OStream::pointer output_impl(const File type) = 0;

    virtual IStream::pointer input_impl(File type) const = 0;

    virtual OStream::pointer
    output_impl(const TileId tileId, TileFile type) = 0;

    virtual IStream::pointer
    input_impl(const TileId tileId, TileFile type) const = 0;

    virtual void remove_impl(const TileId tileId, TileFile type) = 0;

    virtual FileStat stat_impl(File type) const = 0;

    virtual FileStat stat_impl(const TileId tileId, TileFile type)
        const = 0;

    virtual Resources resources_impl() const { return {}; }

    virtual void begin_impl() = 0;

    virtual void commit_impl() = 0;

    virtual void rollback_impl() = 0;

    virtual void flush_impl() {};

    virtual void drop_impl() = 0;

    virtual void update_impl() = 0;

    virtual DriverProperties properties_impl() const = 0;

    static void registerDriver(const std::shared_ptr<Factory> &factory);

    static std::string detectType(const std::string &location);

    virtual void postOpenCheck() {}

    void checkRunning() const;

    void notRunning() const;

    bool readOnly_;

    /** Runnable associated with the transaction.
     */
    utility::Runnable *runnable_;
};

class Driver::Factory {
public:
    typedef std::shared_ptr<Factory> pointer;
    Factory(const std::string &type) : type(type) {}

    virtual ~Factory() {}

    virtual Driver::pointer
    create(const std::string location, CreateMode mode
           , const Driver::CreateProperties &properties)
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
    checkRunning();
    return output_impl(type);
}

inline IStream::pointer Driver::input(File type) const
{
    checkRunning();
    return input_impl(type);
}

inline OStream::pointer Driver::output(const TileId tileId, TileFile type)
{
    checkRunning();
    return output_impl(tileId, type);
}

inline IStream::pointer Driver::input(const TileId tileId, TileFile type) const
{
    checkRunning();
    return input_impl(tileId, type);
}

inline void Driver::remove(const TileId tileId, TileFile type)
{
    checkRunning();
    return remove_impl(tileId, type);
}

inline FileStat Driver::stat(File type) const
{
    auto stat(stat_impl(type));
    stat.contentType = contentType(type);
    return stat;
}

inline FileStat Driver::stat(const TileId tileId, TileFile type) const
{
    auto stat(stat_impl(tileId, type));
    stat.contentType = contentType(type);
    return stat;
}

inline Driver::Resources Driver::resources() const
{
    return resources_impl();
}

inline void Driver::begin(utility::Runnable *runnable)
{
    begin_impl();
    runnable_ = runnable;
}

inline void Driver::commit()
{
    commit_impl();
    runnable_ = nullptr;
}

inline void Driver::rollback()
{
    rollback_impl();
    runnable_ = nullptr;
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

inline void Driver::checkRunning() const
{
    if (!runnable_ || *runnable_) { return; }
    notRunning();
}

inline Driver::Resources& Driver::Resources::operator+=(Resources &o)
{
    openFiles += o.openFiles;
    memory += o.memory;
    return *this;
}

inline Driver::Resources& Driver::Resources::operator-=(Resources &o)
{
    openFiles -= o.openFiles;
    memory -= o.memory;
    return *this;
}

inline bool Driver::Resources::operator<(Resources &o) const
{
    if (openFiles < o.openFiles) {
        return true;
    } else if (o.openFiles < openFiles) {
        return false;
    }
    return memory < o.memory;
}

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os
           , const Driver::Resources &r)
{
    return os << "{openFiles=" << r.openFiles
              << ", memory=" << r.memory << '}';
}

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_driver_hpp_included_
