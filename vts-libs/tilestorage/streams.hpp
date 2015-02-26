#ifndef vadstena_libs_tilestorage_streams_hpp_included_
#define vadstena_libs_tilestorage_streams_hpp_included_

#include <iostream>
#include <memory>

namespace vadstena { namespace tilestorage {

enum class TileFile { meta, mesh, atlas };

enum class File { config, tileIndex };

class OStream {
public:
    OStream() {}
    virtual ~OStream() {}
    virtual std::ostream& get() = 0;
    virtual void close() = 0;
    virtual std::string name() = 0;

    operator std::ostream&() { return get(); }
    typedef std::shared_ptr<OStream> pointer;
};

class IStream {
public:
    IStream() {}
    virtual ~IStream() {}
    virtual std::istream& get() = 0;
    virtual void close() = 0;
    virtual std::string name() = 0;

    operator std::istream&() { return get(); }
    typedef std::shared_ptr<IStream> pointer;
};

inline void copyFile(const IStream::pointer &in
                     , const OStream::pointer &out)
{
    out->get() << in->get().rdbuf();
    in->close();
    out->close();
}

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_streams_hpp_included_

