/**
 * \file tilestorage/tilar.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile Archive handler.
 */

#ifndef vadstena_libs_tilestorage_tilar_hpp_included_
#define vadstena_libs_tilestorage_tilar_hpp_included_

#include <memory>
#include <cstdint>

#include <boost/filesystem/path.hpp>

#include "./types.hpp"
#include "../ids.hpp"
#include "../range.hpp"

namespace vadstena { namespace tilestorage {

/** Tilar interface.
 */
class Tilar {
public:
    enum class CreateMode {
        //!< truncates already existing file
        truncate
        //!< about to extend already existing file, options must match
        , append
        //!< fail if the file exists
        , failIfExists
    };

    enum class OpenMode { readOnly, readWrite };

    struct Options {
        unsigned int binaryOrder;
        unsigned int filesPerTile;

        bool operator==(const Options &o) const {
            return ((binaryOrder == o.binaryOrder)
                    && (filesPerTile == o.filesPerTile));
        }

        bool operator!=(const Options &o) const {
            return !operator==(o);
        }
    };

    static Tilar open(const boost::filesystem::path &path
                      , OpenMode openMode = OpenMode::readWrite);

    static Tilar create(const boost::filesystem::path &path
                        , const Options &options
                        , CreateMode createMode = CreateMode::append);

    // these 3 functions cannot be defined here due to undefined Detail struct
    Tilar(Tilar&&);
    Tilar& operator=(Tilar&&);
    ~Tilar();

    Tilar(const Tilar&) = delete;
    Tilar& operator=(const Tilar&) = delete;

private:
    struct Detail;

    Tilar(Detail *detail);

    std::unique_ptr<Detail> detail_;
    Detail& detail() { return *detail_; }
    const Detail& detail() const { return *detail_; }
};

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_tilar_hpp_included_
