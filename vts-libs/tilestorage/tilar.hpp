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
#include <ostream>

#include <boost/filesystem/path.hpp>
#include <boost/iostreams/categories.hpp>

#include "../ids.hpp"
#include "../range.hpp"

#include "./types.hpp"
#include "./streams.hpp"

namespace vadstena { namespace tilestorage {

/** Tilar interface.
 */
class Tilar {
public:
    enum class CreateMode {
        //!< truncates already existing file
        truncate
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

    /** Opens existing tilar files.
     *
     *  \param path to the tilar file
     *  \param openMode open mode (r/o, r/w)
     *  \return open tilar file
     */
    static Tilar open(const boost::filesystem::path &path
                      , OpenMode openMode = OpenMode::readWrite);

    /** Creates a new tilar file.
     *
     *  \param path to the tilar file
     *  \param options options to set
     *  \param createMode creation mode (fail if file exists, truncate original)
     *  \return freshly created tilar file (r/w mode)
     */
    static Tilar create(const boost::filesystem::path &path
                        , const Options &options
                        , CreateMode createMode = CreateMode::failIfExists);

    /** Special version of open: read-only access older file revision.
     *
     *  \param path to the tilar file
     *  \param indexOffset index offset inside the file
     *  \return open tilar file
     */
    static Tilar open(const boost::filesystem::path &path
                      , std::uint32_t indexOffset);

    // these 3 functions cannot be defined here due to undefined Detail struct
    Tilar(Tilar&&);
    Tilar& operator=(Tilar&&);
    ~Tilar();

    Tilar(const Tilar&) = delete;
    Tilar& operator=(const Tilar&) = delete;

    /** Index of a file in the tilar archive (3-dimensional index).
     */
    struct FileIndex {
        unsigned int col;
        unsigned int row;
        unsigned int type;

        FileIndex(unsigned int col = 0, unsigned int row = 0
                  , unsigned int type = 0)
            : col(col), row(row), type(type) {}
    };

    /** File entry in the tilar archive. Used only in listing.
     */
    struct Entry {
        FileIndex index;
        std::uint32_t start;
        std::uint32_t size;

        Entry(const FileIndex &index, std::uint32_t start
              , std::uint32_t size)
            : index(index), start(start), size(size) {}

        typedef std::vector<Entry> list;
    };

    /** Archive information.
     */
    struct Info {
        /** Position of previous index in the file.
         */
        std::uint32_t offset;

        /** Position of previous index in the file.
         */
        std::uint32_t previousOffset;

        /** Number of bytes wasted in this file.
         */
        std::uint32_t overhead;

        /** Timestamp when this index has been saved.
         */
        std::time_t modified;
    };

    /** Flushes file to the disk (writes new index if needed).
     */
    void flush();

    // operations

    const Options& options() const;

    void expect(const Options &options);

    /** Get output stream to write content of new file at given index.
     */
    OStream::pointer output(const FileIndex &index);

    /** Get input stream to read content of file at given index.
     *  Throws if file doesn't exist.
     */
    IStream::pointer input(const FileIndex &index);

    /** Removes file at given index.
     */
    void remove(const FileIndex &index);

    /** List all existing files.
     */
    Entry::list list() const;

    Info info() const;

private:
    struct Detail;

    Tilar(Detail *detail);

    std::unique_ptr<Detail> detail_;
    Detail& detail() { return *detail_; }
    const Detail& detail() const { return *detail_; }

    class Device; friend class Device;
    class Source; friend class Source;
    class Sink; friend class Sink;
};

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_tilar_hpp_included_
