/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \file storage/tilar.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile Archive handler.
 */

#ifndef vtslibs_storage_tilar_hpp_included_
#define vtslibs_storage_tilar_hpp_included_

#include <memory>
#include <cstdint>
#include <ostream>

#include <boost/filesystem/path.hpp>
#include <boost/uuid/uuid.hpp>

#include "streams.hpp"

namespace vtslibs { namespace storage {

/** Tilar interface.
 */
class Tilar {
public:
    enum class CreateMode {
        //!< truncates already existing file
        truncate
        //!< fail if the file exists
        , failIfExists
        //!< about to extend already existing file, options must match
        , append
        //!< append file if options exist, truncate otherwise
        , appendOrTruncate
    };

    enum class OpenMode { readOnly, readWrite };

    struct Options {
        /** square edge = 2^binaryOrder
         */
        unsigned int binaryOrder;

        /** Number of files per tile.
         */
        unsigned int filesPerTile;

        /** UUID of file. In case of a group of file (i.e. used as backing store
         *  of a tileset use UUID of tileset.
         */
        boost::uuids::uuid uuid;

        bool operator==(const Options &o) const;

        bool operator!=(const Options &o) const {
            return !operator==(o);
        }

        Options(unsigned int binaryOrder = 0, unsigned int filesPerTile = 0);

        Options(unsigned int binaryOrder, unsigned int filesPerTile
                , const boost::uuids::uuid &uuid)
            : binaryOrder(binaryOrder), filesPerTile(filesPerTile), uuid(uuid)
        {}
    };

    typedef std::vector<std::string> ContentTypes;

    /** Opens existing tilar files.
     *
     *  \param path to the tilar file
     *  \param openMode open mode (r/o, r/w)
     *  \return open tilar file
     */
    static Tilar open(const boost::filesystem::path &path
                      , OpenMode openMode = OpenMode::readWrite);

    /** Opens existing tilar files.
     *
     *  \param path to the tilar file
     *  \param openMode open mode (r/o, r/w)
     *  \return open tilar file
     */
    static Tilar open(const boost::filesystem::path &path
                      , const NullWhenNotFound_t&
                      , OpenMode openMode = OpenMode::readWrite);

    /** Opens existing tilar files and checks options before returning.
     *
     *  \param path to the tilar file
     *  \param options expected options
     *  \param openMode open mode (r/o, r/w)
     *  \return open tilar file
     */
    static Tilar open(const boost::filesystem::path &path
                      , const Options &options
                      , const NullWhenNotFound_t&
                      , OpenMode openMode = OpenMode::readWrite);

    /** Opens existing tilar files and checks options before returning.
     *
     *  \param path to the tilar file
     *  \param options expected options
     *  \param openMode open mode (r/o, r/w)
     *  \return open tilar file
     */
    static Tilar open(const boost::filesystem::path &path
                      , const Options &options
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

    ~Tilar();

    /** Associates file type with content type.
     *  Must have same number of arguments as options.filesPerTile.
     */
    Tilar& setContentTypes(const ContentTypes &mapping);

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
    void commit();

    /** Rolls back all changes since last commit.
     */
    void rollback();

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

    /** Get input stream to read content of file at given index.
     *  Returns invalid pointer if file doesn't exist.
     */
    IStream::pointer input(const FileIndex &index, const NullWhenNotFound_t&);

    /** Get size of stored file.
     *  Throws if file doesn't exist.
     */
    std::size_t size(const FileIndex &index);

    /** Get file's statistics (size and time).
     *  Throws if file doesn't exist.
     */
    FileStat stat(const FileIndex &index);

    /** Removes file at given index.
     */
    void remove(const FileIndex &index);

    /** List all existing files.
     */
    Entry::list list() const;

    Info info() const;

    /** Returns true if low-level I/O ignores interrupts.
     */
    bool ignoreInterrupts() const;

    /** Sets interrupt ignoring flag.
     */
    void ignoreInterrupts(bool value);

    /** Detaches open archive from the file (i.e. closes file).
     *
     *  Once file access is needed archive attaches itself to the file again.
     */
    void detach();

    /** Archive status.
     */
    enum class State { pristine, changed, detaching, detached };

    /** Returns curren archive accessor state.
     */
    State state() const;

    /** Returns path to archive file.
     */
    boost::filesystem::path path() const;

    operator bool() const { return detail_.get(); }

private:
    struct Detail;

    Tilar(const std::shared_ptr<Detail> &detail);

    std::shared_ptr<Detail> detail_;
    inline Detail& detail() { return *detail_; }
    inline const Detail& detail() const { return *detail_; }

    class Device; friend class Device;
    class Source; friend class Source;
    class Sink; friend class Sink;
};

inline Tilar Tilar::open(const boost::filesystem::path &path
                         , const Options &options, OpenMode openMode)
{
    auto tilar(open(path, openMode));
    tilar.expect(options);
    return tilar;
}

inline Tilar Tilar::open(const boost::filesystem::path &path
                         , const Options &options
                         , const NullWhenNotFound_t&
                         , OpenMode openMode)
{
    auto tilar(open(path, NullWhenNotFound, openMode));
    if (tilar) { tilar.expect(options); }
    return tilar;
}

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const Tilar::Options &o)
{
    os << "{binaryOrder=" << o.binaryOrder
       << ", filesPerTile=" << o.filesPerTile
       << ", uuid=" << o.uuid << "}";
    return os;
}

} } // namespace vtslibs::storage

#endif // vtslibs_storage_tilar_hpp_included_
