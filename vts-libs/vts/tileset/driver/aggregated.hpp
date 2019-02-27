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
#ifndef vtslibs_vts_tileset_driver_aggregated_hpp_included_
#define vtslibs_vts_tileset_driver_aggregated_hpp_included_

#include <set>
#include <map>
#include <memory>

#include "../driver.hpp"
#include "../../storage.hpp"
#include "../../tsmap.hpp"
#include "cache.hpp"

namespace vtslibs { namespace vts { namespace driver {

/** Helper class.
 */
struct AggregatedDriverBase {
    AggregatedDriverBase() {}
    AggregatedDriverBase(const CloneOptions &cloneOptions);
};

class AggregatedDriver : private AggregatedDriverBase, public Driver {
private:
    struct PrivateTag {};

public:
    typedef std::shared_ptr<AggregatedDriver> pointer;

    /** Creates new storage. Existing storage is overwritten only if mode ==
     *  CreateMode::overwrite.
     */
    AggregatedDriver(const boost::filesystem::path &root
                     , const AggregatedOptions &options
                     , const CloneOptions &cloneOptions);

    /** Creates in-memory storage.
     */
    AggregatedDriver(const AggregatedOptions &options
                     , const CloneOptions &cloneOptions);

    /** Opens storage.
     */
    AggregatedDriver(const boost::filesystem::path &root
                     , const OpenOptions &openOptions
                     , const AggregatedOptions &options);

    virtual ~AggregatedDriver();

    typedef std::uint16_t TilesetReference;

    typedef std::vector<TilesetReference> TilesetReferences;

    typedef std::vector<TilesetReferences> TilesetReferencesList;

    struct DriverEntry {
        /** Driver.
         */
        Driver::pointer driver;

        /** Derived metatile index.
         */
        TilesetReferences tilesets;

        /** Derived metatile index.
         */
        TileIndex metaIndex;

        typedef std::vector<DriverEntry> list;

        DriverEntry(const Driver::pointer &driver
                    , const TilesetReferences &tilesets)
            : driver(driver), tilesets(tilesets)
        {}

        DriverEntry(const TilesetReferences &tilesets)
            : tilesets(tilesets)
        {}
    };

    class Index : public tileset::Index {
    public:
        Index(unsigned int metaBinaryOrder, DriverEntry::list &drivers
              , const AggregatedOptions *options = nullptr)
            : tileset::Index(metaBinaryOrder), drivers_(drivers)
            , staticMetaRange_(options
                               ? options->staticMetaRange
                               : LodRange::emptyRange())
        {}

        virtual void loadRest_impl(std::istream &f
                                   , const boost::filesystem::path &path);

        virtual void saveRest_impl(std::ostream &f) const;

        bool staticMeta(const TileId &tileId) const {
            return vts::in(staticMetaRange_, tileId);
        }

    private:
        DriverEntry::list &drivers_;
        LodRange staticMetaRange_;
    };

    /** Reencodes aggregated tileset.
     */
    static bool reencode(const boost::filesystem::path &root
                         , const AggregatedOptions &driverOptions
                         , const ReencodeOptions &options
                         , const std::string &prefix = "");

    /** Async open.
     */
    static void open(const boost::filesystem::path &root
                     , const OpenOptions &openOptions
                     , const AggregatedOptions &options
                     , const DriverOpenCallback::pointer &callback);

    /** Cloner
     */
    AggregatedDriver(PrivateTag, const boost::filesystem::path &root
                     , AggregatedOptions options
                     , const CloneOptions &cloneOptions
                     , const AggregatedDriver &src);

    /** Opened dependencies used in final async open ctor.
     */
    struct Dependencies {
        DriverEntry::list drivers;
        boost::optional<Storage> storage;
    };

    /** Final async open ctor.
     */
    AggregatedDriver(PrivateTag, const boost::filesystem::path &root
                     , const OpenOptions &openOptions
                     , const AggregatedOptions &options
                     , const Dependencies &dependencies);

private:

    virtual OStream::pointer output_impl(const File type);

    virtual IStream::pointer input_impl(File type) const {
        return input_impl(type, true);
    }

    virtual IStream::pointer input_impl(File type, const NullWhenNotFound_t&)
        const
    {
        return input_impl(type, false);
    }

    virtual OStream::pointer
    output_impl(const TileId &tileId, TileFile type);

    virtual IStream::pointer
    input_impl(const TileId &tileId, TileFile type) const {
        return input_impl(tileId, type, true);
    }

    virtual IStream::pointer
    input_impl(const TileId &tileId, TileFile type, const NullWhenNotFound_t&)
        const
    {
        return input_impl(tileId, type, false);
    }

    IStream::pointer input_impl(File type, bool noSuchFile) const;

    IStream::pointer input_impl(const TileId &tileId, TileFile type
                                , bool noSuchFile) const;

    virtual void input_impl(const TileId &tileId, TileFile type
                            , const InputCallback &cb
                            , const IStream::pointer *notFound) const;

    virtual void drop_impl();

    virtual void flush_impl();

    virtual FileStat stat_impl(File type) const;

    virtual FileStat stat_impl(const TileId &tileId, TileFile type) const;

    virtual Resources resources_impl() const;

    Driver::pointer clone_impl(const boost::filesystem::path &root
                               , const CloneOptions &cloneOptions) const;

    virtual std::string info_impl() const;

    IStream::pointer input_mem(File type) const;

    virtual tileset::Index* getTileIndex_impl() { return &tsi_; }

    virtual const tileset::Index* getTileIndex_impl() const { return &tsi_; }

    inline const AggregatedOptions& options() const {
        return Driver::options<const AggregatedOptions&>();
    }

    TileSet::Properties build(AggregatedOptions options
                              , const CloneOptions &cloneOptions
                              , bool onDisk = false);

    void generateMetatiles(AggregatedOptions &options);

    void copyMetatiles(AggregatedOptions &options, Cache *srcCache);

    inline IStream::pointer input_impl(const std::string &name) const {
        return input_impl(name, true);
    }

    inline IStream::pointer input_impl(const std::string &name
                                       , const NullWhenNotFound_t&) const
    {
        return input_impl(name, false);
    }

    IStream::pointer input_impl(const std::string &name
                                , bool noSuchFile) const;

    FileStat stat_impl(const std::string &name) const;

    /** Asynchronously builds metatile stream.
     */
    void buildMeta(const TileId &tileId, std::time_t lastModified
                   , const InputCallback &cb, const IStream::pointer *notFound)
        const;

    Storage storage_;

    registry::ReferenceFrame referenceFrame_;

    DriverEntry::list drivers_;

    Index tsi_;

    boost::optional<TileSet::Properties> memProperties_;

    bool surfaceReferences_;

    /** Metatile cache, valid only when there are any pre-generated tiles.
     */
    mutable std::unique_ptr<Cache> cache_;
};

inline MetaNode::SourceReference
sourceReferenceFromFlags(TileIndex::Flag::value_type flags)
{
    return flags >> 16;
}

} } } // namespace vtslibs::vts::driver

#endif // vtslibs_vts_tileset_driver_aggregated_hpp_included_
