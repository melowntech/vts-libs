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
#include <mutex>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/crc.hpp>

#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/identity.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/mem_fun.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/time.hpp"

#include "../../../storage/openfiles.hpp"
#include "../../io.hpp"
#include "cache.hpp"

namespace vtslibs { namespace vts { namespace driver {

namespace {

Tilar::ContentTypes tileContentTypes({ "", "image/jpeg" });
Tilar::ContentTypes metatileContentTypes;
Tilar::ContentTypes navtileContentTypes({ "image/jpeg" });

std::uint32_t calculateHash(const std::string &data)
{
    boost::crc_32_type crc;
    crc.process_bytes(data.data(), data.size());
    return crc.checksum();
}

fs::path dir(const fs::path &filename)
{
    const auto hash(calculateHash(filename.string()));
    return str(boost::format("%02x") % ((hash >> 24) & 0xff));
}

int fileType(TileFile type) {
    switch (type) {
    case TileFile::meta: return 0;
    case TileFile::mesh: return 0;
    case TileFile::atlas: return 1;
    case TileFile::navtile: return 0;
    default: break;
    }

    utility::raise<storage::NoSuchTile>("File type <%s> unsupported by "
                                        "this driver", type);
    throw;
}

typedef decltype(utility::usecFromEpoch()) Time;
const Time MaxTime(std::numeric_limits<Time>::max());

} // namespace

struct Cache::Archives
{
    struct Record {
        Record(TileId index, Tilar &&tilar)
            : index(index), lastHit(utility::usecFromEpoch())
            , tilar_(std::move(tilar))
        {}

        Tilar& tilar() const { return tilar_; }

        void hit() { lastHit = utility::usecFromEpoch(); }

        void infinity() { lastHit = MaxTime; }

        bool isInInfinity() const { return lastHit == MaxTime; }

        TileId index;
        Time lastHit;

    private:
        mutable Tilar tilar_;
    };

    Archives(const fs::path &root, const std::string &extension
             , bool readOnly, int filesPerTile
             , const PlainOptions &options
             , const Tilar::ContentTypes &contentTypes);

    Tilar open(const TileId &archive, bool noSuchFile = true);

    fs::path filePath(const TileId &index) const;

    void flush() {
        finish([](Tilar &tilar) { tilar.commit(); });
    }

    std::size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return map_.size();
    }

private:
    void houseKeeping(const TileId *keep = nullptr);

    template <typename Idx, typename Iterator>
    inline void hit(Idx &idx, Iterator iterator) {
        idx.modify(iterator, [](Record &r) { r.hit(); });
    }

    template <typename Idx, typename Iterator>
    inline void infinity(Idx &idx, Iterator iterator) {
        idx.modify(iterator, [](Record &r) { r.infinity(); });
    }

    template <typename Op>
    void finish(Op op) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto iidx(map_.begin()); iidx != map_.end(); ) {
            op(iidx->tilar());
            iidx = map_.erase(iidx);
        }
    }

    struct IndexIdx {};
    struct LastHitIdx {};

    typedef boost::multi_index_container<
        Record
        , boost::multi_index::indexed_by<
              boost::multi_index::ordered_unique
              <boost::multi_index::tag<IndexIdx>
               , BOOST_MULTI_INDEX_MEMBER
               (Record, decltype(Record::index), index)>
              , boost::multi_index::ordered_non_unique
              <boost::multi_index::tag<LastHitIdx>
               , BOOST_MULTI_INDEX_MEMBER
               (Record, decltype(Record::lastHit), lastHit)>
              >
        > Map;

    const fs::path root_;
    const std::string extension_;
    const Tilar::Options options_;
    const bool readOnly_;
    Map map_;
    const Tilar::ContentTypes &contentTypes_;

    mutable std::mutex mutex_;
};

Cache::Archives::Archives(const fs::path &root, const std::string &extension
                          , bool readOnly, int filesPerTile
                          , const PlainOptions &options
                          , const Tilar::ContentTypes &contentTypes)
    : root_(root), extension_(extension)
    , options_(options.tilar(filesPerTile))
    , readOnly_(readOnly), contentTypes_(contentTypes)
{}

fs::path Cache::Archives::filePath(const TileId &index) const
{
    const auto filename(str(boost::format("%s-%07d-%07d.%s")
                            % index.lod % index.x % index.y
                            % extension_));
    const auto parent(root_ / dir(filename));
    if (!readOnly_) {
        // ensure dir exists
        create_directories(parent);
    }
    return parent / filename;
}

Cache::Cache(const fs::path &root, const PlainOptions &options
             , bool readOnly)
    : root_(root), options_(options), readOnly_(readOnly)
    , tiles_(new Archives(root, "tiles", readOnly, 2, options
                          , tileContentTypes))
    , metatiles_(new Archives(root, "metatiles", readOnly, 1, options
                              , metatileContentTypes))
    , navtiles_(new Archives(root, "navtiles", readOnly, 1, options
                             , navtileContentTypes))
{}

namespace {

Tilar tilar(const fs::path &path, const Tilar::Options &options
            , bool readOnly, bool noSuchFile = true)
{
    if (readOnly) {
        // read-only
        if (noSuchFile) {
            return Tilar::open(path, options
                               , Tilar::OpenMode::readOnly);
        } else {
            return Tilar::open(path, options
                               , NullWhenNotFound
                               , Tilar::OpenMode::readOnly);
        }
    }
    return Tilar::create(path, options
                         , Tilar::CreateMode::appendOrTruncate);
}

} // namespace

Cache::~Cache() = default;

void Cache::Archives::houseKeeping(const TileId *keep)
{
    if (!storage::OpenFiles::critical()) { return; }

    auto &idx(map_.get<LastHitIdx>());

    LOG(info1)
        << "Critical number of open files reached (current count is "
        << storage::OpenFiles::count() << ") trying to drop some of "
        << idx.size() << " files owned by this driver.";

    int toDrop(5);
    for (auto iidx(idx.begin()); toDrop && (iidx != idx.end()); ) {
        // skip keep file
        if (keep && (iidx->index == *keep)) {
            LOG(debug) << "File " << iidx->tilar().path()
                       << " must be kept in the cache.";
            ++iidx;
            continue;
        }

        // files in infinity -> this and all its friends are detached
        if (iidx->isInInfinity()) { return; }

        auto &file(iidx->tilar());

        switch (file.state()) {
        case Tilar::State::detached:
            // NB: this should be never seen
            LOG(debug)
                << "File " << file.path() << '/' << iidx->lastHit
                << " is already detached. In fact, this log line "
                << "should not be seen in the log at all.";
            return;

        case Tilar::State::detaching:
            // NB: this should be never seen
            LOG(debug)
                << "File " << file.path() << '/' << iidx->lastHit
                << " is being detached. In fact, this log line "
                << "should not be seen in the log at all.";
            return;

        case Tilar::State::pristine:
            // no changes -> we are free to remove the file
            LOG(debug) << "Removing pristine tilar file "
                       << file.path() << '/' << iidx->lastHit
                       << " from the cache.";

            iidx = idx.erase(iidx);
            --toDrop;
            break;

        case Tilar::State::changed:
            // file is changed -> ask to detach
            LOG(debug)
                << "Asking to detach changed file "
                << file.path() << '/' << iidx->lastHit << ".";

            // ask for file detachment
            file.detach();

            // hit -> move to the future, next time this is function is called
            // we will not encounter this detached file again
            {
                auto old(iidx);
                ++iidx;
                infinity(idx, old);
            }

            // mark one more dropped file
            --toDrop;
            break;
        }
    }
}

Tilar Cache::Archives::open(const TileId &archive, bool noSuchFile)
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto fmap(map_.find(archive));
    if (fmap != map_.end()) {
        hit(map_, fmap);
        // housekeeping, we want to keep found file
        houseKeeping(&fmap->index);
        return fmap->tilar();
    }

    // housekeeping before open
    houseKeeping();

    const auto path(filePath(archive));
    auto file(tilar(path, options_, readOnly_, noSuchFile));
    if (!file) { return file; }
    file.setContentTypes(contentTypes_);

    return map_.insert
        (Record(archive, std::move(file))).first->tilar();
}

IStream::pointer Cache::input(const TileId tileId, TileFile type)
{
    const auto index(options_.index(tileId, type, fileType(type)));
    return getArchives(type).open(index.archive).input(index.file);
}

IStream::pointer Cache::input(const TileId tileId, TileFile type
                              , const NullWhenNotFound_t&)
{
    const auto index(options_.index(tileId, type, fileType(type)));
    auto file(getArchives(type).open(index.archive, false));
    if (!file) { return {}; }
    return file.input(index.file, NullWhenNotFound);
}

OStream::pointer Cache::output(const TileId tileId, TileFile type)
{
    const auto index(options_.index(tileId, type, fileType(type)));
    return getArchives(type).open(index.archive).output(index.file);
}

std::size_t Cache::size(const TileId tileId, TileFile type)
{
    const auto index(options_.index(tileId, type, fileType(type)));
    return getArchives(type).open(index.archive).size(index.file);
}

FileStat Cache::stat(const TileId tileId, TileFile type)
{
    const auto index(options_.index(tileId, type, fileType(type)));
    return getArchives(type).open(index.archive).stat(index.file);
}

storage::Resources Cache::resources()
{
    return { (tiles_->size()
              + metatiles_->size()
              + navtiles_->size())
            , 0 };
}

void Cache::flush()
{
    if (readOnly_) { return; }
    tiles_->flush();
    metatiles_->flush();
    navtiles_->flush();
}

} } } // namespace vtslibs::vts::driver
