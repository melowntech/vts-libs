/**
 * Copyright (c) 2019 Melown Technologies SE
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
#include <atomic>

#include "aggregated.hpp"
#include "runcallback.hpp"

namespace vtslibs { namespace vts { namespace driver {

namespace vs = vtslibs::storage;

namespace {

/** Special in-memory stream.
 */
struct NotFoundMarker : public IStream {
    NotFoundMarker() : IStream(File::config) {}
    virtual void close() {};
    virtual std::string name() const { return "Not Found"; }

    virtual std::istream& get() {
        LOGTHROW(err2, vs::IOError)
            << "Cannot get stream from not-found marker.";
        throw;
    }

    virtual FileStat stat_impl() const {
        LOGTHROW(err2, vs::IOError)
            << "Cannot get stat not-found marker.";
        throw;
    }
};

NotFoundMarker notFoundMarkerRaw;

const IStream::pointer notFoundMarker(&notFoundMarkerRaw, [](void*) {});

struct MetaBuilder : std::enable_shared_from_this<MetaBuilder> {
public:
    typedef std::shared_ptr<MetaBuilder> pointer;

    MetaBuilder(const fs::path &root, const TileId &tileId, int mbo
                , bool surfaceReferences, std::time_t lastModified
                , const InputCallback &cb, const IStream::pointer *notFound
                , const TileIndex &tileIndex)
        : root_(root), tileId_(tileId), mbo_(mbo)
        , surfaceReferences_(surfaceReferences)
        , lastModified_(lastModified), cb_(cb), callersNotFound_(notFound)

        , parentId_(parent(tileId_, mbo_))
        , shrinkedId_(tileId_.lod, parentId_.x, parentId_.y)
        , meta_(tileId_, mbo_)
        , expect_()
        , errorSink_(*this)
    {
        // fill in expected source references in the generated metatile
        if (const auto *tree = tileIndex.tree(tileId_.lod)) {
            tree->forEach
                (parentId_.lod, parentId_.x, parentId_.y
                 , [&](unsigned int x, unsigned int y, QTree::value_type value)
            {
                meta_.expectReference
                    (TileId(tileId_.lod, tileId_.x + x, tileId_.y + y)
                     , sourceReferenceFromFlags(value));
            }, QTree::Filter::white);
        }
    }

    void run(const AggregatedDriver::DriverEntry::list &drivers) {
        try {
            runImpl(drivers);
        } catch (...) {
            return runCallback(cb_);
        }
    }

private:
    void runImpl(const AggregatedDriver::DriverEntry::list &drivers) {
        const auto self(shared_from_this());

        // 1) collect data source information
        enum class SourceInfo { skip, async, sync };
        std::vector<SourceInfo> si;
        si.reserve(drivers.size());

        for (const auto &de : drivers) {
            if (!de.metaIndex.get(shrinkedId_)) {
                // no metatile -> skip
                si.push_back(SourceInfo::skip);
                continue;
            }
            ++expect_;

            // detect driver type
            si.push_back(de.driver->ccapabilities().async
                         ? SourceInfo::async
                         : SourceInfo::sync);
        }

        if (!expect_) {
            // no metatile to fetch -> not found
            return runCallback([&]() -> IStream::pointer
            {
                LOGTHROW(err1, vs::NoSuchFile)
                    << "There is no metatile for " << tileId_ << ".";
                return {};
            }, cb_);
        }

        // 2) asynchronously fetch from source preferring asynchronous fetch
        {
            auto isi(si.begin());
            int idx(0);
            for (const auto &de : drivers) {
                ++idx;
                if (*isi++ != SourceInfo::async) { continue; }

                de.driver->input(tileId_, TileFile::meta
                                 , [self, this, idx](const EIStream &eis)
                {
                    metaFetched(idx, eis);
                }, &notFoundMarker);
            }
        }

        // 3) synchronously fetch from rest of drivers
        {
            auto isi(si.begin());
            int idx(0);
            for (const auto &de : drivers) {
                ++idx;
                if (*isi++ != SourceInfo::sync) { continue; }

                IStream::pointer is;
                try {
                    is = de.driver->input
                        (tileId_, TileFile::meta, NullWhenNotFound);
                } catch (...) {
                    return error(std::current_exception());
                }

                if (!metaFetched(idx, is)) { return; }
            }
        }
    }

    bool metaFetched(int index, const IStream::pointer &is) {
        std::lock_guard<std::mutex> lock(mutex_);

        // canceled?
        if (!expect_) { return false; }
        --expect_;

        if (!is) { return handleNotFound(); }

        try {
            // try to update metatile with provided data
            meta_.update(index, loadMetaTile(*is, mbo_, is->name()));
        } catch (...) {
            // forward error and done
            runCallback(cb_);
            return false;
        }
        return done();
    }

    void metaFetched(int index, const EIStream &eis) {
        // finished asynchronously
        if (const auto &is = eis.get(errorSink_)) {
            metaFetched(index, (is == notFoundMarker)
                        ? IStream::pointer() : is);
        }
    }

    void error(const std::exception_ptr &exc) {
        // failed (a)synchronously: exception
        std::lock_guard<std::mutex> lock(mutex_);

        // already canceled?
        if (!expect_) { return; }

        // cancel all
        expect_ = 0;

        // forward error
        cb_(exc);
    }

    bool handleNotFound() {
        // TODO: check for node validity; all metatile nodes invalid ->
        // fine, empty metatile is ok; otherwise it should be an error
        return done();
    }

    bool done() {
        if (expect_) { return true; }

        // this was the last source metatile, OK
        if (meta_.empty()) {
            // empty -> not found
            runCallback([&]() -> IStream::pointer
            {
                LOGTHROW(err1, vs::NoSuchFile)
                    << "There is no metatile for " << tileId_ << ".";
                return {};
            }, cb_);
        } else {
            // erase surface references if configured to do so
            if (!surfaceReferences_) {
                meta_.for_each([&](const TileId&, MetaNode &node)
                {
                    node.sourceReference = 0;
                });
            }

            runCallback([this]() { return serialize(); }, cb_);
        }

        // done
        return false;
    }

    IStream::pointer serialize() const {
        // create in-memory stream
        auto fname(root_ / str(boost::format("%s.%s")
                                % tileId_ % TileFile::meta));
        auto s(std::make_shared<StringIStream>
               (TileFile::meta, fname.string(), lastModified_));

        // and serialize metatile
        meta_.save(s->sink());
        s->updateSize();

        // done
        return s;
    }

    const fs::path root_;
    const TileId tileId_;
    const int mbo_;
    const bool surfaceReferences_;
    const std::time_t lastModified_;
    const InputCallback cb_;
    const IStream::pointer *callersNotFound_;

    const TileId parentId_;
    const TileId shrinkedId_;
    MetaTile meta_;

    std::mutex mutex_;
    unsigned int expect_;

    struct ErrorSink {
        MetaBuilder &self;
        ErrorSink(MetaBuilder &self) : self(self) {}

        void operator()(const std::error_code &ec) {
            self.error(utility::makeErrorCodeException(ec));
        }

        void operator()(const std::exception_ptr &exc) {
            self.error(exc);
        }
    } errorSink_;
};

} // namespace

void AggregatedDriver::buildMeta(const TileId &tileId, std::time_t lastModified
                                 , const InputCallback &cb
                                 , const IStream::pointer *notFound) const
{
    try {
        auto mb(std::make_shared<MetaBuilder>
                (root(), tileId, referenceFrame_.metaBinaryOrder
                 , surfaceReferences_, lastModified, cb, notFound
                 , tsi_.tileIndex));

        mb->run(drivers_);
    } catch (...) {
        // forward exception
        runCallback(cb);
    }
}

} } } // namespace vtslibs::vts::driver
