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
#include <atomic>

#include <boost/format.hpp>
#include <boost/io/ios_state.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/openmp.hpp"

#include "../vts.hpp"

#include "tileop.hpp"
#include "io.hpp"
#include "encoder.hpp"
#include "csconvertor.hpp"
#include "../storage/error.hpp"

namespace vtslibs { namespace vts {

namespace {

struct ConstraintsFlag {
    typedef std::uint32_t type;
    enum : type {
        useLodRange = 0x01
        , useExtents = 0x02
        , clearExtentsOnHit = 0x04

        , all = (useLodRange | useExtents)
    };

    static type build(const Encoder::Constraints &c) {
        type flags(0);

        if (c.lodRange) { flags |= useLodRange; }
        if (c.extents || c.extentsGenerator) { flags |= useExtents; }
        if (c.useExtentsForFirstHit) { flags |= clearExtentsOnHit; }

        return flags;
    }

    static void clearExtents(type &flags, bool value) {
        if (value && (flags & clearExtentsOnHit)) {
            flags &= ~useExtents;
        }
    }
};

struct TileFlags {
    const Encoder::TileResult &tr;
};

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const TileFlags &tf)
{
    const char *sep("");
    if (tf.tr.hasMesh()) { os << "mesh"; sep = ","; }
    if (tf.tr.hasAtlas()) { os << sep << "atlas"; sep = ","; }
    if (tf.tr.hasNavtile()) { os << sep << "navtile"; }
    return os;
}

} // namespace

void Encoder::TileResult::fail(const char *what) const
{
    LOGTHROW(err1, std::runtime_error)
        << "Encoder: " << what << ".";
}

struct Encoder::Detail {
    Detail(Encoder *owner, const boost::filesystem::path &path
           , const TileSetProperties &properties, CreateMode mode
           , const Options &options)
        : owner(owner), options(options)
        , ownTs(createTileSet(path, properties, mode))
        , tileSet(*ownTs)
        , properties(tileSet.getProperties())
        , referenceFrame(tileSet.referenceFrame())
        , physicalSrs(registry::system.srs
                      (referenceFrame.model.physicalSrs))
        , navigationSrs(registry::system.srs
                      (referenceFrame.model.navigationSrs))
        , generated_(0), estimated_(0)
    {}

    Detail(Encoder *owner, TileSet &tileset, const Options &options)
        : owner(owner), options(options)
        , tileSet(tileset)
        , properties(tileSet.getProperties())
        , referenceFrame(tileSet.referenceFrame())
        , physicalSrs(registry::system.srs
                      (referenceFrame.model.physicalSrs))
        , navigationSrs(registry::system.srs
                      (referenceFrame.model.navigationSrs))
        , generated_(0)
    {}

    TileSet run()
    {
        UTILITY_OMP(parallel)
        UTILITY_OMP(single)
        {
            owner->threadCount(omp_get_num_threads());
            process({}, ConstraintsFlag::build(constraints)
                    , NodeInfo(referenceFrame), {});
        }
        LOG(info3) << "VTS Encoder: generated. Finishing.";

        // let the caller finish the tileset
        owner->finish(tileSet);

        // flush result
        if (options.flush()) {
            LOG(info3) << "VTS Encoder: Flushing.";
            tileSet.flush();
        }

        LOG(info3) << "VTS Encoder: done.";

        // done
        return tileSet;
    }

    void setConstraints(const Constraints &constraints);

    const math::Extents2& getExtents(const std::string &srs) const;

    typedef registry::ReferenceFrame::Division::Node Node;

    void process(const TileId &tileId, ConstraintsFlag::type useConstraints
                 , const NodeInfo &nodeInfo, const TileResult &parentTile);

    Encoder *owner;
    const Options options;
    boost::optional<TileSet> ownTs;
    TileSet &tileSet;

    TileSetProperties properties;
    registry::ReferenceFrame referenceFrame;
    const registry::Srs &physicalSrs;
    const registry::Srs &navigationSrs;

    Constraints constraints;

    typedef std::map<std::string, math::Extents2> SrsExtentsMap;
    SrsExtentsMap srsExtents;

    std::atomic<std::size_t> generated_;
    std::atomic<std::size_t> estimated_;
};

void Encoder::Detail::setConstraints(const Constraints &c)
{
    constraints = c;

    for (const auto &tileId : c.invalidNodes) {
        referenceFrame.invalidate(rfNodeId(tileId));
    }

    if (constraints.extents) {
        for (const auto &srs : referenceFrame.division.srsList()) {
            srsExtents[srs] = CsConvertor(constraints.extents->srs, srs)
                (constraints.extents->extents);
        }
    } else if (constraints.extentsGenerator) {
        for (const auto &srs : referenceFrame.division.srsList()) {
            srsExtents[srs] = constraints.extentsGenerator(srs);
        }
    }
}

inline const math::Extents2&
Encoder::Detail::getExtents(const std::string &srs) const
{
    auto fsrsExtents(srsExtents.find(srs));
    if (fsrsExtents == srsExtents.end()) {
        LOGTHROW(err2, storage::Error)
            << "Inconsistency: no extents constraints for srs "
            << " <" << srs << ">.";
    }
    return fsrsExtents->second;
}

struct Estimated {
    Estimated(std::size_t count, std::size_t estimated)
        : count(count), estimated(estimated)
    {}

    std::size_t count;
    std::size_t estimated;
};

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const Estimated &e)
{
    if (e.estimated) {
        double percentage((100.0 * e.count) / e.estimated);
        boost::io::ios_precision_saver ps(os);
        return os << '#' << e.count << " of " << e.estimated << " ("
                  << std::fixed << std::setprecision(2)
                  << std::setw(6) << percentage
                  << " % done)";
    }
    return os << '#' << e.count;
}

void Encoder::Detail::process(const TileId &tileId
                              , ConstraintsFlag::type useConstraints
                              , const NodeInfo &nodeInfo
                              , const TileResult &parentTile)
{
    struct TIDGuard {
        TIDGuard(const std::string &id)
            : old(dbglog::thread_id())
        {
            dbglog::thread_id(id);
        }
        ~TIDGuard() { try { dbglog::thread_id(old); } catch (...) {} }

        const std::string old;
    };

    // skip invalid node
    if (!nodeInfo.valid()) { return; }

    if (constraints.validTree && !constraints.validTree->get(tileId)) {
        // tile not in valid tree-> stop
        return;
    }

    bool processTile(true);

    if (useConstraints & ConstraintsFlag::useLodRange) {
        if (tileId.lod < constraints.lodRange->min) {
            // no data yet -> go directly down
            // * equivalent to TileResult::noDataYet
            processTile = false;
        }
        if (tileId.lod > constraints.lodRange->max) {
            // nothing can live down here -> done
            // * equivalent to TileResult::noData
            return;
        }
    }

    TIDGuard tg(str(boost::format("tile:%s") % tileId));

    const auto &extents(nodeInfo.extents());
    if (useConstraints & ConstraintsFlag::useExtents) {
        if (!overlaps(getExtents(nodeInfo.srs()), extents)) {
            // nothing can live out here -> done
            // * equivalent to TileResult::noData
            return;
        }
    }

    // skip unproductive tiles
    if (!nodeInfo.productive()) {
        processTile = false;
    }

    TileResult tile;
    if (processTile) {
        LOG(info2)
            << "Trying to generate " << tileId << " (" << nodeInfo.srs()
            << ", extents: " << std::fixed << extents
            << (nodeInfo.partial() ? ", partial" : "") << ").";

        tile = owner->generate(tileId, nodeInfo, parentTile);
        switch (auto result = tile.result()) {
        case TileResult::Result::tile:
        case TileResult::Result::source:
        {
            auto number(++generated_);

            LOGR(options.level())
                << "Generated tile " << Estimated(number, estimated_) << ": "
                << tileId << " ("  << nodeInfo.srs()
                << ", extents: " << std::fixed << extents
                << (nodeInfo.partial() ? ", partial" : "")
                << ") [" << TileFlags{tile} << "].";

            bool hasMesh(false);
            if (result == TileResult::Result::tile) {
                const auto &t(tile.tile());

                UTILITY_OMP(critical)
                tileSet.setTile(tileId, t, nodeInfo);

                hasMesh = bool(t.mesh);
            } else {
                const auto &t(tile.source());

                UTILITY_OMP(critical)
                tileSet.setTile(tileId, t, nodeInfo);

                hasMesh = bool(t.mesh);
            }

            // we hit a tile with mesh -> do not apply extents constraints for
            // children if set
            ConstraintsFlag::clearExtents(useConstraints, hasMesh);
            break;
        }

        case TileResult::Result::influenced: {
            auto number(++generated_);

            // no data generated, only influenced flag to be set
            LOGR(options.level())
                << "Generated tile " << Estimated(number, estimated_) << ": "
                << tileId << " ("  << nodeInfo.srs()
                << ", extents: " << std::fixed << extents
                << (nodeInfo.partial() ? ", partial" : "")
                << ") [influenced].";

            tileSet.markInfluencedTile(tileId);
        } break;

        case TileResult::Result::noDataYet:
            // fine, something could be down there
            break;

        case TileResult::Result::noData:
            // no data and nothing will ever be there
            return;
        }
    } else {
        LOG(info3)
            << "Falling through " << tileId << " ("  << nodeInfo.srs()
            << ", extents: " << std::fixed << extents
            << (nodeInfo.partial() ? ", partial" : "")
            << ").";
    }

    // we can proces children -> go down
    for (auto child : children(tileId)) {
        // compute child node
        auto childNode(nodeInfo.child(child));

        UTILITY_OMP(task)
        process(child, useConstraints, childNode, tile);
    }
}

Encoder::Encoder(const boost::filesystem::path &path
                 , const TileSetProperties &properties, CreateMode mode
                 , const Options &options)
    : detail_(std::make_shared<Detail>(this, path, properties, mode, options))
{}

Encoder::Encoder(TileSet &tileset, const Options &options)
    : detail_(std::make_shared<Detail>(this, tileset, options))
{}

TileSetProperties Encoder::properties() const
{
    return detail_->properties;
}

const registry::ReferenceFrame& Encoder::referenceFrame() const
{
    return detail_->referenceFrame;
}

const std::string& Encoder::physicalSrsId() const
{
    return detail_->referenceFrame.model.physicalSrs;
}

const registry::Srs& Encoder::physicalSrs() const
{
    return detail_->physicalSrs;
}

const std::string& Encoder::navigationSrsId() const
{
    return detail_->referenceFrame.model.navigationSrs;
}

const registry::Srs& Encoder::navigationSrs() const
{
    return detail_->navigationSrs;
}

void Encoder::setConstraints(const Constraints &constraints)
{
    detail_->setConstraints(constraints);
}

TileSet Encoder::run()
{
    return detail_->run();
}

std::size_t Encoder::threadIndex() const
{
    return omp_get_thread_num();
}

void Encoder::setEstimatedTileCount(std::size_t count)
{
    detail_->estimated_ = count;
}

void Encoder::updateEstimatedTileCount(int diff)
{
    if (!diff) { return; }

    if (diff > 0) {
        detail_->estimated_ += diff;
    } else {
        std::size_t d(-diff);
        if (detail_->estimated_ > d) {
            detail_->estimated_ -= d;
        } else {
            detail_->estimated_ = 0;
        }
    }
}

} } // namespace vtslibs::vts
