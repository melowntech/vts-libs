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
 * \file vts/storage/storage.cpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile set storage change operations.
 * Not present in vts-core library.
 */

#include <memory>
#include <string>
#include <exception>
#include <algorithm>
#include <iterator>
#include <functional>

#include <boost/optional.hpp>
#include <boost/noncopyable.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/range/adaptor/reversed.hpp>

#include "utility/streams.hpp"
#include "utility/openmp.hpp"
#include "utility/path.hpp"
#include "utility/duration.hpp"
#include "utility/time.hpp"
#include "utility/expect.hpp"

#ifdef UTILITY_HAS_PROC
#  include "utility/procstat.hpp"
#endif

#include "../../storage/error.hpp"
#include "../../storage/tidguard.hpp"
#include "../storage.hpp"
#include "../../vts.hpp"
#include "detail.hpp"
#include "../tileset/detail.hpp"
#include "../encoder.hpp"
#include "../io.hpp"

#include "config.hpp"
#include "paths.hpp"
#include "gluerules.hpp"
#include "mergeconf.hpp"
#include "locking.hpp"

namespace fs = boost::filesystem;

namespace vtslibs { namespace vts {

namespace {

#ifdef UTILITY_HAS_PROC

void reportMemoryUsage(const std::string &what)
{
    const double kb2gb(1 << 20);
    auto stat(utility::getProcStat());
    LOG(info3)
        << std::fixed
        << "Memory occupied " << what << ": " << (stat.occupies() / kb2gb)
        << " GiB (of which " << (stat.swap / kb2gb) << " GiB is swapped).";
}

#else

void reportMemoryUsage(const std::string&) {}

#endif

inline std::string glueId2path(const Glue::Id &id)
{
    return boost::lexical_cast<std::string>(utility::join(id, "_"));
}

Storage::AddOptions updateAddOptions(Storage::AddOptions addOptions
                                     , const MergeConf &mergeConf)
{
    addOptions.openOptions.updateCNames(mergeConf.cnames);
    addOptions.openOptions.scarceMemory(true);
    return addOptions;
}

inline std::string lockName(const TilesetId &tilesetId) { return tilesetId; }
inline std::string lockName(const Glue &glue) { return glue.path; }
inline std::string lockName(const VirtualSurface &virtualSurface) {
    return "#" + virtualSurface.path;
}

} // namespace

void Storage::add(const boost::filesystem::path &tilesetPath
                  , const Location &where, const TilesetId &tilesetId
                  , const AddOptions &addOptions)
{
    const auto ao
        (updateAddOptions
         (addOptions
          , loadMergeConf(detail().root / storage_paths::mergeConfPath()
                          , true)));

    auto ts(openTileSet(tilesetPath, ao.openOptions));
    detail().add(ts, where
                 , (tilesetId.empty() ? ts.getProperties().id : tilesetId)
                 , ao);
}

void Storage::generateGlues(const TilesetId &tilesetId
                            , const AddOptions &addOptions)
{
    const auto ao
        (updateAddOptions
         (addOptions
          , loadMergeConf(detail().root / storage_paths::mergeConfPath()
                          , true)));

    detail().generateGlues(tilesetId, ao);
}

void Storage::generateGlue(const Glue::Id &glueId
                           , const AddOptions &addOptions)
{
    const auto ao
        (updateAddOptions
         (addOptions
          , loadMergeConf(detail().root / storage_paths::mergeConfPath()
                          , true)));

    detail().generateGlue(detail().properties.normalize(glueId)
                          , ao);
}

void Storage::remove(const TilesetIdList &tilesetIds)
{
    detail().remove(tilesetIds);
}

void Storage::createVirtualSurface( const TilesetIdSet &tilesets
                                  , const CloneOptions &createOptions)
{
    detail().createVirtualSurface(tilesets, createOptions);
}

void Storage::removeVirtualSurface(const TilesetIdSet &tilesets)
{
    detail().removeVirtualSurface(tilesets);
}

namespace {

void rmrf(const fs::path &path)
{
    boost::system::error_code ec;
    remove_all(path, ec);
}

class Tx : boost::noncopyable {
public:
    Tx(const fs::path &root, const boost::optional<fs::path> &tmpRoot
       , const OpenOptions &openOptions = OpenOptions())
        : root_(root), tmpRoot_(tmpRoot), openOptions_(openOptions)
    {
        prepare();
    }

    Tx(Tx &&other)
        : root_(std::move(other.root_))
        , tmpRoot_(std::move(other.tmpRoot_))
        , openOptions_(std::move(other.openOptions_))
        , glueRules_(std::move(other.glueRules_))
    {
        std::swap(mapping_, other.mapping_);
    }

    ~Tx();

    /** Create sub transaction.
     */
    Tx subtx() const;

    /** Join in other transaction.
     */
    void join(Tx &other);

    void add(const fs::path &work, const fs::path &dst);

    const fs::path root() const { return root_; }

    fs::path tilesetPath(const std::string &tilesetId, bool tmp = false)
        const
    {
        return createPath(storage_paths::tilesetPath
                          (root_, tilesetId, tmp, tmpRoot_));
    }

    fs::path gluePath(const Glue &glue, bool tmp = false)
        const
    {
        return createPath(storage_paths::gluePath
                          (root_, glue, tmp, tmpRoot_));
    }

    fs::path virtualSurfacePath(const VirtualSurface &virtualSurface
                                , bool tmp = false)
        const
    {
        return createPath(storage_paths::virtualSurfacePath
                          (root_, virtualSurface, tmp, tmpRoot_));
    }

    TileSet open(const TilesetId &tilesetId) const;

    TileSet open(const Glue &glue) const;

    fs::path addGlue(const Glue &glue);

    fs::path addTileset(const std::string &tilesetId);

    fs::path addVirtualSurface(const VirtualSurface &virtualSurface);

    void remove(const fs::path &path);

    void commit();

    const GlueRule::list& glueRules() const { return glueRules_; }

    /** Returns real path for temporary path.
     *  Returns provided path if there is no such temporaty path.
     */
    fs::path realPath(const fs::path &tmpPath) const;

private:
    struct SubTx {};
    Tx(const SubTx&, const Tx &other);

    void rollback();

    fs::path createPath(const fs::path &path) const;

    void prepare();

    const fs::path root_;
    const boost::optional<fs::path> &tmpRoot_;
    const OpenOptions openOptions_;

    typedef std::map<fs::path, fs::path> Mapping;
    Mapping mapping_;

    GlueRule::list glueRules_;
};

void Tx::prepare()
{
    glueRules_ = loadGlueRules(root_ / storage_paths::glueRulesPath(), true);
}

Tx::~Tx() {
    if (std::uncaught_exception()) {
        // we cannot throw!
        rollback();
    }
}

Tx::Tx(const SubTx&, const Tx &other)
    : root_(other.root_)
    , tmpRoot_(other.tmpRoot_)
    , openOptions_(other.openOptions_)
    , glueRules_(other.glueRules_)
{}

Tx Tx::subtx() const { return Tx(SubTx(), *this); }

void Tx::join(Tx &other)
{
    // copy stuff
    mapping_.insert(other.mapping_.begin(), other.mapping_.end());
    // purge other
    other.mapping_.clear();
}

void Tx::add(const fs::path &work, const fs::path &dst)
{
    mapping_.insert(Mapping::value_type(work, dst));
}

void Tx::rollback()
{
    for (const auto &item : mapping_) {
        try { rmrf(item.first); } catch (...) {}
    }
    mapping_.clear();
}

void Tx::commit()
{
    for (const auto &item : mapping_) {
        if (item.second.empty()) {
            LOG(info2) << "commit(rm(" << item.first << "))";
            // no destination -> just remove
            rmrf(item.first);
        } else {
            LOG(info2)
                << "commit(rm(" << item.second << "), "
                << "mv(" << item.first << ", " << item.second << "))";
            // remove old stuff
            rmrf(item.second);
            // move new stuff there
            rename(item.first, item.second);
        }
    }
    mapping_.clear();
}

TileSet Tx::open(const TilesetId &tilesetId) const
{
    return openTileSet(tilesetPath(tilesetId), openOptions_);
}

TileSet Tx::open(const Glue &glue) const
{
    return openTileSet(gluePath(glue), openOptions_);
}

fs::path Tx::addGlue(const Glue &glue)
{
    auto tmp(gluePath(glue, true));
    add(tmp, gluePath(glue));
    return tmp;
}

fs::path Tx::addTileset(const std::string &tilesetId)
{
    auto tmp(tilesetPath(tilesetId, true));
    add(tmp, tilesetPath(tilesetId));
    return tmp;
}

fs::path Tx::addVirtualSurface(const VirtualSurface &virtualSurface)
{
    auto tmp(virtualSurfacePath(virtualSurface, true));
    add(tmp, virtualSurfacePath(virtualSurface));
    return tmp;
}

fs::path Tx::createPath(const fs::path &path) const
{
    fs::create_directories(path.parent_path());
    return path;
}

void Tx::remove(const fs::path &path)
{
    mapping_[path] = fs::path();
}

fs::path Tx::realPath(const fs::path &tmpPath) const
{
    auto fmapping_(mapping_.find(tmpPath));
    if (fmapping_ == mapping_.end()) { return tmpPath; }
    return fmapping_->second;
}

} // namespace


namespace {

typedef std::vector<TileSet> TileSets;
typedef std::vector<TileIndex> TileIndices;

std::tuple<TileSets, std::size_t>
openTilesets(Tx &tx, const StoredTileset::list &infos, const TileSet &tileset
             , const vts::TilesetId &addedId)
{
    std::tuple<TileSets, std::size_t> res;
    TileSets &tilesets(std::get<0>(res));
    std::size_t index(0);
    for (const auto &info : infos) {
        if (info.tilesetId == addedId) {
            tilesets.push_back(tileset);
            std::get<1>(res) = index;
            LOG(info2) << "Reused already open <" << addedId << ">.";
        } else {
            tilesets.push_back(tx.open(info.tilesetId));
            LOG(info2) << "Opened tileset <" << info.tilesetId << ">.";
        }

        ++index;
    }
    return res;
}

TileSets openTilesets(Tx &tx, const StoredTileset::list &infos)
{
    TileSets tilesets;
    std::size_t index(0);
    for (const auto &info : infos) {
        tilesets.push_back(tx.open(info.tilesetId));
        LOG(info2) << "Opened tileset <" << info.tilesetId << ">.";
        ++index;
    }
    return tilesets;
}

LodRange range(const TileSets &tilesets)
{
    auto lr(LodRange::emptyRange());
    for (const auto &ts : tilesets) {
        lr = unite(lr, ts.lodRange());
    }
    return lr;
}

struct Ts {
    typedef std::vector<Ts> list;
    typedef std::vector<Ts*> ptrlist;
    typedef std::vector<const Ts*> const_ptrlist;

    /** Own index in the list of tilesets.
     */
    std::size_t index;

    /** Marks added tileset.
     */
    bool added;

    /** This tileset
     */
    TileSet set;

    /** Stored ID.
     */
    StoredTileset stored;

    /** Set of tilesets that overlap with this one.
     */
    std::set<std::size_t> incidentSets;

    Ts(int index, const TileSet &tileset, const LodRange &lodRange
       , const Storage::Properties &properties, bool added)
        : index(index), added(added), set(tileset)
        , stored(properties.tilesets[index])
        , lodRange(lodRange)
    {}

    bool notoverlaps(const Ts &other) const {
        return sphereOfInfluence().notoverlaps
            (other.sphereOfInfluence(), TileIndex::Flag::any);
    }

    std::string id() const { return stored.tilesetId; }

    std::string base() const { return stored.baseId; }

    const TileIndex& sphereOfInfluence() const {
        if (!sphereOfInfluence_) {
            sphereOfInfluence_ =
                (set.sphereOfInfluence(lodRange, TileIndex::Flag::mesh));
        }
        return *sphereOfInfluence_;
    }

    bool tileindexIdentical(const const_ptrlist &tss) const {
        // tile indices are identical if they have same quality for mesh and
        // watertighness
        const TileIndex::Flag::value_type mask
            (TileIndex::Flag::mesh | TileIndex::Flag::watertight);

        const auto compare([](TileIndex::Flag::value_type v1
                              , TileIndex::Flag::value_type v2)
        {
            return (v1 & mask) == (v2 & mask);
        });

        const auto &ti(set.tileIndex());
        for (const auto *ts : tss) {
            if (ti.identical(ts->set.tileIndex(), compare)) { return true; }
        }
        return false;
    }

private:
    /** Tileset's sphere of influence.
     */
    mutable boost::optional<TileIndex> sphereOfInfluence_;

    LodRange lodRange;
};

class BitSet {
public:
    BitSet(std::size_t size = 0) : bs_(size, 0) {}

    bool increment();

    typedef std::vector<bool> repr_type;

    const repr_type& repr() const { return bs_; }

    typedef repr_type::iterator iterator;
    typedef repr_type::const_iterator const_iterator;

    iterator begin() { return bs_.begin(); }
    iterator end() { return bs_.end(); }
    const_iterator begin() const { return bs_.begin(); }
    const_iterator end() const { return bs_.end(); }
    const_iterator cbegin() { return bs_.begin(); }
    const_iterator cend() { return bs_.end(); }
    repr_type::reference operator[](std::size_t index) { return bs_[index]; }
    repr_type::const_reference operator[](std::size_t index) const {
        return bs_[index];
    }

private:
    std::vector<bool> bs_;
};

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const BitSet &bs)
{
    for (auto bit : boost::adaptors::reverse(bs.repr())) {
        os << (bit ? '1' : '0');
    }
    return os;
}

bool BitSet::increment()
{
    for (std::size_t i(0), e(bs_.size()); i != e; ++i) {
        auto nval(!bs_[i]);
        bs_[i] = nval;
        if (nval) { return true; }
    }
    return false;
}

struct GlueDescriptor {
    std::size_t index;
    TileSet::list combination;
    Glue glue;
    TilesetId glueSetId;

    GlueDescriptor(std::size_t index, const TileSet::list &combination
                   , const Glue &glue, const TilesetId &glueSetId)
        : index(index), combination(combination), glue(glue)
        , glueSetId(glueSetId)
    {}

    typedef std::vector<GlueDescriptor> list;
};

GlueDescriptor::list prepareGlues(Tx &tx, Ts::list &tilesets, Ts &added
                                  , const Storage::AddOptions &addOptions)
{
    Ts::ptrlist incidentSets;
    {
        //save first for added tileset itself
        incidentSets.push_back(&added);

        for (auto &ts : tilesets) {
            // ignore added tileset
            if (ts.added) {continue; }
            if (ts.notoverlaps(added)) {
                LOG(info1) << "Tileset <" << ts.id()
                           << "> does not overlap added tileset <"
                           << added.id() << ">.";
                continue;
            }

            // incidence between spheres of influence -> remember
            LOG(info1) << "Adding <" << ts.id() << "> to incident set.";
            incidentSets.push_back(&ts);
            // note that added tileset coincides with this
            incidentSets[0]->incidentSets.insert(ts.index);
        }
    }

    // for each tileset in the input
    {
        // TODO skip first since it coverder in preceeding FOR
        for (auto iincidentSets(incidentSets.begin())
                 , eincidentSets(incidentSets.end());
             iincidentSets != eincidentSets; ++iincidentSets)
        {
            auto &first(**iincidentSets);

            // process all remaining tilesets
            for (auto iincidentSets2(std::next(iincidentSets));
                 iincidentSets2 != eincidentSets; ++iincidentSets2)
            {
                auto &second(**iincidentSets2);

                // overlaps -> remember
                if (!first.notoverlaps(second)) {
                    first.incidentSets.insert(second.index);
                }
            }
        }
    }

    /* Build putative glues:
     * do a depth-first search where any two consecutive datasets are incident
     * this may yield some false positives ( A->B, A->C, B->C coincide does not
     * imply ABC coincide) but these will be ruled out in the next step.
     */

    GlueDescriptor::list gd;
    {
        // fwd
        std::function<void(const Ts&, Ts::const_ptrlist
                           , GlueRuleChecker, std::set<TilesetId>)>
            buildGlueCombination;

        // lambda
        buildGlueCombination = ([&]( const Ts& tsToAdd
            , Ts::const_ptrlist glueMembers
            , GlueRuleChecker ruleChecker
            , std::set<TilesetId> seenBases) -> void
        {
            LOG(info2) << "Trying tileset: " << tsToAdd.index << " <"
                       << tsToAdd.id() << "> ";
            if (!seenBases.insert(tsToAdd.base()).second) {
                // this base tileset has already been seen
                LOG(info1) << "Backtracking due to different version.";
                return;
            }

            if (!ruleChecker(tsToAdd.stored)) {
                // glue rule prevents glue generation
                LOG(info1) << "Backtracking due to rule check failed.";
                return;
            }

            // test if really incident with all already added
            for (auto & ats : glueMembers) {
                if (ats->incidentSets.find(tsToAdd.index)
                    == ats->incidentSets.end())
                {
                    LOG(info1)
                        << "Backtracking due to not incident with all "
                        "previous tilesets.";
                    return;
                }
            }

            // do not descend this path if tileindex is the same as one of the
            // previous tileset's tileindices and we are allowed to make such
            // shortcut
            if (addOptions.checkTileindexIdentity
                && tsToAdd.tileindexIdentical(glueMembers))
            {
                LOG(info1) << "Backtracking due to identical tileindices.";
                return;
            }

            // seems fine, add tileset
            glueMembers.push_back(&tsToAdd);

            if (glueMembers.size() > 1) {
                // sort glue content
                std::sort( glueMembers.begin(), glueMembers.end()
                         , [](const Ts* a, const Ts* b) {
                            return a->index < b->index;
                         } );

                Glue glue;
                TileSet::list combination;

                for (const auto gts : glueMembers) {
                    glue.id.push_back(gts->id());
                    combination.push_back(gts->set);
                }

                auto glueSetId(glueId2path(glue.id));
                glue.path = glueSetId;

                // create glue
                gd.emplace_back(gd.size() + 1, combination, glue, glueSetId);
            }

            // try tilesets incident with this tileset
            for ( auto its(tsToAdd.incidentSets.begin())
                ; its != tsToAdd.incidentSets.end(); ++its)
            {
                buildGlueCombination(tilesets[*its], glueMembers
                                     , ruleChecker, seenBases);
            }
        });

        buildGlueCombination(added, {}, GlueRuleChecker(tx.glueRules()), {});
    }

    // result
    return gd;
}

GlueDescriptor::list
prepareGlues(Tx &tx, Storage::Properties properties
             , const std::tuple<TileSets, std::size_t> &tsets
             , const Storage::AddOptions &addOptions)
{
    if (properties.tilesets.size() <= 1) {
        LOG(info3) << "No need to create any glue.";
        return {};
    }

    // create tileset list
    Ts::list tilesets;
    {
        // accumulate lod range for all tilesets
        auto lr(range(std::get<0>(tsets)));
        LOG(info1) << "Glue lod range: " << lr;

        std::size_t index(0);
        for (auto &set : std::get<0>(tsets)) {
            tilesets.emplace_back(index, set, lr, properties
                                  , (index == std::get<1>(tsets)));
            ++index;
        }
    }

    // prepare glues
    auto gds(prepareGlues(tx, tilesets, tilesets[std::get<1>(tsets)]
                          , addOptions));

    // done
    return gds;
}

GlueDescriptor::list
prepareGlues(const Glue::list &glues, const TileSets &tilesets)
{
    GlueDescriptor::list gds;

    for (const auto &glue : glues) {
        TileSet::list combination;

        auto itilesets(tilesets.begin()), etilesets(tilesets.end());
        auto iglueId(glue.id.begin()), eglueId(glue.id.end());

        while ((itilesets != etilesets) && (iglueId != eglueId)) {
            const auto &ts(*itilesets);
            const auto tsId(ts.id());
            const auto &gtsId(*iglueId);

            if (tsId != gtsId) {
                ++itilesets;
                continue;
            }

            combination.push_back(ts);
            ++itilesets;
            ++iglueId;
        }

        utility::expect((iglueId == eglueId)
                        , "Cannot assign tilesets to glue <%s>"
                        , utility::join(glue.id, ","));

        auto glueSetId(glueId2path(glue.id));
        gds.emplace_back(gds.size() + 1, combination, glue, glueSetId);
    }

    // done
    return gds;
}

void writePendingGlues(Storage::Properties &properties
                       , const GlueDescriptor::list &gds)
{
    for (const auto &gd : gds) {
        properties.pendingGlues.insert(gd.glue.id);
    }
}

Glue createGlue(Tx &tx, const GlueDescriptor &gd
                , const Storage::AddOptions &addOptions
                , std::size_t glueCount)
{
    LOG(info3)
        << "Trying to generate glue #" << gd.index
        << '/' << glueCount << " <" << gd.glueSetId << ">.";
    vtslibs::storage::TIDGuard tg
        (str(boost::format("%d:%s") % gd.index % gd.glueSetId)
         , true);

    TileSetProperties gprop;
    gprop.id = gd.glueSetId;
    gprop.referenceFrame
        = gd.combination.front().getProperties().referenceFrame;

    reportMemoryUsage("before glue creation");

    // create glue
    auto gPath(tx.addGlue(gd.glue));
    auto gts(createTileSet(gPath, gprop, CreateMode::overwrite));
    // make sure we have newer revision than any possible other revision
    gts.ensureRevision(tx.realPath(gPath));

    // create glue
    utility::DurationMeter timer;
    TileSet::createGlue(gts, gd.combination, addOptions);
    auto duration(timer.duration());

    reportMemoryUsage("after merge");

    // empty cached input data (no need for them now)
    for (auto &ts : gd.combination) { ts.emptyCache(); }

    reportMemoryUsage("after emptying input caches");

    Glue glue(gd.glue);

    if (gts.empty()) {
        // unusable
        LOG(info3)
            << "Glue <" << gprop.id  << "> contains no tile; "
            << "ignoring. Duration: "
            << utility::formatDuration(duration) << ".";

        tx.remove(gPath);
        glue.path = {};
    } else {
        // usable
        LOG(info3)
            << "Glue <" << gprop.id  << "> created, duration: "
            << utility::formatDuration(duration) << ".";

        // flush
        gts.flush();

        reportMemoryUsage("after glue flush");
    }

    reportMemoryUsage("after glue creation");
    return glue;
}

} // namespace

std::tuple<Storage::Properties, StoredTileset>
Storage::Detail::addTileset(const Properties &properties
                            , const TilesetId &tilesetId
                            , const AddOptions &addOptions
                            , const Location &where) const
{
    if (tilesetId.find('@') != std::string::npos) {
        LOGTHROW(err1, vtslibs::storage::TileSetAlreadyExists)
            << "Invalid character in tileset ID <" << tilesetId << ">.";
    }

    StoredTileset tileset;
    {
        tileset.baseId = tilesetId;
        auto lastVersion(properties.lastVersion(tilesetId));
        if (lastVersion >= 0) {
            if (!addOptions.bumpVersion) {
                LOGTHROW(err1, vtslibs::storage::TileSetAlreadyExists)
                    << "Tileset <" << tilesetId
                    << "> already present in storage "
                    << root << ". Use version bumping to introduce "
                    "new version.";
            }

            tileset.version = lastVersion + 1;
            tileset.baseId = tilesetId;
            tileset.tilesetId = str(boost::format("%s@%d")
                                    % tilesetId % tileset.version);
            LOG(info2)
                << "Bumping version " << tileset.version
                << " of tileset <" << tileset.baseId << "> under name ID <"
                << tileset.tilesetId << ">";
        } else {
            tileset.baseId = tileset.tilesetId = tilesetId;
            tileset.version = 0;
        }

        // assign tags
        tileset.tags = addOptions.tags;
    }

    if (properties.hasTileset(tileset.tilesetId)) {
        LOGTHROW(err1, vtslibs::storage::TileSetAlreadyExists)
            << "Tileset <" << tileset.tilesetId
            << "> already present in storage "
            << root << ".";
    }

    std::tuple<Properties, StoredTileset> res;
    auto &p(std::get<0>(res) = properties);
    std::get<1>(res) = tileset;

    auto &tilesets(p.tilesets);

    if (where.where.empty()) {
        // void reference
        if (where.direction == Location::Direction::below) {
            // below void -> to the top of the stack
            tilesets.push_back(tileset);
        } else {
            // above void -> to the bottom of the stack
            tilesets.insert(tilesets.begin(), tileset);
        }
        return res;
    }

    // some reference
    auto ftilesets(p.findTilesetIt(where.where));
    if (ftilesets == tilesets.end()) {
        LOGTHROW(err1, vtslibs::storage::NoSuchTileSet)
            << "Tileset <" << where.where << "> (used as a reference) "
            "not found in storage " << root << ".";
    }

    if (where.direction == Location::Direction::below) {
        // below given reference -> just insert here
        tilesets.insert(ftilesets, tileset);
    } else {
        // above reference -> advance and insert
        tilesets.insert(std::next(ftilesets), tileset);
    }

    return res;
}

std::tuple<Storage::Properties, Glue::map, VirtualSurface::map>
Storage::Detail::removeTilesets(const Properties &properties
                                , const TilesetIdList &tilesetIds)
    const
{
    std::tuple<Properties, Glue::map, VirtualSurface::map>
        res(properties, {}, {});
    auto &p(std::get<0>(res));
    auto &tilesets(p.tilesets);

    for (const auto &tilesetId : tilesetIds) {
        auto ftilesets(p.findTilesetIt(tilesetId));
        if (ftilesets == tilesets.end()) {
            LOG(warn2) << "Tileset <" << tilesetId << "> "
                "not found in storage " << root << ".";
        } else {
            // remove from tilesets
            tilesets.erase(ftilesets);
        }
    }

    // drop all glues that reference requested tilesets
    auto &resGlues(std::get<1>(res));
    for (const auto &tilesetId : tilesetIds) {
        for (auto iglues(p.glues.begin()); iglues != p.glues.end(); ) {
            if (iglues->second.references(tilesetId)) {
                resGlues.insert(*iglues);
                iglues = p.glues.erase(iglues);
            } else {
                ++iglues;
            }
        }

        for (auto iglues(p.pendingGlues.begin());
             iglues != p.pendingGlues.end(); )
        {
            if (Glue::references(*iglues, tilesetId)) {
                iglues = p.pendingGlues.erase(iglues);
            } else {
                ++iglues;
            }
        }

        for (auto iglues(p.emptyGlues.begin());
             iglues != p.emptyGlues.end(); )
        {
            if (Glue::references(*iglues, tilesetId)) {
                iglues = p.emptyGlues.erase(iglues);
            } else {
                ++iglues;
            }
        }
    }

    auto &virtualSurfaces(p.virtualSurfaces);
    auto &resVirtualSurfaces(std::get<2>(res));
    for (const auto &tilesetId : tilesetIds) {
        for (auto ivirtualSurfaces(virtualSurfaces.begin());
             ivirtualSurfaces != virtualSurfaces.end(); )
        {
            if (ivirtualSurfaces->second.references(tilesetId)) {
                resVirtualSurfaces.insert(*ivirtualSurfaces);
                ivirtualSurfaces = virtualSurfaces.erase(ivirtualSurfaces);
            } else {
                ++ivirtualSurfaces;
            }
        }
    }

    return res;
}

std::tuple<Storage::Properties, VirtualSurface::map>
Storage::Detail::removeVirtualSurfaces(const Properties &properties
                                       , const VirtualSurface::Ids &ids)
    const
{
    std::tuple<Properties, VirtualSurface::map> res(properties, {});
    auto &p(std::get<0>(res));
    auto &resVirtualSurfaces(std::get<1>(res));

    for (const auto &id : ids) {
        auto fvirtualSurfaces(p.findVirtualSurface(id));
        if (fvirtualSurfaces == p.virtualSurfaces.end()) {
            LOG(warn2)
                << "Virtual surface <"
                << utility::join(id, ",") << "> "
                "not found in storage " << root << ".";
        } else {
            resVirtualSurfaces.insert(*fvirtualSurfaces);
            p.virtualSurfaces.erase(fvirtualSurfaces);
        }
    }

    return res;
}

void generateGluesImpl(Tx &tx, const GlueDescriptor::list &gds
                       , Storage::Detail &detail
                       , Storage::Properties &properties
                       , const Storage::AddOptions &addOptions)
{
    // report
    LOG(info3) << "Generating " << gds.size() << " glue(s):";
    for (const auto &gd : gds) {
        LOG(info3)
            << "    #" << gd.index
            << '/' << gds.size() << " <" << gd.glueSetId << ">.";
    }

    // not a lazy add, try to generate all glues

    // prepare progress if available
    if (addOptions.progress) {
        // TODO: optimize when there is just one glue

        // accumulate total number of tiles to generate
        std::size_t total(0);
        for (const auto &gd : gds) {
            total += TileSet::analyzeGlue(gd.combination, addOptions)
                .tilesToGenerate;
        }
        // notify progress about how many tiles to expect
        addOptions.progress->expect(total);
    }

    auto validGlue([&](const Glue::Id &id) -> bool
    {
        if (!properties.knownGlue(id)) {
            // hm... someone did something nasty and this glue is no longer
            // member of this storage
            LOG(warn3) << "Glue <" << utility::join(id, ",")
                       << "> does no longer participate in this storage.";
            return false;
        }
        return true;
    });

    // run the thing
    for (const auto &gd : gds) {
        // create glue

        // skip if invalid
        if (!validGlue(gd.glue.id)) { continue; }

        // create a sub-transaction
        auto subTx(tx.subtx());

        Glue glue;
        try {
            // create glue under glue lock with unlocked storage
            ScopedStorageLock glueLock(&detail.storageLock, lockName(gd.glue));
            glue = createGlue(subTx, gd, addOptions, gds.size());
        } catch (const StorageComponentLocked&) {
            LOG(warn3) << "Unable to lock glue <"
                       << utility::join(gd.glue.id, ",")
                       << ">; skipping.";
            continue;
        }

        // legacy mode?
        if (addOptions.mode == Storage::AddOptions::Mode::legacy) {
            // legacy mode, update properties
            properties.glueGenerated(glue);
            // and join sub transaction into main transaction
            tx.join(subTx);
            continue;
        }

        // update stored properties

        // re-load stored properties (may be changed by another storage access
        // instance)
        properties = detail.readConfig();

        // skip if invalid
        if (!validGlue(glue.id)) { continue; }

        // update properties, save, commit
        properties.glueGenerated(glue);
        detail.saveConfig(properties);

        // main transaction is not commit changes to the sub transaction
        subTx.commit();
    }
}

void Storage::Detail::add(const TileSet &tileset, const Location &where
                          , const TilesetId &tilesetId
                          , const AddOptions &addOptions)
{
    if (storageLock && (addOptions.mode == AddOptions::Mode::legacy)) {
        LOGTHROW(err2, vtslibs::storage::InconsistentInput)
            << "Legacy add mode is incompatible with locker2 storage locking "
            "strategy.";
    }

    // (re)load config to have fresh copy when under lock
    if (storageLock) { loadConfig(); }

    std::string simulation(addOptions.dryRun ? "(simulation) " : "");
    vtslibs::storage::TIDGuard tg
        (str(boost::format("%sadd(%s)") % simulation % tilesetId));

    // check compatibility
    if (tileset.getProperties().referenceFrame != properties.referenceFrame) {
        LOGTHROW(err1, vtslibs::storage::IncompatibleTileSet)
            << "Tileset <" << tilesetId << "> "
            "uses different reference frame ("
            << tileset.getProperties().referenceFrame
            << ") from the one  this storage supports ("
            << properties.referenceFrame << ").";
    }

    // prepare new tileset list
    Properties nProperties;
    StoredTileset tilesetInfo;
    std::tie(nProperties, tilesetInfo)
        = addTileset(properties, tilesetId, addOptions, where);

    LOG(info3)
        << "Adding tileset <" << tileset.id() << "> (from "
        << tileset.root() << ").";

    Tx tx(root, addOptions.tmp, addOptions.openOptions);

    auto dst([&]() -> TileSet
    {
        if (addOptions.dryRun) { return tileset; }

        // lock tileset when cloning
        ScopedStorageLock tsLock
            (&storageLock, lockName(tilesetInfo.tilesetId));

        // create tileset at work path (overwrite any existing stuff here)
        // NB: we have to clone original tileset's content as-is!
        return cloneTileSet(tx.addTileset(tilesetInfo.tilesetId), tileset
                            , CloneOptions()
                            .mode(CreateMode::overwrite)
                            .sameType(true)
                            .tilesetId(tilesetInfo.tilesetId)
                            .lodRange(addOptions.filter.lodRange())
                            .openOptions(addOptions.openOptions)
                            );
    }());

    auto tilesets(openTilesets(tx, nProperties.tilesets
                               , dst, tilesetInfo.tilesetId));

    auto gds(prepareGlues(tx, nProperties, tilesets, addOptions));

    // dry run -> do nothing
    if (addOptions.dryRun) { return; }

    writePendingGlues(nProperties, gds);

    if (addOptions.mode != AddOptions::Mode::legacy) {
        // new interface: commit new properties and changes to transaction
        saveConfig(nProperties);
        tx.commit();
    }

    // lazy add? wrap it here
    if (addOptions.mode == AddOptions::Mode::lazy) { return; }

    // generate all glue
    generateGluesImpl(tx, gds, *this, nProperties, addOptions);

    if (addOptions.mode == AddOptions::Mode::legacy) {
        // old interface: flush and done
        saveConfig(nProperties);
        tx.commit();
    }
}

void Storage::Detail::generateGlues(const TilesetId &tilesetId
                                    , const AddOptions &addOptions)
{
    // (re)load config to have fresh copy when under lock
    if (storageLock) { loadConfig(); }

    Glue::list glues;

    for (const auto &id : properties.pendingGlues) {
        if (std::find(id.begin(), id.end(), tilesetId) != id.end()) {
            glues.emplace_back(id, glueId2path(id));
        }
    }

    if (glues.empty()) {
        LOG(info3) << "All glues for tileset <" << tilesetId << "> exist.";
        return;
    }

    Tx tx(root, addOptions.tmp, addOptions.openOptions);

    const auto gds(prepareGlues(glues, openTilesets(tx, properties.tilesets)));

    // generate all glues
    generateGluesImpl(tx, gds, *this, properties, addOptions);
}

void Storage::Detail::generateGlue(const Glue::Id &glueId
                                   , const AddOptions &addOptions)
{
    // (re)load config to have fresh copy when under lock
    if (storageLock) { loadConfig(); }

    switch (properties.glueType(glueId)) {
    case GlueType::valid:
        // An existing glue, we generate it only when asked to.
        if (addOptions.collisionCheck) {
            // check for collision -> fail
            LOGTHROW(err3, std::runtime_error)
                << "Glue <" << utility::join(glueId, ",")
                << "> already exists.";
        } else if (!addOptions.collisionCheck) {
            // do not check for collision -> overwrite
            LOG(info3)
                << "Re-generating existing glue <"
                << utility::join(glueId, ",") << ">.";
        } else {
            // keep existing glue
            LOG(info2)
                << "Glue <" << utility::join(glueId, ",")
                << "> already exists, not touching.";
            return;
        }
        break;

    case GlueType::empty:
        // An existing "empty" glue.
        if (addOptions.collisionCheck) {
            // check for collision -> fail
            LOGTHROW(err3, std::runtime_error)
                << "Glue <" << utility::join(glueId, ",") << "> is empty.";
        } else if (!addOptions.collisionCheck) {
            // do not check for collision -> overwrite
            LOG(info3)
                << "Glue <" << utility::join(glueId, ",")
                << "> is empty, trying to regenerate as instructed.";
        } else {
            // keep existing "empty" glue
            LOG(info3)
                << "Glue <" << utility::join(glueId, ",")
                << "> is empty, not touching.";
            return;
        }
        break;

    case GlueType::pending:
        // A pending glue! Our best friend here!
        break;

    case GlueType::unknown:
        // Completely unknown glue.
        if (addOptions.collisionCheck) {
            LOGTHROW(err3, std::runtime_error)
                << "Glue <" << utility::join(glueId, ",") << "> not found.";
        }

        LOG(info3)
            << "Glue <" << utility::join(glueId, ",") << "> not found"
            << ", ignoring.";
        return;
    }

    Glue::list glues;
    glues.emplace_back(glueId, glueId2path(glueId));

    Tx tx(root, addOptions.tmp, addOptions.openOptions);

    const auto gds
        (prepareGlues(glues, openTilesets(tx, properties.tilesets)));

    // generate all glues
    generateGluesImpl(tx, gds, *this, properties, addOptions);
}

void Storage::Detail::remove(const TilesetIdList &tilesetIds)
{
    // (re)load config to have fresh copy when under lock
    if (storageLock) { loadConfig(); }

    Properties nProperties;
    Glue::map glues;
    VirtualSurface::map virtualSurfaces;
    std::tie(nProperties, glues, virtualSurfaces)
        = removeTilesets(properties, tilesetIds);

    LOG(info3)
        << "Removing tilesets <" << utility::join(tilesetIds, ", ") << ">.";

    properties = nProperties;
    saveConfig();

    // physical removal (failure doesn't corrupt the configuration)
    for (const auto &tilesetId : tilesetIds) {
        auto path(storage_paths::tilesetPath(root, tilesetId));
        LOG(info3) << "Removing tileset <" << tilesetId
                   << "> from " << path << ".";
        rmrf(path);
    }

    for (const auto &item : glues) {
        const auto &glue(item.second);
        auto path(storage_paths::gluePath(root, glue));
        LOG(info3) << "Removing glue <" << utility::join(glue.id, ",")
                   << "> from " << glue.path << ".";
        rmrf(path);
    }

    for (const auto &item : virtualSurfaces) {
        const auto &virtualSurface(item.second);
        auto path(storage_paths::virtualSurfacePath(root, virtualSurface));
        LOG(info3) << "Removing virtual surface <"
                   << utility::join(virtualSurface.id, ",")
                   << "> from " << virtualSurface.path << ".";
        rmrf(path);
    }
}

void Storage::Detail
::createVirtualSurface( const TilesetIdSet &tilesets
                      , const CloneOptions &createOptions)
{
    // (re)load config to have fresh copy when under lock
    if (storageLock) { loadConfig(); }

    auto tmp(tilesets);

    VirtualSurface vs;
    for (const auto &stored : properties.tilesets) {
        auto ftmp(tmp.find(stored.tilesetId));
        if (ftmp == tmp.end()) { continue; }

        vs.id.push_back(stored.tilesetId);
        tmp.erase(ftmp);
    }

    if (!tmp.empty()) {
        LOGTHROW(err1, vtslibs::storage::NoSuchTileSet)
            << "Tileset(S) <" << utility::join(tmp, ", ")
            << "> not found in storage " << root << ".";
    }

    // get existing tileset (NB we have to copy if since pointer is changed by
    // manipulation with the storage properties)
    boost::optional<VirtualSurface> existing;
    if (const auto *e = properties.getVirtualSurface(vs.id)) {
        existing = *e;
    }

    if (existing && (createOptions.mode() == CreateMode::failIfExists)) {
        LOGTHROW(err1, vtslibs::storage::TileSetAlreadyExists)
            << "Virtual surface <" << utility::join(vs.id, ",")
            << "> already present in storage "
            << root << ".";
    }

    LOG(info3) << "Creating virtual surface <" << utility::join(vs.id, ",")
               << "> in storage " << root << ".";

    // use provided tilesetId or build one from contituent tilesets' IDs
    const auto vsSetId
        (createOptions.tilesetId()
         ? *createOptions.tilesetId()
         : boost::lexical_cast<std::string>(utility::join(vs.id, "_")));
    vs.path = vsSetId;

    Tx tx(root, boost::none);

    {
        // lock virtual surface and unlock storage
        ScopedStorageLock vsLock(&storageLock, lockName(vs));

        auto vsPath(tx.addVirtualSurface(vs));

        auto aggCreateOptions = createOptions;
        aggCreateOptions.mode(CreateMode::overwrite);
        aggCreateOptions.tilesetId(vsSetId);
        aggCreateOptions.createFlags(AggregateFlags::dontAbsolutize
                        | AggregateFlags::sourceReferencesInMetatiles);

        vts::aggregateTileSets
            (vsPath, "../..", aggCreateOptions
             , TilesetIdSet(vs.id.begin(), vs.id.end()));
    }

    // virtual surface is unlocked and surface is locked again

    // re-load stored properties (may be changed by another storage access
    // instance)
    properties = readConfig();

    // TODO: resolve conflicts (some virtual surface's tileset has been removed
    // etc)

    // store virtual surface
    properties.virtualSurfaces[vs.id] = vs;

    // commit new properties and changes to the transaction
    saveConfig();
    tx.commit();

    // if virtual surface's path has been changed remove original
    if (existing && (existing->path != vs.path)) {
        const auto path(tx.virtualSurfacePath(*existing));
        LOG(info2) << "commit(rm(" << path << "))";
        rmrf(path);
    }
}

void Storage::Detail
::removeVirtualSurface(const TilesetIdSet &tilesets)
{
    // (re)load config to have fresh copy when under lock
    if (storageLock) { loadConfig(); }

    auto tmp(tilesets);

    VirtualSurface::Id vsId;
    for (const auto &stored : properties.tilesets) {
        auto ftmp(tmp.find(stored.tilesetId));
        if (ftmp == tmp.end()) { continue; }

        vsId.push_back(stored.tilesetId);
        tmp.erase(ftmp);
    }

    if (!tmp.empty()) {
        LOGTHROW(err1, vtslibs::storage::NoSuchTileSet)
            << "Tileset(S) <" << utility::join(tmp, ", ")
            << "> not found in storage " << root << ".";
    }

    Properties nProperties;
    VirtualSurface::map virtualSurfaces;
    std::tie(nProperties, virtualSurfaces)
        = removeVirtualSurfaces(properties, { vsId });

    properties = nProperties;
    saveConfig();

    for (const auto &item : virtualSurfaces) {
        const auto &virtualSurface(item.second);
        auto path(storage_paths::virtualSurfacePath(root, virtualSurface));
        LOG(info3)
            << "Removing virtual Surface " << virtualSurface.path << ".";
        rmrf(path);
    }
}

void Storage::lockStressTest(utility::Runnable &running)
{
    detail().lockStressTest(running);
}

void Storage::Detail::lockStressTest(utility::Runnable &running)
{
    while (running) {
        // storage locked

        // load config, sleep a while and write again
        LOG(info3) << "Loading storage properties.";
        auto properties(readConfig());
        sleep(5);
        LOG(info3) << "Saving storage properties.";
        saveConfig(properties);

        {
            try {
                // !stress locked, storage unlocked
                ScopedStorageLock lock(&storageLock, "!stress");

                // sleep a while
                sleep(10);
            } catch (const StorageComponentLocked&) {
                LOG(warn3) << "Unable to lock \"stress\", skipping.";
            }
        }
    }
}

} } // namespace vtslibs::vts
