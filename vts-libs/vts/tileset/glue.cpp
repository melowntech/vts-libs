#include <boost/format.hpp>

#include "utility/progress.hpp"
#include "utility/path.hpp"

#include "../io.hpp"

#include "./detail.hpp"
#include "./merge.hpp"

namespace fs = boost::filesystem;

namespace vadstena { namespace vts {

namespace {

inline std::vector<TileIndex> tileIndices(const TileSet::const_ptrlist &sets
                                          , const LodRange &lodRange)
{
    std::vector<TileIndex> indices;
    for (const auto ts : sets) {
        indices.push_back
            (ts->tileIndex(lodRange).simplify(TileIndex::Flag::mesh));
    }
    return indices;
}

inline TileIndices tileIndices(const std::vector<TileIndex> &tis
                               , std::size_t size
                               = std::numeric_limits<std::size_t>::max())
{
    size = std::min(size, tis.size());

    TileIndices indices;
    for (const auto &ti : tis) {
        indices.push_back(&ti);
    }

    return indices;
}

LodRange range(const TileSet::const_ptrlist &sets)
{
    // start with invalid range
    LodRange r(LodRange::emptyRange());

    if (sets.empty()) { return r; }

    for (const auto &set : sets) {
        r = unite(r, set->lodRange());
    }
    return r;
}

const char *TILEINDEX_DUMP_ROOT("TILEINDEX_DUMP_ROOT");

const char* getDumpDir()
{
    return std::getenv(TILEINDEX_DUMP_ROOT);
}

void dumpTileIndex(const char *root, const fs::path &name
                   , const TileIndex &index
                   , const std::string &referenceFrame)
{
    if (!root) { return; }
    LOG(info1) << "Dumping tileindex " << name << ".";
    auto filename(root / utility::addExtension(name, ".index"));
    create_directories(filename.parent_path());
    index.save(filename);

    auto rfFilename(root / utility::addExtension(name, ".rframe"));
    utility::write(rfFilename, referenceFrame.data(), referenceFrame.size());
}

inline void dump(const char *root, const boost::filesystem::path &dir
                 , const std::vector<TileIndex> &tileIndices
                 , const std::string &referenceFrame)
{
    if (!root) { return; }

    int i(0);
    for (const auto &ti : tileIndices) {
        dumpTileIndex(root, dir / str(boost::format("%03d") % i), ti
                      , referenceFrame);
        ++i;
    }
}

typedef std::vector<const TileSet::Detail*> DetailList;

DetailList details(TileSet::Detail &detail
                   , const TileSet::const_ptrlist &sets)
{
    DetailList out;
    for (const auto *set : sets) {
        out.push_back(&detail.other(*set));
    }
    return out;
}

struct Merger {
public:
    Merger(TileSet::Detail &self, const TileIndex &generate
           , const TileSet::const_ptrlist &srcSets)
        : self(self), world(generate), generate(generate)
        , src(details(self, srcSets)), top(*src.back())
        , topId(src.size() - 1), progress(generate.count())
    {
        // make world complete
        world.complete();
    }

    void operator()() {
        mergeTile(NodeInfo(self.referenceFrame));
    }

private:
    struct Constraints : merge::MergeConstraints {
        Constraints(const Merger &merger, bool generable)
            : merge::MergeConstraints(generable), merger(merger)
        {}
        virtual bool feasible(const merge::Output &result) const {
            return merger.isGlueTile(result);
        }
        const Merger &merger;
    };
    friend struct Constraints;

    /** Merge subtree starting at index.
     *  Calls itself recursively.
     */
    void mergeTile(const NodeInfo &nodeInfo
                   , const TileId &tileId = TileId()
                   , const merge::Input::list &parentSource
                   = merge::Input::list());

    /** Generates new tile as a merge of tiles from other tilesets.
     */
    merge::Output processTile(const NodeInfo &nodeInfo
                              , const TileId &tileId
                              , const merge::Input::list &parentSource
                              , const Constraints &constraints);

    bool isGlueTile(const merge::Output &tile) const;

    TileSet::Detail &self;
    TileIndex world;
    const TileIndex &generate;
    const DetailList src;
    const TileSet::Detail &top;
    const merge::Input::Id topId;

    utility::Progress progress;
};

struct TIDGuard {
    TIDGuard(const std::string &id, bool append = false)
        : valid(false), old(dbglog::thread_id())
    {
        if (append) {
            dbglog::thread_id(old + "/" + id);
        } else {
            dbglog::thread_id(id);
        }
        valid = true;
    }
    ~TIDGuard() { pop(); }

    void pop() {
        if (!valid) { return; }
        dbglog::thread_id(old);
        valid = false;
    }

    bool valid;
    const std::string old;
};

inline bool Merger::isGlueTile(const merge::Output &tile) const
{
    // TODO: make better

    // tile that is fully covered by top set -> not a glue tile
    if (top.fullyCovered(tile.tileId)) {
        LOG(info1) << "Fully covered by top set.";
        return false;
    }

    if (tile.source.size() == src.size()) {
        // merged from all input sets -> glue
        return true;
    }

    // check multisource
    for (const auto &source : tile.source) {
        if (source.tileId().lod != tile.tileId.lod) {
            // fallback used -> glue tile
            return true;
        }

        if (source.id() == topId) {
            // tile generated from top set -> glue tile
            return true;
        }
    }

    return false;
}

void Merger::mergeTile(const NodeInfo &nodeInfo, const TileId &tileId
                       , const merge::Input::list &parentSource)
{
    if (!world.exists(tileId)) {
        // no data below
        return;
    }

    TIDGuard tg(str(boost::format("tile:%s") % tileId), true);

    bool g(generate.exists(tileId));
    bool atBottom(tileId.lod >= generate.maxLod());

    LOG(info2) << "(glue) Processing tile " << tileId << ".";

    // terminate descent if at bottom and nothing is going to be generated
    if (atBottom && !g) { return; }

    // process tile
    auto tile(processTile(nodeInfo, tileId, parentSource
                          , Constraints(*this, g)));

    if (tile) {
        self.setTile(tileId, tile.getMesh(), tile.getAtlas()
                     , tile.getNavtile(), &nodeInfo);
    }

    if (g) {
        (++progress).report(utility::Progress::ratio_t(5, 1000), "(glue) ");
    }

    // do not descent if we are at the bottom
    if (atBottom) { return; }

    tg.pop();

    // OK, process children
    for (const auto &child : children(tileId)) {
        mergeTile(nodeInfo.child(child), child, tile.source);
    }
}

merge::Output Merger::processTile(const NodeInfo &nodeInfo
                                  , const TileId &tileId
                                  , const merge::Input::list &parentSource
                                  , const Constraints &constraints)
{
    merge::Input::list input;
    {
        merge::Input::Id id(0);
        for (const auto *ts : src) {
            merge::Input t(id++, *ts, tileId, nodeInfo);
            if (t) { input.push_back(t); }
        }
    }
    return merge::mergeTile(tileId, nodeInfo, input, parentSource
                            , constraints);
}

} // namespace

void TileSet::createGlue(const const_ptrlist &sets)
{
    LOG(info3) << "(glue) Calculating generate set.";

    const auto *dumpRoot(getDumpDir());
    const auto &referenceFrame(getProperties().referenceFrame);

    // make update (last set) round (all tiles have siblings)
    auto update(sets.back()->tileIndex());
    update.simplify(TileIndex::Flag::mesh).makeQuadComplete();
    dumpTileIndex(dumpRoot, "roundUpdate", update, referenceFrame);

    auto generate(update);
    dumpTileIndex(dumpRoot, "generate", generate, referenceFrame);
    LOG(info1) << "generate: " << generate.count();

    if (generate.empty()) {
        LOG(warn3) << "(glue) Nothing to generate. Bailing out.";
        return;
    }

    Merger(detail(), generate, sets)();
    setPosition(sets.back()->getProperties().position);
}

} } // namespace vadstena::vts
