/**
 * \file vts/tileset.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tileset creation/clone options.
 */

#ifndef vadstena_libs_vts_options_hpp_included_
#define vadstena_libs_vts_options_hpp_included_

#include <memory>

#include <boost/optional.hpp>

#include "./basetypes.hpp"

namespace vadstena { namespace vts {

// fwd declaration; include metatile.hpp if MetaNode is needed.
struct MetaNode;

/** Tileset open options.
 *
 *  Available options:
 *      relocate: temporary dataset reloaction; really useful only for URLs
 */
class OpenOptions {
public:
    OpenOptions() {}

    typedef std::map<std::string, std::string> CNames;

    const CNames& cnames() const { return cnames_; }
    OpenOptions& cnames(const CNames &cnames) {
        cnames_ = cnames; return *this;
    }
    void updateCNames(const CNames &cnames) {
        cnames_.insert(cnames.begin(), cnames.end());
    }

private:
    CNames cnames_;
};

/** Tilset clone options. Sometimes used for tileset creation.
 *
 *  Options:
 *    * mode: behaviour in case of existent destination path
 *
 *    * tilesetId: tileset ID to use for created tileset instead of the one from
 *                 source tileset (optional)
 *
 *    * lodRange: limits clone operation to given lod range (optional)
 *
 *    * sameType: generates tileset of same type (i.e. with the same low-level
 *                driver) if set; otherwise creates new plain tilesets and
 *                copies data tile-by-tile (default behaviour)
 *
 *                NB: if driver cannot clone dataset by its own the dataset se
 *                cloned ad plain dataset (i.e. sameType flag is ignored)
 *    * metaNodeManipulator:
 *                allows metanode change during cloning operation
 *
 *    * createFlags:
 *                Generic flags interpreted by lower layers.
 *                See configuration of given tileset/driver.
 *
 *    * openOptions: open time options
 */
class CloneOptions {
public:
    typedef std::function<MetaNode (const MetaNode&)> MetaNodeManipulator;

    CloneOptions()
        : mode_(CreateMode::failIfExists), sameType_(false)
        , encodeFlags_(0x0), createFlags_(0)
    {}

    CreateMode mode() const { return mode_; }
    CloneOptions& mode(CreateMode mode) { mode_ = mode; return *this; }

    boost::optional<std::string> tilesetId() const { return tilesetId_; }
    CloneOptions& tilesetId(boost::optional<std::string> tilesetId) {
        tilesetId_ = tilesetId; return *this;
    }

    boost::optional<LodRange> lodRange() const { return lodRange_; }
    CloneOptions& lodRange(const boost::optional<LodRange> &lodRange) {
        lodRange_ = lodRange; return *this;
    }


    MetaNodeManipulator metaNodeManipulator() const {
        return metaNodeManipulator_;
    };
    CloneOptions&
    metaNodeManipulator(MetaNodeManipulator metaNodeManipulator) {
        metaNodeManipulator_ = metaNodeManipulator; return *this;
    };

    CloneOptions& sameType(bool sameType) {
        sameType_ = sameType; return *this;
    }
    bool sameType() const { return sameType_; }

    /** Encoding flags, using any of these flags can lead to long cloning time
     */
    struct EncodeFlag {
        typedef int value_type;
        enum : value_type {
            mesh = 0x1      // reencode meshes
            , inpaint = 0x2 // inpaint atlas textures
        };
    };

    CloneOptions& encodeFlags(EncodeFlag::value_type encodeFlags) {
        encodeFlags_ = encodeFlags; return *this;
    }
    EncodeFlag::value_type encodeFlags() const { return encodeFlags_; }

    const OpenOptions& openOptions() const { return openOptions_; }
    CloneOptions& openOptions(const OpenOptions &openOptions) {
        openOptions_ = openOptions; return *this;
    }

    typedef std::uint32_t CreateFlags;
    const CreateFlags& createFlags() const { return createFlags_; }
    CloneOptions& createFlags(const CreateFlags &createFlags) {
        createFlags_ = createFlags; return *this;
    }

private:
    CreateMode mode_;
    boost::optional<std::string> tilesetId_;
    boost::optional<LodRange> lodRange_;
    MetaNodeManipulator metaNodeManipulator_;
    bool sameType_;
    EncodeFlag::value_type encodeFlags_;
    OpenOptions openOptions_;
    CreateFlags createFlags_;
};

class RelocateOptions {
public:
    RelocateOptions() : dryRun(false) {}

    struct Result {
        boost::optional<std::string> replacement;
        std::string follow;

        Result(const std::string &follow) : follow(follow) {}
    };

    struct Rule {
        std::string prefix;
        std::string replacement;

        Rule() = default;
        Rule(const std::string &prefix, const std::string &replacement)
            : prefix(prefix), replacement(replacement)
        {}

        typedef std::vector<Rule> list;
    };

    Result apply(const std::string &path) const;

    Rule::list rules;
    bool dryRun;
};

std::istream& operator>>(std::istream &is, RelocateOptions::Rule &rule);

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os
           , const RelocateOptions::Rule &rule)
{
    return os << rule.prefix << '=' << rule.replacement;
}

class MergeProgress {
public:
    typedef std::shared_ptr<MergeProgress> pointer;
    virtual ~MergeProgress() {}

    void expect(std::size_t total);
    void tile();

private:
    virtual void expect_impl(std::size_t total) = 0;
    virtual void tile_impl() = 0;
};

/** Glue creation options.
 */
struct GlueCreationOptions {
    /** Texture quality. JPEG quality 0-100. 0 means no atlas repacking.
     */
    int textureQuality;

    /** Clip meshes based on merge coverage.
     */
    bool clip;

    /** Merge progress reporting.
     *  Pinged with each tile.
     */
    MergeProgress::pointer progress;

    GlueCreationOptions()
        : textureQuality(), clip(true)
    {}
};

// inlines

inline void MergeProgress::expect(std::size_t total)
{
    expect_impl(total);
}

inline void MergeProgress::tile()
{
    tile_impl();
}


} } // namespace vadstena::vts

#endif // vadstena_libs_vts_options_hpp_included_
