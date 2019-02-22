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
 * \file vts/tileset.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tileset creation/clone options.
 */

#ifndef vtslibs_vts_options_hpp_included_
#define vtslibs_vts_options_hpp_included_

#include <memory>
#include <iostream>

#include <boost/optional.hpp>
#include <boost/program_options.hpp>

#include "basetypes.hpp"
#include "tileindex.hpp"
#include "metatile.hpp"

// forward declaration
namespace utility { class ResourceFetcher; }

namespace vtslibs { namespace vts {

/** Tileset open options.
 *
 *  Available options:
 *      relocate: temporary dataset reloaction; really useful only for URLs
 */
class OpenOptions {
public:
    OpenOptions()
        : ioRetries_(-1) // infinity
        , ioRetryDelay_(1000) // 1000 ms
        , ioWait_(-1) // infinity
        , scarceMemory_(false)
    {}

    typedef std::map<std::string, std::string> CNames;

    const CNames& cnames() const { return cnames_; }
    OpenOptions& cnames(const CNames &cnames) {
        cnames_ = cnames; return *this;
    }
    void updateCNames(const CNames &cnames) {
        cnames_.insert(cnames.begin(), cnames.end());
    }

    int ioRetries() const { return ioRetries_; }
    OpenOptions& ioRetries(int ioRetries) {
        ioRetries_ = ioRetries; return *this;
    }

    unsigned long ioRetryDelay() const { return ioRetryDelay_; }
    OpenOptions& ioRetryDelay(unsigned long ioRetryDelay) {
        ioRetryDelay_ = ioRetryDelay; return *this;
    }

    long ioWait() const { return ioWait_; }
    OpenOptions& ioWait(long ioWait) {
        ioWait_ = ioWait; return *this;
    }

    bool scarceMemory() const { return scarceMemory_; }
    OpenOptions& scarceMemory(bool scarceMemory) {
        scarceMemory_ = scarceMemory; return *this;
    }

    const std::shared_ptr<utility::ResourceFetcher>& resourceFetcher() const {
        return resourceFetcher_;
    }

    OpenOptions&
    resourceFetcher(const std::shared_ptr<utility::ResourceFetcher> &f) {
        resourceFetcher_ = f; return *this;
    }

    void configuration(boost::program_options::options_description &od
                       , const std::string &prefix = "");

    void configure(const boost::program_options::variables_map &vars
                   , const std::string &prefix = "");

    std::ostream& dump(std::ostream &os, const std::string &prefix = "") const;

private:
    /** Common name mimicing. Interpreted by remote driver.
     */
    CNames cnames_;

    /** Number of IO operation reties. Interpreted by remote driver.
     */
    int ioRetries_;

    /** Delay between individual retries.
     */
    unsigned long ioRetryDelay_;

    /** Timeout in ms for IO operations. Interpreted by remote driver.
     */
    long ioWait_;

    /** Optional resource fetcher. Interpreted by remote driver.
     *  If not set, internal on-demand fetcher is used.
     */
    std::shared_ptr<utility::ResourceFetcher> resourceFetcher_;

    /** We are (or do not want to be) running out of memory.
     */
    bool scarceMemory_;
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
 *                NB: if driver cannot clone dataset by its own the dataset is
 *                cloned aa plain dataset (i.e. sameType flag is ignored)
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
            /** reencode metanode from other data
             *  exact meaning is dependent on metanode version
             *
             *  v*->v4: generate geomExtents from mesh/navtiles(surrogate)
             */
            , meta = 0x4
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

    int textureQuality() const { return textureQuality_; }
    CloneOptions& textureQuality(int value) {
        textureQuality_ = value; return *this;
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

    /** Texture quality (used only when inpainting)
     */
    int textureQuality_;
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

/** Skirt mode
 */
UTILITY_GENERATE_ENUM(SkirtMode,
                      // do not generate any skirt
                      ((none))
                      // stretch skirt to minimum value at given tile point
                      ((minimum))
                      // stretch skirt to maximum value at given tile point
                      ((maximum))
                      // stretch skirt to average value at given tile point
                      ((average))
                      )

/** How to generate glue tile
 */
UTILITY_GENERATE_ENUM(GlueMode,
                      // compose meshes together
                      ((compose))
                      // clip by coverate map mask
                      ((simpleClip))
                      // clip by coverage map contours
                      ((coverageContour))
                      )

UTILITY_GENERATE_ENUM(ContourSimplification,
                      // no simplification at all
                      ((none))
                      // simplify straight segments
                      ((straight))
                      // use Ramer–Douglas–Peucker algorithm
                      ((rdp))
                      )

struct MergeOptions {
    GlueMode glueMode;

    /** Skirt generation mode. Taken into the account by other glue modes than
     *  GlueMode::compose.
     */
    SkirtMode skirtMode;

    /** Scale computer skirt by provided factor.
     */
    double skirtScale;

    /** Safety margin around tile (in mesh coverage mask pixels).
     */
    int safetyMargin;

    /** Contour simplification (used by GlueMode::coverageContour glue mode).
     */
    ContourSimplification contourSimplification;

    /** Maximum segment error for ContourSimplification::rpd.
     */
    double rdpMaxError;

    MergeOptions()
        : glueMode(GlueMode::simpleClip)
        , skirtMode(SkirtMode::none)
        , skirtScale(1.0)
        , safetyMargin(1)
        , contourSimplification(ContourSimplification::rdp)
        , rdpMaxError(0.9)
    {}
};

/** Glue creation options.
 */
struct GlueCreationOptions : MergeOptions {
    /** Texture quality. JPEG quality 0-100. 0 means no atlas repacking.
     */
    int textureQuality;

    /** Merge progress reporting.
     *  Pinged with each tile.
     */
    MergeProgress::pointer progress;

    /** Tileindex is passed to this function where it can be manipulated
     * just before returning it from buildGenerateSet
     */
    typedef std::function<void(vts::TileIndex&)> GenerateSetManipulator;
    GenerateSetManipulator generateSetManipulator;

    GlueCreationOptions()
        : textureQuality()
    {}
};

class ReencodeOptions {
public:
    ReencodeOptions()
        : encodeFlags(), dryRun(false), cleanup(false)
        , descend(true)
    {}

    CloneOptions::EncodeFlag::value_type encodeFlags;
    bool dryRun;
    bool cleanup;
    std::string tag;
    bool descend;
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


} } // namespace vtslibs::vts

#endif // vtslibs_vts_options_hpp_included_
