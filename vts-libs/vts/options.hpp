/**
 * \file vts/tileset.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tileset creation/clone options.
 */

#ifndef vadstena_libs_vts_options_hpp_included_
#define vadstena_libs_vts_options_hpp_included_

#include <boost/optional.hpp>

#include "./basetypes.hpp"

namespace vadstena { namespace vts {

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
 */
class CloneOptions {
public:
    CloneOptions()
        : mode_(CreateMode::failIfExists), sameType_(false)
    {}

    CreateMode mode() const { return mode_; }
    CloneOptions& mode(CreateMode mode) { mode_ = mode; return *this; }

    boost::optional<std::string> tilesetId() const {
        return tilesetId_;
    }

    CloneOptions& tilesetId(boost::optional<std::string> tilesetId) {
        tilesetId_ = tilesetId; return *this;
    }

    boost::optional<LodRange> lodRange() const { return lodRange_; }

    CloneOptions& lodRange(const boost::optional<LodRange> &lodRange) {
        lodRange_ = lodRange; return *this;
    }

    CloneOptions& sameType(bool sameType) {
        sameType_ = sameType; return *this;
    }

    bool sameType() const { return sameType_; }

private:
    CreateMode mode_;
    boost::optional<std::string> tilesetId_;
    boost::optional<LodRange> lodRange_;
    bool sameType_;
};

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_options_hpp_included_
