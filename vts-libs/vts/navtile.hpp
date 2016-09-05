#ifndef vadstena_libs_vts_navtile_hpp
#define vadstena_libs_vts_navtile_hpp

#include <cstdlib>
#include <memory>
#include <istream>

#include <boost/filesystem/path.hpp>

#include "math/geometry_core.hpp"
#include "imgproc/rastermask/quadtree.hpp"

#include "../storage/range.hpp"

#include "./multifile.hpp"

namespace vadstena { namespace vts {

class NavTile {
public:
    typedef std::shared_ptr<NavTile> pointer;
    typedef storage::Range<std::int16_t> HeightRange;
    typedef imgproc::quadtree::RasterMask CoverageMask;

    static const math::Size2i size() { return math::Size2i(256, 256); };

    NavTile() : coverageMask_(size(), CoverageMask::InitMode::FULL) {}

    virtual ~NavTile() {}

    virtual HeightRange heightRange() const = 0;

    CoverageMask& coverageMask() { return coverageMask_; }
    const CoverageMask& coverageMask() const { return coverageMask_; }
    void coverageMask(const CoverageMask &mask);

    void serialize(std::ostream &os) const;

    void serializeNavtileProper(std::ostream &os) const;

    void deserialize(const HeightRange &heightRange
                     , std::istream &is, const boost::filesystem::path &path
                     = "unknown");

    static multifile::Table readTable(std::istream &is
                                      , const boost::filesystem::path &path
                                      = "unknown");

    static constexpr unsigned int imageIndex() { return 0; }

private:
    virtual multifile::Table serialize_impl(std::ostream &os) const = 0;

    virtual void deserialize_impl(const HeightRange &heightRange
                                  , std::istream &is
                                  , const boost::filesystem::path &path
                                  , const multifile::Table &table) = 0;

    CoverageMask coverageMask_;
};

class RawNavTile : public NavTile {
public:
    typedef std::shared_ptr<RawNavTile> pointer;

    RawNavTile() : coverageMask_(size(), CoverageMask::InitMode::FULL) {}

    virtual ~RawNavTile() {}

    virtual HeightRange heightRange() const { return heightRange_; }

    typedef std::vector<unsigned char> Image;

    const Image& get() const { return image_; }

private:
    virtual multifile::Table serialize_impl(std::ostream &os) const;

    virtual void deserialize_impl(const HeightRange &heightRange
                                  , std::istream &is
                                  , const boost::filesystem::path &path
                                  , const multifile::Table &table);

    HeightRange heightRange_;
    Image image_;
    CoverageMask coverageMask_;
};

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_navtile_hpp
