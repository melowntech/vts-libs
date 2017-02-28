#ifndef vtslibs_vts_navtile_hpp
#define vtslibs_vts_navtile_hpp

#include <cstdlib>
#include <memory>
#include <istream>

#include <boost/filesystem/path.hpp>

#include "math/geometry_core.hpp"
#include "imgproc/rastermask/quadtree.hpp"

#include "../storage/range.hpp"

#include "./multifile.hpp"

namespace vtslibs { namespace vts {

class NavTile {
public:
    typedef std::shared_ptr<NavTile> pointer;
    typedef storage::Range<std::int16_t> HeightRange;
    typedef imgproc::quadtree::RasterMask CoverageMask;

    static constexpr int binOrder = 8; // bits
    static constexpr int width = (1 << binOrder);
    static constexpr int height = (1 << binOrder);

    static const math::Size2i size() { return math::Size2i(width, height); };

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

    /** Converts coordinates from spatial division system to pixel coordinates
     *  inside navtile.
     */
    static math::Point2 sds2px(const math::Point2 &point
                               , const math::Extents2 &extents);

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

} } // namespace vtslibs::vts

#endif // vtslibs_vts_navtile_hpp
