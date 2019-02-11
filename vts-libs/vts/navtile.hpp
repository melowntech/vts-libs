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
#ifndef vtslibs_vts_navtile_hpp
#define vtslibs_vts_navtile_hpp

#include <cstdlib>
#include <memory>
#include <istream>

#include <boost/filesystem/path.hpp>

#include "math/geometry_core.hpp"
#include "imgproc/rastermask/quadtree.hpp"

#include "../storage/streams.hpp"
#include "../storage/range.hpp"

#include "multifile.hpp"

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
    static multifile::Table readTable(const storage::IStream::pointer &is);

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

inline multifile::Table NavTile::readTable(const storage::IStream::pointer &is)
{
    return readTable(*is, is->name());
}

} } // namespace vtslibs::vts

#endif // vtslibs_vts_navtile_hpp
