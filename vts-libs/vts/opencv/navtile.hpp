#ifndef vadstena_libs_vts_opencv_navtile_hpp
#define vadstena_libs_vts_opencv_navtile_hpp

#include <opencv2/core/core.hpp>

#include "../navtile.hpp"

namespace vadstena { namespace vts { namespace opencv {

class NavTile : public vts::NavTile {
public:
    NavTile() {}

    virtual void serialize(const storage::OStream::pointer &os) const;

    virtual void deserialize(const storage::IStream::pointer &is);

    typedef cv::Mat Image;

    Image& image() { return image_; }
    const Image& image() const { return image_; }

private:
    Image image_;
};

} } } // namespace vadstena::vts::opencv

#endif // vadstena_libs_vts_opencv_navtile_hpp
