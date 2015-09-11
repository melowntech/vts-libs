#ifndef vadstena_libs_vts_opencv_atlas_hpp
#define vadstena_libs_vts_opencv_atlas_hpp

#include <opencv2/core/core.hpp>

#include "../atlas.hpp"

namespace vadstena { namespace vts { namespace opencv {

class Atlas : public vts::Atlas {
public:
    Atlas(int quality) : quality_(quality) {}

    virtual std::size_t count() const { return images_.size(); }

    virtual void serialize(const storage::OStream::pointer &os
                           , std::size_t index) const;

    virtual void deserialize(const storage::IStream::pointer &is
                             , std::size_t index);

    typedef cv::Mat Image;

    void add(const Image &image) { images_.push_back(image); }

    void set(std::size_t index, const Image &image);

private:
    int quality_;
    std::vector<Image> images_;
};

} } } // namespace vadstena::vts::opencv

#endif // vadstena_libs_vts_opencv_atlas_hpp
