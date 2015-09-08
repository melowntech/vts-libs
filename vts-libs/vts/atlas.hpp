#ifndef vadstena_libs_vts_atlas_hpp
#define vadstena_libs_vts_atlas_hpp

#include <opencv2/core/core.hpp>

namespace vadstena { namespace vts {

class Atlas {
public:
    typedef cv::Mat Image;

    Atlas() {}

    void addImage(std::size_t index, const Image &image);

    const Image& getImage(std::size_t index) const;

private:
    typedef std::vector<Image> Images;
    Images images;
};

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_atlas_hpp
