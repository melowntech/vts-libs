#ifndef vadstena_libs_vts_opencv_atlas_hpp
#define vadstena_libs_vts_opencv_atlas_hpp

#include <opencv2/core/core.hpp>

#include "../atlas.hpp"

namespace vadstena { namespace vts { namespace opencv {

class Atlas : public vts::Atlas {
public:
    Atlas(int quality) : quality_(quality) {}

    virtual std::size_t size() const { return images_.size(); }

    typedef cv::Mat Image;

    void add(const Image &image) { images_.push_back(image); }

    void set(std::size_t index, const Image &image);

    Image get(std::size_t index) { return images_[index]; }

private:
    virtual Table serialize_impl(std::ostream &os) const;

    virtual void deserialize_impl(std::istream &is, const Table &table);

    virtual std::size_t area(std::size_t index) const;

    int quality_;
    typedef std::vector<Image> Images;
    Images images_;
};

} } } // namespace vadstena::vts::opencv

#endif // vadstena_libs_vts_opencv_atlas_hpp
