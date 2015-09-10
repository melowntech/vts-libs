#ifndef vadstena_libs_vts_opencvatlas_hpp
#define vadstena_libs_vts_opencvatlas_hpp

#include <opencv2/core/core.hpp>

#include "./atlas.hpp"

namespace vadstena { namespace vts {

class OpenCvAtlas : public Atlas {
public:
    OpenCvAtlas() {}

    virtual std::size_t count() const;

    virtual void serialize(const storage::OStream::pointer &os
                           , std::size_t index
                           , int textureQuality) const;

    virtual void deserialize(const storage::IStream::pointer &is
                             , std::size_t index);

    typedef cv::Mat Image;

    void add(const Image &image) { images_.push_back(image); }

    void set(std::size_t index, const Image &image);

private:
    std::vector<Image> images_;
};

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_opencvatlas_hpp
