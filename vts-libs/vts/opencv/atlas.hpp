#ifndef vadstena_libs_vts_opencv_atlas_hpp
#define vadstena_libs_vts_opencv_atlas_hpp

#include <opencv2/core/core.hpp>

#include "../atlas.hpp"

namespace vadstena { namespace vts { namespace opencv {

class Atlas : public vts::Atlas {
public:
    Atlas(int quality = 100) : quality_(quality) {}

    virtual std::size_t size() const { return images_.size(); }

    typedef cv::Mat Image;

    void add(const Image &image) { images_.push_back(image); }

    void set(std::size_t index, const Image &image);

    Image get(std::size_t index) { return images_[index]; }

private:
    virtual multifile::Table serialize_impl(std::ostream &os) const;

    virtual void deserialize_impl(std::istream &is
                                  , const boost::filesystem::path &path
                                  , const multifile::Table &table);

    virtual double area_impl(std::size_t index) const;

    int quality_;
    typedef std::vector<Image> Images;
    Images images_;
};

class RawAtlas : public Atlas {
public:
    virtual std::size_t size() const { return images_.size(); }

    typedef std::vector<unsigned char> Image;

    const Image& get(std::size_t index) const { return images_[index]; }

    void add(const Image &image, int scale = 1);

    void add(const RawAtlas &other, int scale = 1);

private:
    virtual multifile::Table serialize_impl(std::ostream &os) const;

    virtual void deserialize_impl(std::istream &is
                                  , const boost::filesystem::path &path
                                  , const multifile::Table &table);

    virtual double area_impl(std::size_t index) const;

    typedef std::vector<Image> Images;
    Images images_;
};

} } } // namespace vadstena::vts::opencv

#endif // vadstena_libs_vts_opencv_atlas_hpp
