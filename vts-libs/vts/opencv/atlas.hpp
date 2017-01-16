#ifndef vadstena_libs_vts_opencv_atlas_hpp
#define vadstena_libs_vts_opencv_atlas_hpp

#include <opencv2/core/core.hpp>

#include "../atlas.hpp"

namespace vadstena { namespace vts { namespace opencv {

class Atlas : public vts::Atlas {
public:
    typedef std::shared_ptr<Atlas> pointer;

    Atlas(int quality = 100) : quality_(quality) {}

    virtual std::size_t size() const { return images_.size(); }

    typedef cv::Mat Image;
    typedef std::vector<Image> Images;

    void add(const Image &image) { images_.push_back(image); }

    void set(std::size_t index, const Image &image);

    Image get(std::size_t index) const { return images_[index]; }

    Images get() const { return images_; }

    /** Append.
     */
    void append(const Atlas &atlas);

    int quality() const { return quality_; }

private:
    virtual multifile::Table serialize_impl(std::ostream &os) const;

    virtual void deserialize_impl(std::istream &is
                                  , const boost::filesystem::path &path
                                  , const multifile::Table &table);

    virtual math::Size2 imageSize_impl(std::size_t index) const;

    int quality_;
    Images images_;
};

/** Hybrid atlas that can contain both encoded and decoded JPEG data.
 */
class HybridAtlas : public vts::Atlas {
public:
    typedef std::shared_ptr<HybridAtlas> pointer;

    HybridAtlas(const opencv::Atlas &atlas) : quality_(atlas.quality()) {
        append(atlas);
    }

    HybridAtlas(int quality = 100) : quality_(quality) {}

    HybridAtlas(std::size_t count, const RawAtlas &rawAtlas
                , int quality = 100);

    virtual std::size_t size() const { return entries_.size(); }

    typedef opencv::Atlas::Image Image;
    typedef vts::RawAtlas::Image Raw;

    void add(const Image &image);
    void add(const Raw &raw);

    void set(std::size_t index, const Image &image);
    void set(std::size_t index, const Raw &raw);

    /** Get image at given index.
     */
    Image get(std::size_t index) const;

    /** Append.
     */
    void append(const HybridAtlas &atlas);
    void append(const opencv::Atlas &atlas);

    static Image imageFromRaw(const Raw &raw);
    static Raw rawFromImage(const Image &image, int quality);

private:
    virtual multifile::Table serialize_impl(std::ostream &os) const;

    virtual void deserialize_impl(std::istream &is
                                  , const boost::filesystem::path &path
                                  , const multifile::Table &table);

    virtual math::Size2 imageSize_impl(std::size_t index) const;

    int quality_;

    /** "union" of raw data and color image
     *  Either blob is non-empty or image.data is valid
     */
    struct Entry {
        Raw raw;
        Image image;

        Entry(const Raw &raw) : raw(raw) {}
        Entry(const Image &image) : image(image) {}
        Entry() {}
    };

    typedef std::vector<Entry> Entries;
    Entries entries_;
};

} } } // namespace vadstena::vts::opencv

#endif // vadstena_libs_vts_opencv_atlas_hpp
