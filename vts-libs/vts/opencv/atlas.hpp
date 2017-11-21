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
#ifndef vtslibs_vts_opencv_atlas_hpp
#define vtslibs_vts_opencv_atlas_hpp

#include <opencv2/core/core.hpp>

#include "../atlas.hpp"

namespace vtslibs { namespace vts { namespace opencv {

class Atlas : public vts::Atlas {
public:
    typedef std::shared_ptr<Atlas> pointer;

    Atlas(int quality = 100) : quality_(quality) {}

    /** Construct atlas from any atlas.
     */
    Atlas(const vts::Atlas &atlas, int textureQuality = 100);

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

    virtual void write_impl(std::ostream &os, std::size_t index) const;

    int quality_;
    Images images_;
};

/** Hybrid atlas that can contain on of: encoded image, decoded image or path to
 *  file on the disk.
 *
 *  Deserialization decodes images.
 *  Serialization re-encodes images from disk.
 */
class HybridAtlas : public vts::Atlas {
private:
    struct Entry;

public:
    typedef std::shared_ptr<HybridAtlas> pointer;

    /** Construct hybrid atlas from any atlas.
     */
    HybridAtlas(const Atlas &atlas, int textureQuality = 100);

    HybridAtlas(const opencv::Atlas &atlas) : quality_(atlas.quality()) {
        append(atlas);
    }

    HybridAtlas(int quality = 100) : quality_(quality) {}

    HybridAtlas(std::size_t count, const RawAtlas &rawAtlas
                , int quality = 100);

    HybridAtlas(std::size_t count, const HybridAtlas &hybridAtlas
                , int quality = 100);

    virtual std::size_t size() const { return entries_.size(); }

    typedef opencv::Atlas::Image Image;
    typedef vts::RawAtlas::Image Raw;

    void add(const Image &image);
    void add(const Raw &raw);
    void add(const boost::filesystem::path &file);

    void set(std::size_t index, const Image &image);
    void set(std::size_t index, const Raw &raw);
    void set(std::size_t index, const boost::filesystem::path &file);

    /** Get image at given index.
     */
    Image get(std::size_t index) const;

    /** Append.
     */
    void append(const HybridAtlas &atlas);
    void append(const opencv::Atlas &atlas);

    Entry entry(std::size_t index) const;
    void add(const Entry &entry);

    /** Duplicates last image.
     */
    void duplicate();

    static Image imageFromRaw(const Raw &raw);
    static Raw rawFromImage(const Image &image, int quality);
    static Image imageFromFile(const boost::filesystem::path &file);

private:
    virtual multifile::Table serialize_impl(std::ostream &os) const;

    virtual void deserialize_impl(std::istream &is
                                  , const boost::filesystem::path &path
                                  , const multifile::Table &table);

    virtual math::Size2 imageSize_impl(std::size_t index) const;

    virtual void write_impl(std::ostream &os, std::size_t index) const;

    int quality_;

    /** "union" of raw data and color image
     *  Either blob is non-empty or image.data is valid
     */
    struct Entry {
        Raw raw;
        Image image;
        boost::filesystem::path file;
        boost::optional<std::size_t> source;

        Entry(const Raw &raw) : raw(raw) {}
        Entry(const Image &image) : image(image) {}
        Entry(const boost::filesystem::path &file) : file(file) {}
        Entry(std::size_t source) : source(source) {}
        Entry() {}
    };

    typedef std::vector<Entry> Entries;
    Entries entries_;
};

} } } // namespace vtslibs::vts::opencv

#endif // vtslibs_vts_opencv_atlas_hpp
