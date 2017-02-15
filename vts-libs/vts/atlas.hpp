#ifndef vtslibs_vts_atlas_hpp
#define vtslibs_vts_atlas_hpp

#include <cstdlib>
#include <memory>
#include <istream>
#include <vector>

#include <boost/filesystem/path.hpp>

#include "math/geometry_core.hpp"

#include "./multifile.hpp"

namespace vtslibs { namespace vts {

class Atlas {
public:
    typedef std::shared_ptr<Atlas> pointer;

    Atlas() {}

    virtual ~Atlas() {}

    virtual std::size_t size() const = 0;

    void serialize(std::ostream &os) const;

    void deserialize(std::istream &is
                     , const boost::filesystem::path &path = "unknown");

    /** Returns area of given texture image
     */
    double area(std::size_t index) const;

    /** Returns dimensions of given texture image
     */
    math::Size2 imageSize(std::size_t index) const;

    bool valid(std::size_t index) const { return index < size(); }

    bool empty() const { return !size(); }

    static multifile::Table readTable(std::istream &is
                                      , const boost::filesystem::path &path
                                      = "unknown");

private:
    virtual multifile::Table serialize_impl(std::ostream &os) const = 0;

    virtual void deserialize_impl(std::istream &is
                                  , const boost::filesystem::path &path
                                  , const multifile::Table &table) = 0;

    virtual math::Size2 imageSize_impl(std::size_t index) const = 0;
};

class RawAtlas : public Atlas {
public:
    typedef std::shared_ptr<RawAtlas> pointer;

    virtual std::size_t size() const { return images_.size(); }

    typedef std::vector<unsigned char> Image;

    const Image& get(std::size_t index) const { return images_[index]; }

    void add(const Image &image);

    void add(const RawAtlas &other);

private:
    virtual multifile::Table serialize_impl(std::ostream &os) const;

    virtual void deserialize_impl(std::istream &is
                                  , const boost::filesystem::path &path
                                  , const multifile::Table &table);

    virtual math::Size2 imageSize_impl(std::size_t index) const;

    typedef std::vector<Image> Images;
    Images images_;
};


// inlines

inline double Atlas::area(std::size_t index) const
{
    auto s(imageSize(index));
    return double(s.width) * double(s.height);
}

} } // namespace vtslibs::vts

#endif // vtslibs_vts_atlas_hpp
