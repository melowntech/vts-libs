#ifndef vadstena_libs_vts_atlas_hpp
#define vadstena_libs_vts_atlas_hpp

#include <cstdlib>
#include <memory>
#include <istream>
#include <vector>

#include <boost/filesystem/path.hpp>

#include "./multifile.hpp"

namespace vadstena { namespace vts {

class Atlas {
public:
    typedef std::shared_ptr<Atlas> pointer;

    Atlas() {}

    virtual ~Atlas() {}

    virtual std::size_t size() const = 0;

    void serialize(std::ostream &os) const;

    void deserialize(std::istream &is
                     , const boost::filesystem::path &path = "unknown");

    /** Returns area of all given texture image (apparentResolution applied)
     */
    double area(std::size_t index) const;

    struct Properties {
        /** Apparent area of one pixel, defaults to 1.
         */
        double apparentPixelArea;

        Properties() : apparentPixelArea(1.0) {}

        typedef std::vector<Properties> list;
    };

    void properties(std::size_t index, const Properties &properties);

    Properties properties(std::size_t index) const;

    bool valid(std::size_t index) const { return index < size(); }

    struct Table : multifile::Table {
        Entry properties;

        struct Decompose {};
        Table(const multifile::Table &src);
    };

    static Table readTable(std::istream &is
                           , const boost::filesystem::path &path
                           = "unknown");

private:
    virtual multifile::Table serialize_impl(std::ostream &os) const = 0;

    virtual void deserialize_impl(std::istream &is
                                  , const boost::filesystem::path &path
                                  , const multifile::Table &table) = 0;

    virtual double area_impl(std::size_t index) const = 0;

    Properties::list properties_;
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

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_atlas_hpp
