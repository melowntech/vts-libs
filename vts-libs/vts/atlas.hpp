#ifndef vadstena_libs_vts_atlas_hpp
#define vadstena_libs_vts_atlas_hpp

#include <cstdlib>
#include <memory>
#include <istream>
#include <vector>

#include <boost/filesystem/path.hpp>

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

    /** Returns area of all given texture image.
     */
    virtual std::size_t area(std::size_t index) const = 0;

    struct Entry {
        std::size_t start;
        std::size_t size;

        Entry(std::size_t start = 0, std::size_t size = 0)
            : start(start), size(size)
        {}
    };

    typedef std::vector<Entry> Table;

    /** Helper function to read table contents from file.
     */
    static Table readTable(std::istream &is
                           , const boost::filesystem::path &path = "unknown");

private:
    virtual Table serialize_impl(std::ostream &os) const = 0;

    virtual void deserialize_impl(std::istream &is, const Table &table) = 0;
};

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_atlas_hpp
