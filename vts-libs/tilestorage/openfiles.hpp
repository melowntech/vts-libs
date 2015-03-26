#ifndef vadstena_libs_tilestorage_driver_openfiles_hpp_included_
#define vadstena_libs_tilestorage_driver_openfiles_hpp_included_

#include <atomic>

#include <boost/noncopyable.hpp>

namespace vadstena { namespace tilestorage {

class OpenFiles : boost::noncopyable
{
public:
    static void inc() { ++count_; }
    static void dec() { --count_; }
    static int count() { return count_; }

    static int treshold() { return threshold_; }
    static void treshold(int value) { threshold_ = value; }
    static bool critical() { return count_ >= threshold_; }

private:
    static std::atomic<int> count_;
    static int threshold_;
};

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_driver_openfiles_hpp_included_
