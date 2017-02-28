#ifndef vtslibs_storage_openfiles_hpp_included_
#define vtslibs_storage_openfiles_hpp_included_

#include <atomic>

#include <boost/noncopyable.hpp>

namespace vtslibs { namespace storage {

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

} } // namespace vtslibs::storage

#endif // vtslibs_storage_openfiles_hpp_included_
