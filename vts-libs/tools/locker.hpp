#ifndef vts_libs_tools_locker_hpp_included
#define vts_libs_tools_locker_hpp_included

#include <string>

#include <boost/optional.hpp>
#include <boost/noncopyable.hpp>
#include <boost/filesystem/path.hpp>

class Lock : boost::noncopyable {
public:
    Lock(const boost::filesystem::path &storage
         , const boost::optional<std::string> &lock);
};

#endif // vts_libs_tools_locker_hpp_included
