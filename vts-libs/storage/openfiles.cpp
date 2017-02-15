#include <cstdlib>

#include <boost/lexical_cast.hpp>

#include "utility/rlimit.hpp"

#include "./openfiles.hpp"

namespace vtslibs { namespace storage {

namespace {

const char *TILESET_MAX_OPEN_FILES("TILESET_MAX_OPEN_FILES");

int getOpenFilesThreshold()
{
    int count(0);
    if (auto value = std::getenv(TILESET_MAX_OPEN_FILES)) {
        count = boost::lexical_cast<int>(value);
    } else {
        count = utility::maxOpenFiles() / 2;
    }
    return count;
}

} // namespace

std::atomic<int> OpenFiles::count_(0);
int OpenFiles::threshold_(getOpenFilesThreshold());

} } // namespace vtslibs::torage
