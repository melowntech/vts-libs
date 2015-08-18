#include <mutex>

#include "dbglog/dbglog.hpp"

#include "../io.hpp"
#include "../driver.hpp"
#include "../error.hpp"
#include "./tilardriver.hpp"
#include "../config.hpp"

namespace vadstena { namespace vts {

namespace fs = boost::filesystem;

void Driver::wannaWrite(const std::string &what) const
{
    if (readOnly_) {
        LOGTHROW(err2, ReadOnlyError)
            << "Cannot " << what << ": storage is read-only.";
    }
}

void Driver::notRunning() const
{
    LOGTHROW(warn2, Interrupted)
        << "Operation has been interrupted.";
}

} } // namespace vadstena::vts
