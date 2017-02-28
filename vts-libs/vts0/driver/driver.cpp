#include <mutex>

#include "dbglog/dbglog.hpp"

#include "../../storage/error.hpp"

#include "../io.hpp"
#include "../driver.hpp"
#include "../config.hpp"
#include "./tilardriver.hpp"

namespace vtslibs { namespace vts0 {

namespace fs = boost::filesystem;

void Driver::wannaWrite(const std::string &what) const
{
    if (readOnly_) {
        LOGTHROW(err2, storage::ReadOnlyError)
            << "Cannot " << what << ": storage is read-only.";
    }
}

void Driver::notRunning() const
{
    LOGTHROW(warn2, storage::Interrupted)
        << "Operation has been interrupted.";
}

} } // namespace vtslibs::vts0
