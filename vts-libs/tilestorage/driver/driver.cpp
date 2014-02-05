#include "../driver.hpp"

namespace vadstena { namespace tilestorage {

void Driver::wannaWrite(const std::string &what) const
{
    if (readOnly_) {
        LOGTHROW(err2, Error)
            << "Cannot " << what << ": storage is read-only.";
    }
}

} } // namespace vadstena::tilestorage
