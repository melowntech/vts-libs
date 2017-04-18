#include <thread>
#include <chrono>

#include "dbglog/dbglog.hpp"

#include "./locking.hpp"

namespace vtslibs { namespace vts {

ScopedStorageLock::~ScopedStorageLock()
{
    // unlock this
    try {
        // lock other if set
        if (lockToUnlock_) { lockToUnlock_->lock(); }
        unlock();
    } catch (const std::exception &e) {
        LOG(fatal) << "Unable to unlock storage lock, bailing out. "
            "Error was: <" << e.what() << ">.";
        std::abort();
    } catch (...) {
        LOG(fatal) << "Unable to unlock storage lock, bailing out. "
            "Error is unknown.";
        std::abort();
    }
}

std::string StorageLocker::lock(const std::string &sublock)
{
    for (;;) {
        try {
            return lock_impl(sublock);
        } catch (const StorageLocked&) {
            // storage locked -> retry
            // glue locked -> boom
        }
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
}

void StorageLocker::unlock(const std::string &lock
                           , const std::string &sublock)
{
     unlock_impl(lock, sublock);
}

} } // namespace vtslibs::vts
