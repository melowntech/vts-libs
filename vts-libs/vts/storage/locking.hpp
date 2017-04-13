/**
 * \file vts/storage/locking.hpp
 * \author Vaclav Blazek <vaclav.blazek@melown.com>
 *
 * Storage locking support.
 */

#ifndef vtslibs_vts_storage_locking_hpp_included_
#define vtslibs_vts_storage_locking_hpp_included_

#include <string>
#include <memory>

#include <boost/optional.hpp>

namespace vtslibs { namespace vts {

struct LockError : public std::runtime_error {
    LockError(const std::string &msg) : std::runtime_error(msg) {}
};

struct LockedError : public LockError {
    LockedError(const std::string &msg) : LockError(msg) {}
};

struct LockTimedOut : public LockError {
    LockTimedOut(const std::string &msg) : LockError(msg) {}
};

/** Helper for storage/glue locking.
 *
 *  If glueId is empty, whole storage is locked.
 */
class StorageLocker {
public:
    typedef std::shared_ptr<StorageLocker> pointer;

    StorageLocker() {};
    virtual ~StorageLocker() {}

    /** Locks storage (if sublock is empty) or specific entity inside storage
     *  (if sublock is non-empty)
     *
     *  \param sublock sublock name (optional)
     *  \return lock value
     */
    std::string lock(const std::string &sublock = std::string());

    /** Unlocks storage (if glueId is empty) or specific glue (ig glueId is
     *  non-empty)
     *
     *  \param lock lock value
     *  \param sublock sublock name (optional)
     */
    void unlock(const std::string &lock
                , const std::string &sublock = std::string());

private:
    virtual std::string lock_impl(const std::string &sublock) = 0;
    virtual void unlock_impl(const std::string &lock
                             , const std::string &sublock) = 0;
};

class ScopedStorageLock {
public:
    ScopedStorageLock(const StorageLocker::pointer &locker
                      , const std::string &sublock = std::string()
                      , ScopedStorageLock *lockToUnlock = nullptr)
        : locker_(locker), sublock_(sublock)
        , lockToUnlock_(lockToUnlock)
    {
        // lock this lock
        lock();
        // unlock other if set
        if (lockToUnlock_) { lockToUnlock_->unlock(); }
    }

    ~ScopedStorageLock();

    void lock() {
        if (!locker_ || value_) { return; }
        value_ = locker_->lock(sublock_);
    }
    void unlock() {
        if (!locker_ || !value_) { return; }
        locker_->unlock(*value_, sublock_);
        value_ = boost::none;
    }

private:
    StorageLocker::pointer locker_;
    std::string sublock_;
    boost::optional<std::string> value_;
    ScopedStorageLock *lockToUnlock_;
};

} } // namespace vtslibs::vts

#endif // vtslibs_vts_storage_locking_hpp_included_
