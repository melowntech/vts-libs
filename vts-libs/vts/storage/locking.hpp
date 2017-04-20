/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
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

struct StorageLocked : public LockError {
    StorageLocked(const std::string &msg) : LockError(msg) {}
};

struct StorageComponentLocked : public LockError {
    StorageComponentLocked(const std::string &msg) : LockError(msg) {}
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
    // no locking at all
    ScopedStorageLock() : lockToUnlock_() {}

    /** Lock given (sub)lock.
     */
    explicit ScopedStorageLock(const StorageLocker::pointer &locker
                               , const std::string &sublock = std::string())
        : locker_(locker), sublock_(sublock), lockToUnlock_()
    {
        // lock this lock
        lock();
    }

    /** Lock given (sub)lock and unlock provided other scoped lock.
     *
     *  Other scoped lock is locked again before this lock is unlocked.
     */
    explicit ScopedStorageLock(ScopedStorageLock *other
                               , const std::string &sublock = std::string())
        : locker_(other->locker_), sublock_(sublock)
        , lockToUnlock_(other)
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

    /** Returns true if locking is available.
     */
    operator bool() const { return locker_.get(); }

private:
    StorageLocker::pointer locker_;
    std::string sublock_;
    boost::optional<std::string> value_;
    ScopedStorageLock *lockToUnlock_;
};

} } // namespace vtslibs::vts

#endif // vtslibs_vts_storage_locking_hpp_included_
