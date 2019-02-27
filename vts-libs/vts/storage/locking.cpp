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
#include <thread>
#include <chrono>

#include "dbglog/dbglog.hpp"

#include "locking.hpp"

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
