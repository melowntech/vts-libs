#ifndef vadstena_libs_storage_tidguard_hpp_included_
#define vadstena_libs_storage_tidguard_hpp_included_

#include "dbglog/dbglog.hpp"

namespace vadstena { namespace storage {

struct TIDGuard {
    TIDGuard(const std::string &id, bool append = false)
        : valid(false), old(dbglog::thread_id())
    {
        if (append) {
            dbglog::thread_id(old + "/" + id);
        } else {
            dbglog::thread_id(id);
        }
        valid = true;
    }
    ~TIDGuard() { pop(); }

    void pop() {
        if (!valid) { return; }
        dbglog::thread_id(old);
        valid = false;
    }

    bool valid;
    const std::string old;
};

} } // namespace vadstena::storage

#endif // vadstena_libs_storage_tidguard_hpp_included_
