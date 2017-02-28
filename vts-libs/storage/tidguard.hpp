#ifndef vtslibs_storage_tidguard_hpp_included_
#define vtslibs_storage_tidguard_hpp_included_

#include "dbglog/dbglog.hpp"

namespace vtslibs { namespace storage {

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

} } // namespace vtslibs::storage

#endif // vtslibs_storage_tidguard_hpp_included_
