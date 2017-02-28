/**
 * \file storage/credits.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vtslibs_storage_credits_hpp_included_
#define vtslibs_storage_credits_hpp_included_

#include <cstdint>

#include "utility/small_set.hpp"

namespace vtslibs { namespace storage {

typedef std::uint16_t CreditId;

typedef utility::small_set<CreditId> CreditIds;

} } // namespace vtslibs::storage

#endif // vtslibs_storage_credits_hpp_included_
