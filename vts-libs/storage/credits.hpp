/**
 * \file storage/credits.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vadstena_libs_storage_credits_hpp_included_
#define vadstena_libs_storage_credits_hpp_included_

#include <cstdint>

#include "utility/small_set.hpp"

namespace vadstena { namespace storage {

typedef std::uint16_t CreditId;

typedef utility::small_set<CreditId> CreditIds;

} } // namespace vadstena::storage

#endif // vadstena_libs_storage_credits_hpp_included_
