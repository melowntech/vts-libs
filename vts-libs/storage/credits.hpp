/**
 * \file storage/credits.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vadstena_libs_storage_credits_hpp_included_
#define vadstena_libs_storage_credits_hpp_included_

#include <cstdint>
#include <vector>

namespace vadstena { namespace storage {

typedef std::uint16_t CreditId;

typedef std::vector<CreditId> CreditIds;

} } // namespace vadstena::storage

#endif // vadstena_libs_storage_credits_hpp_included_
