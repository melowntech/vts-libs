#ifndef vtslibs_storage_lod_hpp_included_
#define vtslibs_storage_lod_hpp_included_

#include <cstdint>

namespace vtslibs { namespace storage {

// not using std::uint8_t because it is interpreted as char
typedef std::uint16_t Lod;

} } // namespace vtslibs::storage

#endif // vtslibs_storage_lod_hpp_included_

