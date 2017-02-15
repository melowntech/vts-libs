#ifndef vtslibs_tilestorage_po_hpp_included_
#define vtslibs_tilestorage_po_hpp_included_

#include <boost/program_options.hpp>

#include "./types.hpp"

namespace vtslibs { namespace tilestorage {

inline void validate(boost::any &v
                     , const std::vector<std::string>& values
                     , Locator*, int)
{
    namespace po = boost::program_options;

    po::validators::check_first_occurrence(v);
    v = boost::any(Locator(po::validators::get_single_string(values)));
}

} } // namespace vtslibs::tilestorage

#endif // vtslibs_tilestorage_io_hpp_included_
