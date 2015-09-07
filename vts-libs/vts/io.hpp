#ifndef vadstena_libs_vts_io_hpp_included_
#define vadstena_libs_vts_io_hpp_included_

#include <iostream>
#include <typeinfo>

#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/qi_match.hpp>
#include <boost/spirit/include/qi_match_auto.hpp>

#include "./basetypes.hpp"

namespace vadstena { namespace vts {

// LodLevels

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const LodLevels &ll)
{
    return os << ll.lod << '/' << ll.delta;
}

template<typename CharT, typename Traits>
inline std::basic_istream<CharT, Traits>&
operator>>(std::basic_istream<CharT, Traits> &is, LodLevels &ll)
{
    using boost::spirit::qi::auto_;
    using boost::spirit::qi::omit;
    using boost::spirit::qi::match;

    is >> match((auto_ >> omit['/'] >> auto_)
                , ll.lod, ll.delta);

    return is;
}

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const TileId &tid)
{
    return os << '(' << tid.lod << ", " << tid.x
              << ", " << tid.y << ')';
}

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
dump(std::basic_ostream<CharT, Traits> &os
     , const TileId &tid
     , const std::string &prefix = std::string())
{
    os << prefix << "lod = " << tid.lod << '\n'
       << prefix << "x = " << tid.x << '\n'
       << prefix << "y = " << tid.y << '\n'
        ;

    return os;
}

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
dump(std::basic_ostream<CharT, Traits> &os
     , const LodLevels &ll
     , const std::string &prefix = std::string())
{
    os << prefix << "lod = " << ll.lod << '\n'
       << prefix << "delta = " << ll.delta << '\n'
        ;

    return os;
}

struct AsPosition {
    math::Point3 p;
};

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
dump(std::basic_ostream<CharT, Traits> &os
     , const AsPosition &p
     , const std::string &prefix = std::string())
{
    os << prefix << "x = " << p.p(0) << '\n'
       << prefix << "y = " << p.p(1) << '\n'
       << prefix << "altitude = " << p.p(2) << '\n'
        ;

    return os;
}

struct AsOrientation {
    math::Point3 p;
};

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
dump(std::basic_ostream<CharT, Traits> &os
     , const AsOrientation &p
     , const std::string &prefix = std::string())
{
    os << prefix << "yaw = " << p.p(0) << '\n'
       << prefix << "pitch = " << p.p(1) << '\n'
       << prefix << "roll = " << p.p(2) << '\n'
        ;

    return os;
}

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_io_hpp_included_
