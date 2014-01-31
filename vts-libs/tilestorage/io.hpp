#ifndef vadstena_libs_tilestorage_io_hpp_included_
#define vadstena_libs_tilestorage_io_hpp_included_

#include <iostream>

#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/qi_match.hpp>
#include <boost/spirit/include/qi_match_auto.hpp>

#include "../tilestorage.hpp"

namespace vadstena { namespace tilestorage {

// LodLevels

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const LodLevels &ll)
{
    return os << ll.lod << "/" << ll.delta;
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
    return os << '(' << tid.lod << ", " << tid.easting
              << ", " << tid.northing << ')';
}

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
dump(std::basic_ostream<CharT, Traits> &os
     , const TileId &tid
     , const std::string &prefix = std::string())
{
    os << prefix << "lod = " << tid.lod << '\n'
       << prefix << "easting = " << tid.easting << '\n'
       << prefix << "northing = " << tid.northing << '\n'
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

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
dump(std::basic_ostream<CharT, Traits> &os
     , const math::Point3 &p
     , const std::string &prefix = std::string())
{
    os << prefix << "easting = " << p(0) << '\n'
       << prefix << "northing = " << p(1) << '\n'
       << prefix << "altitude = " << p(2) << '\n'
        ;

    return os;
}

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
dump(std::basic_ostream<CharT, Traits> &os
     , const CreateProperties &p
     , const std::string &prefix = std::string())
{
    dump(os, p.metaLevels, prefix + "metaLevels.");
    os << prefix << "baseTileSize = " << p.baseTileSize << '\n';
    return os;
}

// Properties

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
dump(std::basic_ostream<CharT, Traits> &os
     , const SettableProperties &p
     , const std::string &prefix = std::string())
{
    dump(os, p.defaultPosition, prefix + "defaultPosition.");
    dump(os, p.defaultOrientation, prefix + "defaultOrientation.");
    os << prefix << "textureQuality = " << p.textureQuality
       << '\n';

    return os;
}

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
dump(std::basic_ostream<CharT, Traits> &os
     , const Properties &p
     , const std::string &prefix = std::string())
{
    dump(os, static_cast<const CreateProperties&>(p), prefix);

    dump(os, p.foat, prefix + "foat.");
    os << prefix << "foat.size = " << p.foatSize << '\n'
       << prefix << "meshTemplate = " << p.meshTemplate << '\n'
       << prefix << "textureTemplate = " << p.textureTemplate << '\n'
       << prefix << "metaTemplate = " << p.metaTemplate << '\n'
        ;

    dump(os, static_cast<const SettableProperties&>(p), prefix);

    return os;
}

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_io_hpp_included_
