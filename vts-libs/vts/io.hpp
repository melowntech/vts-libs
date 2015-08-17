#ifndef vadstena_libs_vts_io_hpp_included_
#define vadstena_libs_vts_io_hpp_included_

#include <iostream>
#include <typeinfo>

#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/qi_match.hpp>
#include <boost/spirit/include/qi_match_auto.hpp>

#include "utility/enum-io.hpp"

#include "./basetypes.hpp"
#include "./filetypes.hpp"
#include "./properties.hpp"

namespace vadstena { namespace vts {

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const Locator &l)
{
    return os << l.type << ':' << l.location;
}

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

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
dump(std::basic_ostream<CharT, Traits> &os
     , const DriverProperties &p
     , const std::string &prefix = std::string())
{
    os << prefix << "driver:\n";
    const std::string subPrefix(prefix.size() + 4, ' ');
    os << subPrefix << "type = " << p.type << '\n';
    os << subPrefix << "options:";
    const std::string subSubPrefix(subPrefix.size() + 4, ' ');
    for (const auto &vt : p.options) {
        os << "\n" << subSubPrefix << vt.first << " = ";
        const auto &ti(vt.second.type());
        if (ti == typeid(std::int64_t)) {
            os << boost::any_cast<std::int64_t>(vt.second);
        } else if (ti == typeid(std::uint64_t)) {
            os << boost::any_cast<std::uint64_t>(vt.second);
        } else if (ti == typeid(double)) {
            os << boost::any_cast<double>(vt.second);
        } else if (ti == typeid(std::string)) {
            os << '"' << boost::any_cast<std::string>(vt.second) << '"';
        } else if (ti == typeid(bool)) {
            os << std::boolalpha << boost::any_cast<bool>(vt.second);
        } else {
            os << "???";
        }
    }
    os << '\n';
    return os;
}

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
dump(std::basic_ostream<CharT, Traits> &os
     , const StaticProperties &p
     , const std::string &prefix = std::string())
{
    os << prefix << "id = " << p.id << '\n';
    dump(os, p.metaLevels, prefix + "metaLevels.");
    os << prefix << "srs = " << p.srs << '\n';
    dump(os, p.driver, prefix);
    return os;
}

// Properties

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
dump(std::basic_ostream<CharT, Traits> &os
     , const SettableProperties &p
     , const std::string &prefix = std::string())
{
    dump(os, AsPosition{p.defaultPosition}, prefix + "defaultPosition.");
    dump(os, AsOrientation{p.defaultOrientation}
         , prefix + "defaultOrientation.");
    os << prefix << "textureQuality = " << p.textureQuality << '\n'
       << prefix << "texelSize = " << p.texelSize << '\n';

    return os;
}

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
dump(std::basic_ostream<CharT, Traits> &os
     , const Properties &p
     , const std::string &prefix = std::string())
{
    dump(os, static_cast<const StaticProperties&>(p), prefix);

    os << prefix << "hasData = " << p.hasData << '\n'
       << prefix << "meshTemplate = " << p.meshTemplate << '\n'
       << prefix << "textureTemplate = " << p.textureTemplate << '\n'
       << prefix << "metaTemplate = " << p.metaTemplate << '\n'
        ;

    dump(os, static_cast<const SettableProperties&>(p), prefix);

    return os;
}

UTILITY_GENERATE_ENUM_IO(TileFile,
                         ((meta))
                         ((mesh))
                         ((atlas))
                         )

UTILITY_GENERATE_ENUM_IO(File,
                         ((config))
                         ((tileIndex))
                         )

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_io_hpp_included_
