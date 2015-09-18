#ifndef vadstena_libs_storage_range_hpp_included_
#define vadstena_libs_storage_range_hpp_included_

#include <boost/iterator/counting_iterator.hpp>
#include <boost/iterator/reverse_iterator.hpp>
#include <boost/range/iterator_range.hpp>

#include "utility/streams.hpp"

#include "./lod.hpp"

namespace vadstena { namespace storage {


//! Helper range class for use on the command line or in configs (e.g., "10,25")
template<typename T>
struct Range
{
    T min, max;

    Range() : min(), max() {}
    Range(T l) : min(l), max(l) {}
    Range(T min, T max) : min(min), max(max) {}

    T size() const { return max - min; }

    bool empty() const { return max < min; }

    typedef boost::counting_iterator<T> const_iterator;

    const_iterator begin() const { return const_iterator(min); }
    const_iterator cbegin() const { return const_iterator(min); }

    const_iterator end() const { return const_iterator(max + 1); }
    const_iterator cend() const { return const_iterator(max + 1); }

    bool operator==(const Range &o) const {
        return (min == o.min) && (max == o.max);
    }

    bool operator!=(const Range &o) const { return !operator==(o); }

    static Range emptyRange() { return { T(1), T(0) }; }
};

typedef Range<Lod> LodRange;

template<typename T>
inline Range<T> range(const T min, const T max)
{
    return { min, max };
}

template<typename T>
inline Range<T> unite(const Range<T> &l, const Range<T> &r)
{
    if (l.empty()) { return r; }
    if (r.empty()) { return l; }
    return { std::min(l.min, r.min), std::max(l.max, r.max) };
}

template<typename T>
inline void update(Range<T> &r, const T &v)
{
    if (r.empty()) { r = { v }; }
    if (v < r.min) { r.min = v; }
    if (v > r.max) { r.max = v; }
}

template<typename T>
inline bool in(const T &value, const Range<T> &range)
{
    return (value >= range.min) && (value <= range.max);
}

template<typename CharT, typename Traits, typename RangeT>
inline std::basic_istream<CharT, Traits>&
operator>>(std::basic_istream<CharT, Traits> &is,
           Range<RangeT> &range)
{
    RangeT min, max;
    is >> min >> utility::expect(',') >> max;
    if (!is.fail()) { range.min = min; range.max = max; }
    return is;
}


template <typename E, typename T, typename RangeT>
std::basic_ostream<E, T>&
operator<<(std::basic_ostream<E,T> &os,
           const Range<RangeT> &range)
{
    os << range.min << "," << range.max;
    return os;
}


} } // namesapce vadstena::storage

#endif // vadstena_libs_storage_range_hpp_included_
