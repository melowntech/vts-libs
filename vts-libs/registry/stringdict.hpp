/**
 * \file registry/stringdict.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vtslibs_registry_stringdict_hpp_included_
#define vtslibs_registry_stringdict_hpp_included_

#include <map>
#include <string>
#include <algorithm>

#include <boost/optional.hpp>

#include "dbglog/dbglog.hpp"

namespace vtslibs { namespace registry {

namespace detail {

template <typename T> const std::string& getId(const T &value) {
    return value.id;
}

template <typename T> const std::string& getId(const T *value) {
    return value->id;
}

template <typename T> const std::string& getId(const boost::optional<T> &value)
{
    return value->id;
}

template <typename T> const std::string& getId(const boost::optional<T> *value)
{
    return (*value)->id;
}

template <typename T> const T& getValue(const T &value) {
    return value;
}

template <typename T> const T& getValue(const T *value) {
    return *value;
}

} // namespace detail

template <typename T>
class StringDictionary
{
private:
    typedef std::map<std::string, T> map;

public:
    StringDictionary() {}

    void set(const std::string &id, const T &value);
    void replace(const std::string &id, const T &value);
    const T* get(const std::string &id, std::nothrow_t) const;
    const T& get(const std::string &id) const;
    bool has(const std::string &id) const;

    template <typename U>
    inline void add(const U &value) {
        set(detail::getId(value), detail::getValue(value));
    }

    template <typename U>
    inline void replace(const U &value) {
        replace(detail::getId(value), detail::getValue(value));
    }

    typedef typename map::value_type value_type;
    typedef typename map::const_iterator const_iterator;
    const_iterator begin() const { return map_.begin(); }
    const_iterator end() const { return map_.end(); }

    void update(const StringDictionary &other);

    template <typename Op> void for_each(Op op)
    {
        std::for_each(map_.begin(), map_.end()
                      , [&](typename map::value_type &pair) {
                          op(pair.second);
                      });
    }

    template <typename Op> void for_each(Op op) const
    {
        std::for_each(map_.begin(), map_.end()
                      , [&](const typename map::value_type &pair) {
                          op(pair.second);
                      });
    }

    bool empty() const { return map_.empty(); }

    const T* operator()(const std::string &id, std::nothrow_t) const {
        return get(id, std::nothrow);
    }
    const T& operator()(const std::string &id) const { return get(id); }

private:
    map map_;
};

template <typename T>
void StringDictionary<T>::set(const std::string &id, const T &value)
{
    map_.insert(typename map::value_type(id, value));
}

template <typename T>
void StringDictionary<T>::replace(const std::string &id, const T &value)
{
    auto res(map_.insert(typename map::value_type(id, value)));
    if (!res.second) {
        // already present, replace
        res.first->second = value;
    }
}

template <typename T>
const T* StringDictionary<T>::get(const std::string &id, std::nothrow_t) const
{
    auto fmap(map_.find(id));
    if (fmap == map_.end()) { return nullptr; }
    return &fmap->second;
}

template <typename T>
const T& StringDictionary<T>::get(const std::string &id) const
{
    const auto *value(get(id, std::nothrow));
    if (!value) {
        LOGTHROW(err1, storage::KeyError)
            << "<" << id << "> is not known " << T::typeName << ".";
    }
    return *value;
}

template <typename T>
bool StringDictionary<T>::has(const std::string &id) const
{
    return (map_.find(id) != map_.end());
}

template <typename T>
void StringDictionary<T>::update(const StringDictionary &other)
{
    for (const auto &item : other) { map_.insert(item); }
}

} } // namespace vtslibs::registry

#endif // vtslibs_registry_stringdict_hpp_included_
