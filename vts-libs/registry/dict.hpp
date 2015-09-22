/**
 * \file registry/dict.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vadstena_libs_registry_dict_hpp_included_
#define vadstena_libs_registry_dict_hpp_included_

#include <map>
#include <string>

#include "./dict.hpp"

namespace vadstena { namespace registry {

template <typename T, typename Key = std::string>
class Dictionary
{
private:
    typedef Key key_type;
    typedef std::map<Key, T> map;

public:
    Dictionary() {}

    void set(const key_type &id, const T &value);
    const T* get(const key_type &id, std::nothrow_t) const;
    const T& get(const key_type &id) const;
    bool has(const key_type &id) const;

    inline void add(const T &value) { set(value.id, value); }

    typedef typename map::const_iterator const_iterator;
    const_iterator begin() const { return map_.begin(); }
    const_iterator end() const { return map_.end(); }

private:
    map map_;
};

template <typename T, typename Key>
void Dictionary<T, Key>::set(const key_type &id, const T &value)
{
    map_.insert(typename map::value_type(id, value));
}

template <typename T, typename Key>
const T* Dictionary<T, Key>::get(const key_type &id, std::nothrow_t) const
{
    auto fmap(map_.find(id));
    if (fmap == map_.end()) { return nullptr; }
    return &fmap->second;
}

template <typename T, typename Key>
const T& Dictionary<T, Key>::get(const key_type &id) const
{
    const auto *value(get(id, std::nothrow));
    if (!value) {
        LOGTHROW(err1, storage::KeyError)
            << "<" << id << "> is not known " << T::typeName << ".";
    }
    return *value;
}

template <typename T, typename Key>
bool Dictionary<T, Key>::has(const key_type &id) const
{
    return (map_.find(id) != map_.end());
}

} } // namespace vadstena::registry

#endif // vadstena_libs_registry_dict_hpp_included_
