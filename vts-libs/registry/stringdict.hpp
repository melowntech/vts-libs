/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
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
    typedef typename map::iterator iterator;
    const_iterator begin() const { return map_.begin(); }
    const_iterator end() const { return map_.end(); }
    iterator begin() { return map_.begin(); }
    iterator end() { return map_.end(); }

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

    bool operator==(const StringDictionary &o) const { return map_ == o.map_; }
    bool operator!=(const StringDictionary &o) const { return map_ != o.map_; }

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
