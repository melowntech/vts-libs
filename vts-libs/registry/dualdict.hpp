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

#ifndef vtslibs_registry_dualdict_hpp_included_
#define vtslibs_registry_dualdict_hpp_included_

#include <string>
#include <algorithm>

#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/member.hpp>

#include "dbglog/dbglog.hpp"

namespace vtslibs { namespace registry {

template <typename T, typename NumericId>
class DualDictionary
{
private:
    struct StringIdx {};
    struct NumericIdx {};

    typedef boost::multi_index_container<
        T
        , boost::multi_index::indexed_by<
              boost::multi_index::ordered_unique<
                  boost::multi_index::tag<StringIdx>
                  , BOOST_MULTI_INDEX_MEMBER(T, std::string, id)
                  >
              , boost::multi_index::ordered_non_unique<
                    boost::multi_index::tag<NumericIdx>
                    , BOOST_MULTI_INDEX_MEMBER(T, NumericId, numericId)
                    >
              >
        > map;

public:
    DualDictionary() {}

    void replace(const T &value);

    const T* get(const std::string &id, std::nothrow_t) const;
    const T& get(const std::string &id) const;
    bool has(const std::string &id) const;

    const T* get(NumericId id, std::nothrow_t) const;
    const T& get(NumericId id) const;
    bool has(NumericId id) const;

    inline void add(const T &value) { map_.insert(value); }
    inline void add(const T *value) { if (value) { add(*value); } }

    typedef typename map::value_type value_type;
    typedef typename map::const_iterator const_iterator;
    const_iterator begin() const { return map_.begin(); }
    const_iterator end() const { return map_.end(); }

    void update(const DualDictionary &other);

    template <typename Op> void for_each(Op op)
    {
        std::for_each(map_.begin(), map_.end()
                      , [&](typename map::value_type &item) {
                          op(item);
                      });
    }

    template <typename Op> void for_each(Op op) const
    {
        std::for_each(map_.begin(), map_.end()
                      , [&](const typename map::value_type &item) {
                          op(item);
                      });
    }

    bool empty() const { return map_.empty(); }

    const T* operator()(const std::string &id, std::nothrow_t) const {
        return get(id, std::nothrow);
    }
    const T& operator()(const std::string &id) const { return get(id); }

    const T* operator()(NumericId id, std::nothrow_t) const {
        return get(id, std::nothrow);
    }
    const T& operator()(NumericId id) const { return get(id); }

    bool operator==(const DualDictionary &o) const { return map_ == o.map_; }
    bool operator!=(const DualDictionary &o) const { return map_ != o.map_; }

private:
    map map_;
};

template <typename T, typename NumericId>
void DualDictionary<T, NumericId>::replace(const T &value)
{
    auto res(map_.insert(value));
    if (!res.second) {
        map_.replace(res.first, value);
    }
}

template <typename T, typename NumericId>
const T* DualDictionary<T, NumericId>::get(const std::string &id
                                           , std::nothrow_t) const
{
    auto fmap(map_.find(id));
    if (fmap == map_.end()) { return nullptr; }
    return &*fmap;
}

template <typename T, typename NumericId>
const T& DualDictionary<T, NumericId>::get(const std::string &id) const
{
    const auto *value(get(id, std::nothrow));
    if (!value) {
        LOGTHROW(err1, storage::KeyError)
            << "<" << id << "> is not known " << T::typeName << ".";
    }
    return *value;
}

template <typename T, typename NumericId>
const T* DualDictionary<T, NumericId>::get(NumericId id
                                           , std::nothrow_t) const
{
    const auto &idx(map_.template get<NumericIdx>());
    auto fidx(idx.find(id));
    if (fidx == idx.end()) { return nullptr; }
    return &*fidx;
}

template <typename T, typename NumericId>
const T& DualDictionary<T, NumericId>::get(NumericId id) const
{
    const auto *value(get(id, std::nothrow));
    if (!value) {
        LOGTHROW(err1, storage::KeyError)
            << "<" << id << "> is not known " << T::typeName << ".";
    }
    return *value;
}

template <typename T, typename NumericId>
bool DualDictionary<T, NumericId>::has(const std::string &id) const
{
    return (map_.find(id) != map_.end());
}

template <typename T, typename NumericId>
bool DualDictionary<T, NumericId>::has(NumericId id) const
{
    const auto &idx(map_.template get<NumericIdx>());
    return (idx.find(id) != idx.end());
}

template <typename T, typename NumericId>
void DualDictionary<T, NumericId>::update(const DualDictionary &other)
{
    for (const auto &item : other) { map_.insert(item); }
}

} } // namespace vtslibs::registry

#endif // vtslibs_registry_dualdict_hpp_included_
