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
#ifndef vtslibs_vts_multifile_hpp
#define vtslibs_vts_multifile_hpp

#include <cstdint>
#include <cstdlib>
#include <memory>
#include <istream>
#include <vector>

#include <boost/filesystem/path.hpp>

namespace vtslibs { namespace vts { namespace multifile {

struct Table {
    struct Entry {
        std::size_t start;
        std::size_t size;

        Entry(std::size_t start = 0, std::size_t size = 0)
            : start(start), size(size)
        {}

        std::size_t end() const { return start + size; }

        operator bool() const { return size; }

        bool operator==(const Entry &e) const {
            return (start == e.start) && (size == e.size);
        }
    };

    typedef std::vector<Entry> Entries;

    Entries entries;
    std::uint16_t version;
    std::string magic;

    Table(std::uint16_t version = 1, const std::string &magic = "")
        : version(version), magic(magic)
    {}

    Table& set(std::uint16_t version, const std::string &magic) {
        this->version = version;
        this->magic = magic;
        return *this;
    }

    const Table& versionAtMost(std::uint16_t version
                               , const boost::filesystem::path &path) const;

    const Table& checkEntryCount(Entries::size_type count
                                 , const boost::filesystem::path &path) const;

    std::size_t add(std::size_t start, std::size_t size) {
        entries.emplace_back(start, size);
        return entries.back().end();
    }

    std::size_t add(const Entry &entry) {
        entries.push_back(entry);
        return entry.end();
    }

    typedef Entries::const_iterator const_iterator;
    typedef Entries::iterator iterator;
    const_iterator begin() const { return entries.begin(); }
    const_iterator end() const { return entries.end(); }
    const_iterator cbegin() { return entries.begin(); }
    const_iterator cend() { return entries.end(); }
    iterator begin() { return entries.begin(); }
    iterator end() { return entries.end(); }

    Entries::size_type size() const { return entries.size(); }

    bool empty() const { return entries.empty(); }

    const Entry& operator[](Entries::size_type i) const { return entries[i]; }
};

Table readTable(std::istream &is, const std::string &expectMagic
                , const boost::filesystem::path &path
                = "unknown");

void writeTable(const Table &table, std::ostream &os);

} } } // namespace vtslibs::vts::multifile

#endif // vtslibs_vts_multifile_hpp
