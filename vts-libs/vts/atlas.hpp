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
#ifndef vtslibs_vts_atlas_hpp
#define vtslibs_vts_atlas_hpp

#include <cstdlib>
#include <memory>
#include <istream>
#include <vector>

#include <boost/any.hpp>
#include <boost/filesystem/path.hpp>

#include "math/geometry_core.hpp"

#include "multifile.hpp"

#include "../storage/streams.hpp"

namespace vtslibs { namespace vts {

struct Mesh;

class Atlas {
public:
    typedef std::shared_ptr<Atlas> pointer;

    Atlas() {}

    virtual ~Atlas() {}

    virtual std::size_t size() const = 0;

    void serialize(std::ostream &os) const;

    void deserialize(std::istream &is
                     , const boost::filesystem::path &path = "unknown");

    /** Returns area of given texture image
     */
    double area(std::size_t index) const;

    /** Returns dimensions of given texture image
     */
    math::Size2 imageSize(std::size_t index) const;

    bool valid(std::size_t index) const { return index < size(); }

    bool empty() const { return !size(); }

    /** Write image at given index to output stream.
     */
    void write(std::ostream &os, std::size_t index) const;

    /** Write image at given index to output file.
     */
    void write(const boost::filesystem::path &file, std::size_t index) const;

    static multifile::Table readTable(std::istream &is
                                      , const boost::filesystem::path &path
                                      = "unknown");

    static multifile::Table readTable(const storage::IStream::pointer &is);

private:
    virtual multifile::Table serialize_impl(std::ostream &os) const = 0;

    virtual void deserialize_impl(std::istream &is
                                  , const boost::filesystem::path &path
                                  , const multifile::Table &table) = 0;

    virtual math::Size2 imageSize_impl(std::size_t index) const = 0;

    virtual void write_impl(std::ostream &os, std::size_t index) const = 0;
};

class RawAtlas : public Atlas {
public:
    typedef std::shared_ptr<RawAtlas> pointer;

    virtual std::size_t size() const { return images_.size(); }

    typedef std::vector<unsigned char> Image;
    typedef std::vector<Image> Images;

    const Image& get(std::size_t index) const { return images_[index]; }

    void add(const Image &image);

    void add(const RawAtlas &other);

    /** Access internal data.
     */
    const Images& get() const { return images_; }

private:
    virtual multifile::Table serialize_impl(std::ostream &os) const;

    virtual void deserialize_impl(std::istream &is
                                  , const boost::filesystem::path &path
                                  , const multifile::Table &table);

    virtual math::Size2 imageSize_impl(std::size_t index) const;

    virtual void write_impl(std::ostream &os, std::size_t index) const;

    Images images_;
};

/** Inpaint atlas.
 *
 *  Inpaint atlas. Fails if `vts-libs` library code is not compiled in.
 */
Atlas::pointer inpaint(const Atlas &atlas, const Mesh &mesh
                       , int textureQuality);

inline double Atlas::area(std::size_t index) const
{
    auto s(imageSize(index));
    return double(s.width) * double(s.height);
}

inline void Atlas::write(std::ostream &os, std::size_t index) const
{
    write_impl(os, index);
}

inline multifile::Table Atlas::readTable(const storage::IStream::pointer &is)
{
    return readTable(*is, is->name());
}

} } // namespace vtslibs::vts

#endif // vtslibs_vts_atlas_hpp
