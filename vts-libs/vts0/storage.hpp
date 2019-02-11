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
 * \file vts/storage.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Store.
 */

#ifndef vtslibs_vts0_storage_hpp_included_
#define vtslibs_vts0_storage_hpp_included_

#include <memory>
#include <map>

#include <boost/noncopyable.hpp>
#include <boost/filesystem/path.hpp>

#include "utility/runnable.hpp"

#include "properties.hpp"

namespace vtslibs { namespace vts0 {

/** Tile set descriptor.
 */
struct TileSetDescriptor {
    boost::filesystem::path path;

    typedef std::map<std::string, TileSetDescriptor> map;

    TileSetDescriptor() {}
};

/** Storage properties.
 */
struct StorageProperties {
    /** Set of input tile set descriptors.
     */
    TileSetDescriptor::map inputSets;

    /** Output tile sets.
     */
    TileSetDescriptor outputSet;

    StorageProperties() {}
};

/** Storage create properties.
 */
struct StorageCreateProperties {
    /** Output tile set create properties.
     */
    CreateProperties createProperties;

    StorageCreateProperties() {}

    StorageCreateProperties(const CreateProperties &createProperties)
        : createProperties(createProperties)
    {}
};

class Storage : boost::noncopyable
{
public:
    typedef std::shared_ptr<Storage> pointer;

    StorageProperties getProperties() const;

    void addTileSet(const boost::filesystem::path &path
                    , utility::Runnable *runnable = nullptr);

    void addTileSets(const std::vector<boost::filesystem::path> &paths
                     , utility::Runnable *runnable = nullptr);

    void rebuildOutput();

    /** Needed to instantiate.
     */
    class Factory;
    friend class Factory;

private:
    Storage(const boost::filesystem::path &root, bool readOnly);

    struct Detail;
    std::unique_ptr<Detail> detail_;
    Detail& detail() { return *detail_; }
    const Detail& detail() const { return *detail_; }
};


// inline stuff

inline void Storage::addTileSet(const boost::filesystem::path &path
                                , utility::Runnable *runnable)
{
    return addTileSets({path}, runnable);
}

} } // namespace vtslibs::vts0

#endif // vtslibs_vts0_storage_hpp_included_
