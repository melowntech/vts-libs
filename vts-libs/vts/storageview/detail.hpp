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
 * \file vts/storage/detail.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Storage access (internals).
 */

#ifndef vtslibs_vts_storage_detail_hpp_included_
#define vtslibs_vts_storage_detail_hpp_included_

#include <boost/filesystem/path.hpp>

#include "../storageview.hpp"
#include "../storage.hpp"

namespace vtslibs { namespace vts {

using vtslibs::storage::PathStat;

struct StorageView::Properties : StorageViewProperties {
    /** Paths to full-blown map configuration to merge-in.
     */
    std::vector<boost::filesystem::path> includeMapConfigs;

    /** Absolutizes all filesystem paths.
     */
    void absolutize(const boost::filesystem::path &root);
};

struct StorageView::Detail
{
    boost::filesystem::path configPath;

    Properties properties;

    registry::ReferenceFrame referenceFrame;

    /** Information about config (and other files) when tileset was opened.
     */
    PathStat::list stats;

    /** Time of last modification
     */
    std::time_t lastModified;

    /** Associated storage.
     */
    Storage storage;

    Detail(const boost::filesystem::path &root);

    Detail(const boost::filesystem::path &root
           , const Properties &properties, Storage storage);

    ~Detail();

    void loadConfig();

    static StorageView::Properties
    loadConfig(const boost::filesystem::path &configPath);

    void saveConfig();

    bool externallyChanged() const;

    MapConfig mapConfig() const;

    static MapConfig mapConfig(const boost::filesystem::path &configPath);

    static MapConfig mapConfig(const boost::filesystem::path &configPath
                               , const StorageView::Properties &properties);

private:
    void makeStat();
};

} } // namespace vtslibs::vts

#endif // vtslibs_vts_storage_detail_hpp_included_
