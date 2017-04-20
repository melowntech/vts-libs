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
#ifndef vtslibs_vts_tileset_driver_aggregated_old_hpp_included_
#define vtslibs_vts_tileset_driver_aggregated_old_hpp_included_

#include <set>
#include <map>
#include <memory>

#include "../driver.hpp"
#include "../../storage.hpp"

namespace vtslibs { namespace vts { namespace driver {

/** Helper class.
 */
struct OldAggregatedDriverBase {
    OldAggregatedDriverBase() {}
    OldAggregatedDriverBase(const CloneOptions &cloneOptions);
};

class OldAggregatedDriver : private OldAggregatedDriverBase, public Driver {
public:
    typedef std::shared_ptr<OldAggregatedDriver> pointer;

    /** Opens storage.
     */
    OldAggregatedDriver(const boost::filesystem::path &root
                        , const OpenOptions &openOptions
                        , const OldAggregatedOptions &options);

    /** Cloner
     */
    OldAggregatedDriver(const boost::filesystem::path &root
                     , const OldAggregatedOptions &options
                     , const CloneOptions &cloneOptions
                     , const OldAggregatedDriver &src);

    virtual ~OldAggregatedDriver();

    /** Base class for tileset and glue info.
     */
    struct EnhancedInfo {
        Driver::pointer driver;
        tileset::Index::pointer tsi;
        std::string name;
        bool hasAlienTiles;
        bool isAlien;

        EnhancedInfo(const registry::ReferenceFrame &referenceFrame)
            : tsi(std::make_shared<tileset::Index>
                  (referenceFrame.metaBinaryOrder))
            , hasAlienTiles(false), isAlien(false)
        {}
    };

    struct TileSetInfo : EnhancedInfo {
        struct GlueInfo : TileSetGlues::EnhancedGlue, EnhancedInfo {
            GlueInfo(const registry::ReferenceFrame &referenceFrame
                     , const TileSetGlues::EnhancedGlue &glue)
                : EnhancedGlue(glue)
                , EnhancedInfo(referenceFrame)
            {}

            typedef std::vector<GlueInfo> list;
        };

        TilesetId tilesetId;

        GlueInfo::list glues;

        TileSetInfo(const registry::ReferenceFrame &referenceFrame
                    , const TileSetGlues &tsg)
            : EnhancedInfo(referenceFrame)
            , tilesetId(tsg.tilesetId)
        {
            for (const auto &g : tsg.glues) {
                glues.emplace_back(referenceFrame, g);
            }
        }

        typedef std::vector<TileSetInfo> list;
    };

    /** Reencodes aggregated tileset.
     */
    static bool reencode(const boost::filesystem::path &root
                         , const OldAggregatedOptions &driverOptions
                         , const ReencodeOptions &options
                         , const std::string &prefix = "");

private:
    virtual OStream::pointer output_impl(const File type);

    virtual IStream::pointer input_impl(File type) const {
        return input_impl(type, true);
    }

    virtual IStream::pointer input_impl(File type, const NullWhenNotFound_t&)
        const
    {
        return input_impl(type, false);
    }

    virtual OStream::pointer
    output_impl(const TileId &tileId, TileFile type);

    virtual IStream::pointer
    input_impl(const TileId &tileId, TileFile type) const {
        return input_impl(tileId, type, true);
    }

    virtual IStream::pointer
    input_impl(const TileId &tileId, TileFile type, const NullWhenNotFound_t&)
        const
    {
        return input_impl(tileId, type, false);
    }

    IStream::pointer input_impl(File type, bool noSuchFile) const;

    IStream::pointer input_impl(const TileId &tileId, TileFile type
                                , bool noSuchFile) const;

    virtual void drop_impl();

    virtual void flush_impl();

    virtual FileStat stat_impl(File type) const;

    virtual FileStat stat_impl(const TileId &tileId, TileFile type) const;

    virtual Resources resources_impl() const;

    Driver::pointer clone_impl(const boost::filesystem::path &root
                               , const CloneOptions &cloneOptions) const;

    virtual std::string info_impl() const;

    IStream::pointer input_mem(File type) const;

    virtual tileset::Index* getTileIndex_impl() { return &tsi_; }

    virtual const tileset::Index* getTileIndex_impl() const { return &tsi_; }

    inline const OldAggregatedOptions& options() const {
        return Driver::options<const OldAggregatedOptions&>();
    }

    TileSetInfo::list buildTilesetInfo() const;

    TileSet::Properties build(const OldAggregatedOptions &options
                              , const CloneOptions &cloneOptions);

    Storage storage_;

    registry::ReferenceFrame referenceFrame_;

    tileset::Index tsi_;

    /** Stuff ripe for delivery.
     */
    TileSetInfo::list tilesetInfo_;

    boost::optional<TileSet::Properties> memProperties_;
};

} } } // namespace vtslibs::vts::driver

#endif // vtslibs_vts_tileset_driver_aggregated_hpp_included_
