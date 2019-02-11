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
#ifndef vtslibs_vts0_encoder_hpp_included_
#define vtslibs_vts0_encoder_hpp_included_

#include <memory>

#include <boost/optional.hpp>

#include "geo/srsdef.hpp"

#include "tileset.hpp"

namespace vtslibs { namespace vts0 {

class Encoder {
public:
    Encoder(const boost::filesystem::path &path
            , const CreateProperties &properties, CreateMode mode);

    virtual ~Encoder() {}

    /** Start encoding process from root tile.
     */
    void run();

    struct Constraints;

protected:
    Properties properties() const;
    geo::SrsDefinition srs() const;
    void setConstraints(const Constraints &constraints);

    /** Returned by getTile when metadata are set.
     */
    enum TileResult {
        noDataYet
        , noData
        , data
        , dataWithMetadata
    };

private:
    /** Generates mesh, atlas and optionally metadata for given tile.
     */
    virtual TileResult getTile(const TileId &tileId
                               , const math::Extents2 &tileExtents
                               , Mesh &mesh, Atlas &atlas
                               , TileMetadata &metadata) = 0;


    /** Generates mesh, atlas and optionally metadata for given tile.
     */
    virtual void finish(TileSet &tileSet) = 0;

    void process(const TileId &tileId, int useConstraints);

    struct Detail;
    friend struct Detail;
    std::shared_ptr<Detail> detail_;
};

struct Encoder::Constraints {
    boost::optional<LodRange> lodRange;
    boost::optional<math::Extents2> extents;

    enum {
        useLodRange = 0x01
        , useExtents = 0x02

        , all = (useLodRange | useExtents)
    };

    Constraints& setLodRange(const boost::optional<LodRange> &value);

    Constraints& setExtents(const boost::optional<math::Extents2> &value);
};

inline Encoder::Constraints&
Encoder::Constraints::setLodRange(const boost::optional<LodRange> &value)
{
    lodRange = value;
    return *this;
}

inline Encoder::Constraints&
Encoder::Constraints::setExtents(const boost::optional<math::Extents2> &value)
{
    extents = value;
    return *this;
}

} } // namespace vtslibs::vts0

#endif // vtslibs_vts0_encoder_hpp_included_
