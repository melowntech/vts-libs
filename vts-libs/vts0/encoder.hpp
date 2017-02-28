#ifndef vtslibs_vts0_encoder_hpp_included_
#define vtslibs_vts0_encoder_hpp_included_

#include <memory>

#include <boost/optional.hpp>

#include "geo/srsdef.hpp"

#include "./tileset.hpp"

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
