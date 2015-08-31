#ifndef vadstena_libs_vts_encoder_hpp_included_
#define vadstena_libs_vts_encoder_hpp_included_

#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>

#include "geo/srsdef.hpp"

#include "./tileset.hpp"

namespace vadstena { namespace vts {

class Encoder : boost::noncopyable {
public:
    struct Constraints {
        boost::optional<LodRange> lodRange;
        boost::optional<math::Extents2> extents;

        Constraints& setLodRange(const boost::optional<LodRange> &value) {
            lodRange = value;
            return *this;
        }

        Constraints& setExtents(const boost::optional<math::Extents2> &value) {
            extents = value;
            return *this;
        }

        enum {
            useLodRange = 0x01
            , useExtents = 0x02

            , all = (useLodRange | useExtents)
        };
    };

    Encoder(const boost::filesystem::path &path
            , const CreateProperties &properties, CreateMode mode);

    virtual ~Encoder() {}

    /** Start encoding process from root tile.
     */
    void run();

    TileSet& tileSet() { return *tileSet_; }

protected:
    Properties properties() const { return properties_; }
    geo::SrsDefinition srs() const { return srs_; }

    void setConstraints(const Constraints &constraints) {
        constraints_ = constraints;
    }

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


    virtual void finish(TileSet &tileSet) = 0;

    void process(const TileId &tileId, int useConstraints);

    TileSet::pointer tileSet_;
    Properties properties_;
    geo::SrsDefinition srs_;

    Constraints constraints_;
};

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_encoder_hpp_included_
