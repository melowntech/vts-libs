#ifndef vts_ntgenerator_hpp_included_
#define vts_ntgenerator_hpp_included_

#include <limits>

#include <boost/noncopyable.hpp>
#include <boost/variant.hpp>
#include <boost/filesystem/path.hpp>

#include "./basetypes.hpp"
#include "./nodeinfo.hpp"
#include "./tileset.hpp"
#include "./heightmap.hpp"

namespace vtslibs { namespace vts {

class NtGenerator : boost::noncopyable {
public:
    NtGenerator(const registry::ReferenceFrame *referenceFrame);
    NtGenerator(const registry::ReferenceFrame *referenceFrame
                , const boost::filesystem::path &path);

    void load(const boost::filesystem::path &path);
    void save(const boost::filesystem::path &path);

    /** Add accumulator for given reference frame subtree (defined by
     *  ntinfo.sds)
     */
    void addAccumulator(const std::string &sds, const LodRange &lodRange
                        , double pixelSize);

    typedef boost::variant<boost::blank, std::string, geo::SrsDefinition>
        TileSrs;

    /** Add tile to the generator.
     *
     *  Provided mesh is either in SDS srs or in provided srs which must be
     *  identical to SDS srs except having vertical datum with/without vertical
     *  shift grid.
     *
     * Rationale: All mesh vertices are transformed from SDS srs/provided srs
     * to navigation SRS and converted vertex Z component is used as height
     *
     * Srs override is defined as a variant:
     *     * blank: no override
     *     * std::string: registry key
     *     * geo::SrsDefinition: srs
     */
    void addTile(const TileId &tileId, const NodeInfo &nodeInfo
                 , const Mesh &mesh, const TileSrs &srs = TileSrs());


    /** Generate navtiles and surrogates.
     */
    void generate(TileSet &ts, double dtmExtractionRadius) const;

    struct Accumulator;

private:
    const registry::ReferenceFrame *referenceFrame_;
    const std::map<std::string, const RFNode*> sds2rfnode_;

    std::map<const RFNode*, std::shared_ptr<Accumulator>> accumulators_;
};

} } // namespace vtslibs::vts

#endif // vts_ntgenerator_hpp_included_
