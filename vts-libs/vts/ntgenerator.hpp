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
#ifndef vts_ntgenerator_hpp_included_
#define vts_ntgenerator_hpp_included_

#include <limits>
#include <functional>

#include <boost/noncopyable.hpp>
#include <boost/optional.hpp>
#include <boost/variant.hpp>
#include <boost/filesystem/path.hpp>

#include "basetypes.hpp"
#include "nodeinfo.hpp"
#include "tileset.hpp"

namespace vtslibs { namespace vts {

class NtGenerator : boost::noncopyable {
public:
    NtGenerator(const registry::ReferenceFrame *referenceFrame
                , const boost::optional<boost::filesystem::path> &path
                = boost::none);

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

    /** Callbacks called when tile is assigned a
     */
    struct Reporter {
        virtual void expect(std::size_t) {};
        virtual void report() {};
        virtual ~Reporter() {}
    };

    /** Generate navtiles and surrogates.
     */
    void generate(TileSet &ts, double dtmExtractionRadius) const;

    /** Generate navtiles and surrogates.
     */
    void generate(TileSet &ts, double dtmExtractionRadius
                  , Reporter &reporter) const;

    struct Accumulator;

private:
    const registry::ReferenceFrame *referenceFrame_;
    const std::map<std::string, const RFNode*> sds2rfnode_;

    std::map<const RFNode*, std::shared_ptr<Accumulator>> accumulators_;
};

} } // namespace vtslibs::vts

#endif // vts_ntgenerator_hpp_included_
