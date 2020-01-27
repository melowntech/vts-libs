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

#ifndef vtslibs_vts_merge_coverage_hpp_included_
#define vtslibs_vts_merge_coverage_hpp_included_

#include <cstdint>
#include <vector>
#include <iosfwd>

#include <boost/logic/tribool.hpp>
#include <boost/filesystem/path.hpp>

#include <opencv2/core/core.hpp>

#include "utility/openmp.hpp"

#include "../merge.hpp"

namespace vtslibs { namespace vts { namespace merge {

typedef imgproc::Contour::list CookieCutters;

struct Coverage {
    typedef std::int16_t pixel_type;
    typedef cv::Vec2f HeightMapValue;
    typedef cv::Mat_<HeightMapValue> HeightMap;
    typedef cv::Mat_<pixel_type> MatType;

    const TileId tileId;
    const Input::list &sources;
    const MergeOptions &options;
    const math::Size2 innerSize;
    const math::Size2 coverageSize;
    MatType coverage;
    bool hasHoles;
    std::vector<bool> indices;
    boost::optional<Input::Id> single;

    /** List of cookie cutters for valid sources (i.e. indices[id] == bool)
     *  NB: use input.id() as index to this list.
     */
    CookieCutters cookieCutters;

    /** Heightmap filled in by covered() function.
     */
    HeightMap hm;

    Coverage(const TileId &tileId, const NodeInfo &nodeInfo
             , const Input::list &sources, const MergeOptions &options
             , bool needCookieCutters);

    void getSources(Output &output, const Input::list &navtileSource) const;

    /** Checks for face coverage by given surface (id). Updates height map with
     *  minumum sampled value.
     *
     * \param face face to check
     * \param vertices vertices referenced by face
     * \param id surface identifier
     * \return tribool where:
     *             false: face is fully outside
     *             true: face is fully inside
     *             intermediate: face is partially inside
     */
    boost::tribool covered(const Face &face, const math::Points3d &vertices
                           , Input::Id id);

    /** Simplified hit test.
     */
    struct Hit {
        bool inside;
        bool covered;
    };

    Hit hit(const Face &face, const math::Points3d &vertices
            , Input::Id id) const;

    void dump(const boost::filesystem::path &dump) const;
    void dumpCookieCutters(std::ostream &os) const;

    void dumpCookieCutters(const boost::filesystem::path &dump) const;

    bool topmost(const Input &input) const { return topmost(input.id()); }

    bool topmost(const Input::Id &id) const { return id == topmost_; }

    void dilateHm();

    boost::optional<double> hmMin(int x, int y) const;

    boost::optional<double> hmMax(int x, int y) const;

    boost::optional<double> hmAvg(int x, int y) const;

private:
    void generateCoverage(const NodeInfo &nodeInfo);

    void analyze();

    void findCookieCutters();

    /** Id of topmost surface
     */
    Input::Id topmost_;
};

} } } // namespace vtslibs::vts::merge

#endif // vtslibs_vts_merge_coverage_hpp_included_
