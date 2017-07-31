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

#include <boost/filesystem/path.hpp>

#include <opencv2/core/core.hpp>

#include "../merge.hpp"

namespace vtslibs { namespace vts { namespace merge {

typedef std::vector<imgproc::Contours> CookieCutters;

struct Coverage {
    typedef std::int16_t pixel_type;

    const TileId tileId;
    const Input::list &sources;
    cv::Mat_<pixel_type> coverage;
    bool hasHoles;
    std::vector<bool> indices;
    boost::optional<Input::Id> single;
    CookieCutters cookieCutters;

    Coverage(const TileId &tileId, const NodeInfo &nodeInfo
             , const Input::list &sources);

    void getSources(Output &output, const Input::list &navtileSource) const;

    std::tuple<bool, bool>
    covered(const Face &face, const math::Points3d &vertices
            , Input::Id id) const;

    void dump(const boost::filesystem::path &dump) const;

    void dumpCookieCutters(const boost::filesystem::path &dump) const;

private:
    void generateCoverage(const NodeInfo &nodeInfo);

    void analyze();

    void findCookieCutters();
};

} } } // namespace vtslibs::vts::merge

#endif // vtslibs_vts_merge_coverage_hpp_included_
