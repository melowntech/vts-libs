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
#include "math/math.hpp"

#include "tileop.hpp"
#include "csconvertor.hpp"
#include "subtrees.hpp"

namespace vtslibs { namespace vts {

Subtrees findSubtrees(const registry::ReferenceFrame &referenceFrame, Lod lod
                      , const geo::SrsDefinition &srs
                      , const math::Extents2 &extents
                      , int samples)
{
    Subtrees st;

    // process all nodes
    for (const auto &item : referenceFrame.division.nodes) {
        // skip invalid nodes
        const auto &node(item.second);
        if (!node.real()) { continue; }

        // skip nodes below requested lod
        const auto &nid(node.id);
        if (nid.lod > lod) { continue; }

        auto nodeRange(vts::childRange
                       ({nid.x, nid.y, nid.x, nid.y}, (lod - nid.lod)));

        // check for subtree validity
        vts::NodeInfo ni(referenceFrame, vts::tileId(node.id));
        vts::NodeInfo cni(referenceFrame, vts::tileId(lod, nodeRange.ll));
        if (ni.subtree() != cni.subtree()) {
            // skipped some node -> not a good node here
            continue;
        }

        math::Extents2 e(math::InvalidExtents{});

        const vts::CsConvertor conv(srs, node.srs);
        const auto dss(math::size(extents));
        math::Size2f px(dss.width / samples, dss.height / samples);

        // convert dataset's extents into node's SRS
        for (int j(0); j <= samples; ++j) {
            auto y(extents.ll(1) + j * px.height);
            for (int i(0); i <= samples; ++i) {
                math::Point2 p(extents.ll(0) + i * px.width, y);
                try {
                    // try to convert point from dataset's SRS into node
                    auto pp(conv(p));

                    // check whether point is inside valid area of node
                    if (ni.inside(pp)) {
                        update(e, pp);
                    }
                } catch (...) {}
            }
        }

        // skip invalid extents
        if (!valid(e)) { continue; }

        // convert range to node range

        // tile size in this subtree
        const auto ts(vts::tileSize(node.extents, (lod - nid.lod)));

        math::Point2 llDiff(e.ll(0) - node.extents.ll(0)
                            , node.extents.ur(1) - e.ur(1));
        math::Point2 urDiff(e.ur(0) - node.extents.ll(0)
                            , node.extents.ur(1) - e.ll(1));

        math::Point2 llId(llDiff(0) / ts.width, llDiff(1) / ts.height);
        math::Point2 urId(urDiff(0) / ts.width, urDiff(1) / ts.height);

        auto fix([](double &x, bool up) -> void
        {
            if (math::isInteger(x, 1e-15)) {
                // close enough to be an integer
                x = std::round(x);
            }
            // too far away, floor/ceil
            if (up) {
                if (x < 0) {
                    x = std::floor(x);
                } else {
                    x = std::ceil(x);
                }
            } else {
                if (x < 0) {
                    x = std::ceil(x);
                } else {
                    x = std::floor(x);
                }
            }
        });


        // fix ids
        fix(llId(0), false); fix(llId(1), false);
        fix(urId(0), true); fix(urId(1), true);

        // shift to fit into node
        vts::TileRange r(nodeRange.ll(0) + int(llId(0))
                         , nodeRange.ll(1) + int(llId(1))
                         , nodeRange.ll(0) + int(urId(0))
                         , nodeRange.ll(1) + int(urId(1)));

        st.ranges.insert(Subtrees::Ranges::value_type(vts::tileId(nid), r));

        // merge into overall tile range
        math::update(st.overallRange, r.ll);
        math::update(st.overallRange, r.ur);
    }

    return st;
}

} } // namespace vtslibs::vts
