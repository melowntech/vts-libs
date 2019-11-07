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
#include <algorithm>

#include "dbglog/dbglog.hpp"

#include "nodeinfo.hpp"
#include "tileop.hpp"
#include "csconvertor.hpp"

namespace vtslibs { namespace vts {

namespace {

math::Extents2 makeExtents(const RFNode &rootNode, const RFNode::Id &nodeId)
{
    // determine tile extents
    auto lid(local(rootNode.id.lod, nodeId));
    auto tc(tileCount(lid.lod));
    auto rs(size(rootNode.extents));
    math::Size2f ts(rs.width / tc, rs.height / tc);
    return  math::Extents2
        (rootNode.extents.ll(0) + lid.x * ts.width
         , rootNode.extents.ur(1) - (lid.y + 1) * ts.height
         , rootNode.extents.ll(0) + (lid.x + 1) * ts.width
         , rootNode.extents.ur(1) - lid.y * ts.height);
}

RFNode makeNode(const RFNode &subtreeRoot
                , const TileId &tileId)
{
    // clone root
    auto node(subtreeRoot);

    // set id
    node.id = tileId;

    // change extents for productive nodes only
    if (subtreeRoot.real()) {
        // set extents
        node.extents = makeExtents(subtreeRoot, tileId);
    }

    return node;
}

bool checkPartial(const RFTreeSubtree &subtree, RFNode &node
                  , bool invalidateWhenMasked = true)
{
    auto valid(subtree.valid(node));
    if (valid) {
        return false;
    } else if (!valid && invalidateWhenMasked) {
        // masked node -> invalidate if allowed
        node.invalidate();
        return false;
    }

    // indeterminate -> valid but partial
    return true;
}

const RFNode invalidNode(RFNode::Id(~Lod(0), 0, 0)
                         , registry::PartitioningMode::none);

const RFNode& findSubtreeRoot(const registry::ReferenceFrame &referenceFrame
                              , const TileId &tileId)
{
    const auto *node(referenceFrame.findSubtreeRoot(tileId, std::nothrow));
    if (!node) {
        // cannot find root for this node -> return invalid node
        return invalidNode;
    }
    return *node;
}

} // namespace

inline NodeInfo::NodeInfo(const registry::ReferenceFrame &referenceFrame
                          , const RFNode &node
                          , const registry::Registry &reg)
    : referenceFrame_(&referenceFrame)
    , subtree_(node, reg), node_(node)
    , partial_(checkPartial(subtree_, node_))
{}

NodeInfo::NodeInfo(const registry::ReferenceFrame &referenceFrame
                   , const TileId &tileId, bool invalidateWhenMasked
                   , const registry::Registry &reg)
    : referenceFrame_(&referenceFrame)
    , subtree_(findSubtreeRoot(*referenceFrame_, tileId), reg)
    , node_(makeNode(subtree_.root(), tileId))
    , partial_(checkPartial(subtree_, node_, invalidateWhenMasked))
{}

NodeInfo::list NodeInfo::nodes(const registry::ReferenceFrame &referenceFrame
                               , const registry::Registry &reg)
{
    NodeInfo::list nodes;
    for (const auto &item : referenceFrame.division.nodes) {
        if (item.second.real()) {
            nodes.push_back(NodeInfo(referenceFrame, item.second, reg));
        }
    }
    return nodes;
}

NodeInfo::list NodeInfo::leaves(const registry::ReferenceFrame &referenceFrame
                                , const registry::Registry &reg)
{
    NodeInfo::list nodes;
    for (const auto &item : referenceFrame.division.nodes) {
        if (!item.second.real() || item.second.structure.children) {
            // not real node nor leaf
            continue;
        }
        nodes.push_back(NodeInfo(referenceFrame, item.second, reg));
    }
    return nodes;
}

NodeInfo NodeInfo::child(Child childDef) const
{
    if (!node_.valid()) {
        LOGTHROW(err2, storage::Error)
            << "Node " << node_.id << " has no children.";
    }

    // build child id from this node and index
    RFNode::Id childId(node_.id);
    ++childId.lod;
    childId.x <<= 1;
    childId.y <<= 1;

    switch (childDef.index) {
    case 0: // upper-left
        break;

    case 1: // upper-right
        ++childId.x;
        break;

    case 2: // lower-left
        ++childId.y;
        break;

    case 3: // lower-right
        ++childId.x;
        ++childId.y;
        break;

    default:
        LOGTHROW(err2, storage::Error)
            << "Invalid child number (" << childDef.index << ").";
        break;
    }

    // check for child validity
    if ((childId.lod != childDef.lod)
        || (childId.x != childDef.x)
        || (childId.y != childDef.y))
    {
        LOGTHROW(err2, storage::Error)
            << "Node " << childId << " is not a child of "
            << node_.id << ".";
    }

    if (node_.structure.children) {
        // manual or barren node -> check for validity
        if (node_.structure.children & (1 << childDef.index)) {
            // yes, path exists, replace
            return { *referenceFrame_, referenceFrame_->find(childId)
						, subtree_.registry() };
        }

        // non-existent node -> invalid
        return { *referenceFrame_
                , RFNode(childId, registry::PartitioningMode::none)
                , subtree_.registry()};
    }

    // divide current node's extents in half in both directions
    NodeInfo child(*this);
    child.node_.id = childId;

    // size of extents
    auto es(size(node_.extents));
    // and halve it
    es.width /= 2.0;
    es.height /= 2.0;

    // no need to check childNum since it was checked above
    auto &extents(child.node_.extents);
    switch (childDef.index) {
    case 0: // upper-left
        extents.ur(0) -= es.width;
        extents.ll(1) += es.height;
        break;

    case 1: // upper-right
        extents.ll(0) += es.width;
        extents.ll(1) += es.height;
        break;

    case 2: // lower-left
        extents.ur(0) -= es.width;
        extents.ur(1) -= es.height;
        break;

    case 3: // lower-right
        extents.ll(0) += es.width;
        extents.ur(1) -= es.height;
        break;
    }

    // partiality is inherited from parent -> only partial node needs to be
    // re-checked
    if (child.partial_) {
        child.partial_ = checkPartial(child.subtree_, child.node_);
    }

    // done
    return child;
}

NodeInfo NodeInfo::child(const TileId &childId) const
{
    NodeInfo child(*this);
    child.node_.id = childId;

    child.node_.extents = makeExtents(node_, childId);

    // partiality is inherited from parent -> only partial node needs to be
    // re-checked
    if (child.partial_) {
        child.partial_ = checkPartial(child.subtree_, child.node_);
    }

    return child;
}

class RFTreeSubtree::Sampler : boost::noncopyable {
public:
    Sampler(const RFNode &root, const registry::Registry &reg)
        : conv_(root.srs, root.constraints->extentsSrs, reg)
        , extents_(root.constraints->extents)
    {}

    math::Point2 sample(const math::Point2 &p) const {
        return conv_(p);
    }

    bool inside(const math::Point2 &p) const {
        return math::inside(extents_, sample(p));
    }

private:
    CsConvertor conv_;
    math::Extents2 extents_;
};

bool RFTreeSubtree::initSampler() const
{
    if (!root_->constraints) { return false; }
    if (!sampler_) {
        sampler_ = std::make_shared<Sampler>(*root_, *registry_);
    }
    return true;
}

boost::tribool RFTreeSubtree::valid(const RFNode &node) const
{
    // try to init sampler
    if (!initSampler()) {
        // sampler cannot be initialized because there are no extra constraints
        if (!root_->valid()) {
            // invalid node -> invalid
            return false;
        }

        if (!root_->real()) {
            // not a real node, we have no idea...
            return boost::indeterminate;
        }

        // valid
        return true;
    }

    class Checker {
    public:
        Checker(const Sampler &sampler)
            : sampler_(sampler), inside_(false), outside_(false)
        {}

        bool operator()(const math::Point2 &p) {
            if (sampler_.inside(p)) {
                inside_ = true;
            } else {
                outside_ = true;
            }

            return (inside_ && outside_);
        }

        boost::tribool result() const {
            // both inside and outside -> indeterminate
            if (inside_ && outside_) {
                return boost::indeterminate;
            }

            if (inside_) { return true; }
            return false;
        }

    private:
        const Sampler &sampler_;
        bool inside_;
        bool outside_;
    };

    Checker check(*sampler_);

    // check tile corners first
    if (check(ll(node.extents)) || check(ur(node.extents))
        || check(ul(node.extents)) || check(lr(node.extents)))
    {
        return check.result();
    }

    // calculate center of tile
    auto c(math::center(node.extents));

    // check center of tile
    if (check(c)) { return check.result(); }

    // check centers of tile borders
    if (check({ node.extents.ll(0), c(1) }) // left border
        || check({ node.extents.ur(0), c(1) }) // right border
        || check({ c(0), node.extents.ll(1) } ) // bottom border
        || check({ c(0), node.extents.ur(1) } )) // top border
    {
        return check.result();
    }

    // done
    return check.result();
}

bool RFTreeSubtree::inside(const math::Point2 &point) const
{
    // try to init sampler; if sampler cannot be initialized, we are fully
    // inside
    if (!initSampler()) { return true; }

    return sampler_->inside(point);
}

NodeInfo::CoverageMask
NodeInfo::coverageMask(CoverageType type, const math::Size2 &size
                       , unsigned int dilation) const
{
    if (!valid()) {
        return CoverageMask(size, CoverageMask::InitMode::EMPTY);
    }

    if (!partial_) {
        return CoverageMask(size, CoverageMask::InitMode::FULL);
    }

    return subtree_.coverageMask(type, size, dilation, node_);
}

RFTreeSubtree::CoverageMask
RFTreeSubtree::coverageMask(CoverageType type, const math::Size2 &size
                            , int dilation, const RFNode &node) const
{
    if (!initSampler()) {
        // no sampler -> no constraints -> full mask
        return CoverageMask(size, CoverageMask::InitMode::FULL);
    }

    // extents to process
    const auto grid([&]() -> math::Extents2
    {
        // grid coordinates: leave extents
        if (type == CoverageType::grid) { return node.extents; }

        // pixel coordinates: move one half pixel inside
        auto grid(node.extents);
        auto s(math::size(node.extents));
        math::Size2f hps(s.width / (2.0 * size.width)
                         , s.height / (2.0 * size.height));
        grid.ll(0) += hps.width;
        grid.ll(1) += hps.height;
        grid.ur(0) -= hps.width;
        grid.ur(1) -= hps.height;
        return grid;
    }());

    // get grid size and calculate pixel size (i.e. step)
    const auto gs(math::size(grid));
    math::Size2f ps(gs.width / (size.width - 1)
                    , gs.height / (size.height - 1));
    const auto ref(ul(grid));

    // NB: we cannot use OpenCV here, this is core lib functionality
    std::vector<std::uint8_t> pane(area(size), false);

    auto clip([](int v, int limit) -> int
    {
        if (v < 0) { return 0; }
        if (v > limit) { return limit; }
        return v;
    });

    // sample tile in grid
    const auto &sampler(*sampler_);
    for (int j(-dilation), je(size.height + dilation), gp(0); j < je; ++j) {
        double y(ref(1) - j * ps.height);
        for (int i(-dilation), ie(size.width + dilation); i < ie; ++i, ++gp) {
            double x(ref(0) + i * ps.width);
            if (sampler.inside(math::Point2(x, y))) {
                if (!dilation) {
                    pane[gp] = true;
                }

                // some dilation -> apply
                for (int jj(clip(j - dilation, size.height - 1))
                         , jje(clip(j + dilation, size.height - 1))
                         , gy(jj * size.width);
                     jj <= jje; ++jj, gy += size.width) {
                    for (int ii(clip(i - dilation, size.width - 1))
                             , iie(clip(i + dilation, size.width - 1));
                         ii <= iie; ++ii)
                    {
                        pane[gy + ii] = true;
                    }
                }
            }
        }
    }

    CoverageMask mask(size, CoverageMask::InitMode::EMPTY);
    for (int j(0), gp(0); j < size.height; ++j) {
        for (int i(0); i < size.width; ++i, ++gp) {
            if (pane[gp]) {
                mask.set(i, j, true);
            }
        }
    }

    return mask;
}

bool NodeInfo::inside(const math::Point2 &point) const
{
    // outside -> not inside
    if (!math::inside(node_.extents, point)) { return false; }
    // inside and not partial -> inside
    if (!partial_) { return true; }

    // OK, check

    // check for validty
    return subtree_.inside(point);
}

const geo::SrsDefinition& NodeInfo::srsDef() const
{
    return subtree_.registry().srs(node_.srs).srsDef;
}

NodeInfo::CoveredArea NodeInfo::checkMask(const CoverageMask &mask
                                          , CoverageType type
                                          , unsigned int dilation) const
{
    if (mask.empty() || !valid()) {
        // nothing at all
        return CoveredArea::none;
    }

    // full node -> result is based on mask content
    if (!partial()) {
        return mask.full() ? CoveredArea::whole : CoveredArea::some;
    }

    // partial node: subtract mask from node's mask; if anything is left then
    // mask doesn't cover all pixels in node valid area and therefore coverage
    // is partial; otherwise we have full coverage, i.e. watertight file

    // generate node mask
    auto nm(coverageMask(type, mask.size(), dilation));
    auto total(nm.count());

    // subtract provide mask from node mask
    nm.subtract(mask);

    // empty resukt -> all valid pixels in node are also valid in mask -> whole
    // valid are is covered
    if (nm.empty()) { return CoveredArea::whole; }

    // some pixels are left
    if (nm.count() == total) {
        // nothing in node's mask is in the mask -> nothing covered
        return CoveredArea::none;
    }

    // something in the mask covers valid node mask -> there is something
    return CoveredArea::some;
}

#ifdef GEO_HAS_GDAL
registry::Srs NodeInfo::navsds() const
{
    const auto &reg(subtree_.registry());
    // clone navigation system (with all info)
    auto srs(reg.srs(referenceFrame_->model.navigationSrs));
    // update its SRS by merging node's horizontal system with navigation
    // vertical system
    srs.srsDef = geo::merge(reg.srs(node_.srs).srsDef, srs.srsDef);
    return srs;
}
#endif // GEO_HAS_GDAL

} } // namespace vtslibs::vts
