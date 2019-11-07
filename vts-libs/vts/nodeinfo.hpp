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
#ifndef vtslibs_vts_nodeinfo_hpp_included_
#define vtslibs_vts_nodeinfo_hpp_included_

#include <boost/logic/tribool.hpp>

#include "utility/enum-io.hpp"
#include "imgproc/rastermask/quadtree.hpp"

#include "basetypes.hpp"

namespace vtslibs { namespace vts {

class RFTreeSubtree {
public:
    RFTreeSubtree(const RFNode &root, const registry::Registry &reg)
        : root_(&root), registry_(&reg)
    {}

    const RFNode& root() const { return *root_; }

    const RFNode::Id& id() const { return root_->id; }

    Lod lod() const { return root_->id.lod; }

    bool operator==(const RFTreeSubtree &other) const {
        return (root_ == other.root_);
    }

    bool operator!=(const RFTreeSubtree &other) const {
        return !operator==(other);
    }

    /** Calculates node validity:
     *    * false: node is completely outside subtree's valid area
     *    * true: node is completely inside subtree's valid area
     *    * indeterminate: node is partially inside subtree's valid area
     */
    boost::tribool valid(const RFNode &node) const;

    /** Check whether point is inside.
     */
    bool inside(const math::Point2 &point) const;

    /** Node coverage mask.
     */
    typedef imgproc::quadtree::RasterMask CoverageMask;

    /** Node coverage mask type: pixel or grid.
     */
    enum class CoverageType { pixel, grid };

    CoverageMask coverageMask(CoverageType type, const math::Size2 &size
                              , int dilation, const RFNode &node)
        const;

    const registry::Registry& registry() const { return *registry_; }

private:
    bool initSampler() const;

    class Sampler;

    const RFNode *root_;
    const registry::Registry *registry_;
    mutable std::shared_ptr<Sampler> sampler_;
};

/** Reference frame node information.
 */
class NodeInfo {
public:
    typedef std::vector<NodeInfo> list;

    /** Creates node info from reference frame and tileId.
     *
     * Root node is found in reference frame and then current node is derived.
     *
     * \param referenceFrame reference frame
     * \param tileId ID of node/tile
     * \param invalidateWhenMasked masked node is automatically invalidated
     */
    NodeInfo(const registry::ReferenceFrame &referenceFrame
             , const TileId &tileId, bool invalidateWhenMasked = true
             , const registry::Registry &reg = registry::system);

    /** Root node info.
     */
    NodeInfo(const registry::ReferenceFrame &referenceFrame
             , const registry::Registry &reg = registry::system);

    /** Node.
     */
    const RFNode& node() const { return node_; }

    /** Node id.
     */
    const RFNode::Id& nodeId() const { return node_.id; }

    const math::Extents2& extents() const { return node_.extents; }

    const std::string& srs() const { return node_.srs; }

    /** Full srs from registry.
     */
    const geo::SrsDefinition& srsDef() const;

    /** Distance from root.
     */
    Lod distanceFromRoot() const { return node_.id.lod - subtree_.id().lod; }

    /** Root lod.
     */
    Lod rootLod() const { return subtree_.id().lod; }

    /** Returns child node. Uses same child assignment as children() functiom
     *  children() from tileop.
     */
    NodeInfo child(Child child) const;

    bool valid() const { return node_.valid(); }

    bool productive() const { return node_.real(); }

    const RFTreeSubtree& subtree() const { return subtree_; }

    const registry::ReferenceFrame& referenceFrame() const {
        return *referenceFrame_;
    }

    /** Partial node is not fully inside valid bounds.
     *
     *  NB: node that is fully outside valid bounds is marked as invalid.
     */
    bool partial() const { return partial_; }

    typedef RFTreeSubtree::CoverageMask CoverageMask;
    typedef RFTreeSubtree::CoverageType CoverageType;

    /** Computes coverage mask:
     *    * invalid node: fully black
     *    * non-partial valid node: fully white
     *    * partial valid node: generated mask based on node constraints
     *
     * \param type pixel or grid registration
     * \param size size of mask in pixels
     * \param dilation number of dilation pixels, uses square element
     */
    CoverageMask coverageMask(CoverageType type, const math::Size2 &size
                              , unsigned int dilation = 0) const;

    /** Queries whether given point is inside node's valid area.  Point must be
     *  in node's SRS. Performs extra check to extents in parents SRS in case of
     *  partial node.
     */
    bool inside(const math::Point2 &point) const;

    enum CoveredArea { none, some, whole };

    /** Evaluates area covered by given mask (of given type).
     */
    CoveredArea checkMask(const CoverageMask &mask, CoverageType type
                          , unsigned int dilation = 0) const;

    /** Returns child node based on TileId. No checks is performed. Use only
     *  when you are sure that `child` is realy child of this node.
     */
    NodeInfo child(const TileId &child) const;

#ifdef GEO_HAS_GDAL
    /** Generates special SRS definition:
     *      merge(horizontal(sds),  vertical(navigation)
     *
     *  This SRS is used to interpret data stored inside navtiles.
     */
    registry::Srs navsds() const;
#endif // GEO_HAS_GDAL

    /** Generate list of nodeinfos from valid referenceframes' nodes.
     */
    static NodeInfo::list nodes(const registry::ReferenceFrame &referenceFrame
                                , const registry::Registry &reg
                                = registry::system);

    /** Generate list of nodeinfos from valid referenceframes' leaves.
     */
    static NodeInfo::list leaves(const registry::ReferenceFrame &referenceFrame
                                , const registry::Registry &reg
                                = registry::system);

private:
    /** Node info. Use with care.
     */
    NodeInfo(const registry::ReferenceFrame &referenceFrame
             , const RFNode &node
             , const registry::Registry &reg = registry::system);

    /** Associated reference frame
     */
    const registry::ReferenceFrame *referenceFrame_;

    /** Subtree this node belongs to
     */
    RFTreeSubtree subtree_;

    /** Node.
     */
    RFNode node_;

    /** Partial node is partially inside valid bounds.
     *
     *  NB: node that is fully outside valid bounds is marked as invalid!
     */
    bool partial_;
};

/** Checks compatibility of two nodes.
 *  Both nodes must be in the same subtree
 */
bool compatible(const NodeInfo &ni1, const NodeInfo &ni2);

// inline functions

inline NodeInfo::NodeInfo(const registry::ReferenceFrame &referenceFrame
                          , const registry::Registry &reg)
    : referenceFrame_(&referenceFrame)
    , subtree_(referenceFrame.root(), reg)
    , node_(subtree_.root()), partial_(!node_.real())
{}

inline bool compatible(const NodeInfo &ni1, const NodeInfo &ni2)
{
    return (ni1.subtree() == ni2.subtree());
}

UTILITY_GENERATE_ENUM_IO(RFTreeSubtree::CoverageType,
    ((pixel))
    ((grid))
)

} } // namespace vtslibs::vts

#endif // vtslibs_vts_nodeinfo_hpp_included_
