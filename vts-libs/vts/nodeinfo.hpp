#ifndef vadstena_libs_vts_nodeinfo_hpp_included_
#define vadstena_libs_vts_nodeinfo_hpp_included_

#include "./basetypes.hpp"

namespace vadstena { namespace vts {

class RFTreeSubtree {
public:
    RFTreeSubtree(const RFNode &root)
        : root_(&root)
    {}

    const RFNode& root() const { return *root_; }

    const RFNode::Id& id() const { return root_->id; }

    bool operator==(const RFTreeSubtree &other) const {
        return (root_ == other.root_);
    }

private:
    const RFNode *root_;
};

/** Reference frame node information.
 */
class NodeInfo {
public:
    /** Creates node info from reference frame and tileId.
     *
     * Root node is found in reference frame and then current node is derived.
     */
    NodeInfo(const registry::ReferenceFrame &referenceFrame
             , const TileId &tileId);

    /** Root node info.
     */
    NodeInfo(const registry::ReferenceFrame &referenceFrame);

    /** Node.
     */
    const RFNode& node() const { return node_; }

    /** Node id.
     */
    const RFNode::Id& nodeId() const { return node_.id; }

    const math::Extents2& extents() const { return node_.extents; }

    const std::string& srs() const { return node_.srs; }

    /** Distance from root.
     */
    Lod distanceFromRoot() const { return node_.id.lod - subtree_.id().lod; }

    /** Returns child node. Uses same child assignment as children() functiom
     *  children() from tileop.
     */
    NodeInfo child(Child child) const;

    bool valid() const { return node_.valid(); }

    const RFTreeSubtree& subtree() const { return subtree_; }

    const registry::ReferenceFrame& referenceFrame() const {
        return *referenceFrame_;
    }

private:
    /** Node info.
     */
    NodeInfo(const registry::ReferenceFrame &referenceFrame
             , const RFNode &node);

    /** Associated reference frame
     */
    const registry::ReferenceFrame *referenceFrame_;

    /** Subtree this node belongs to
     */
    RFTreeSubtree subtree_;

    /** Node.
     */
    RFNode node_;
};

/** Checks compatibility of two nodes.
 *  Both nodes must be in the same subtree
 */
bool compatible(const NodeInfo &ni1, const NodeInfo &ni2);

// inline functions

inline NodeInfo::NodeInfo(const registry::ReferenceFrame &referenceFrame)
    : referenceFrame_(&referenceFrame), subtree_(referenceFrame.root())
    , node_(subtree_.root())
{}

inline NodeInfo::NodeInfo(const registry::ReferenceFrame &referenceFrame
                          , const RFNode &node)
    : referenceFrame_(&referenceFrame), subtree_(node), node_(node)
{}

inline bool compatible(const NodeInfo &ni1, const NodeInfo &ni2)
{
    return (ni1.subtree() == ni2.subtree());
}

} } // namespace vadstena::vts

#endif // vadstena_libs_vts_nodeinfo_hpp_included_
