#include "./basetypes.hpp"
#include "./tileop.hpp"

namespace vadstena { namespace vts {

namespace {

math::Extents2 makeExtents(const RFNode &rootNode, const RFNode::Id &nodeId)
{
    // determine tile extents
    auto tc(tileCount(nodeId.lod - rootNode.id.lod));
    auto rs(size(rootNode.extents));
    math::Size2f ts(rs.width / tc, rs.height / tc);
    return  math::Extents2
        (rootNode.extents.ll(0) + nodeId.x * ts.width
         , rootNode.extents.ur(1) - (nodeId.y + 1) * ts.height
         , rootNode.extents.ll(0) + (nodeId.x + 1) * ts.width
         , rootNode.extents.ur(1) - nodeId.y * ts.height);
}

RFNode makeNode(const RFNode &subtreeRoot
                , const TileId &tileId)
{
    auto nid(rfNodeId(tileId));

    // clone root
    auto node(subtreeRoot);

    // set id
    node.id = nid;

    // set extents
    node.extents = makeExtents(subtreeRoot, nid);

    if (node.boundLayerLod) {
        // update bound layer lod
        *node.boundLayerLod += (node.id.lod - subtreeRoot.id.lod);
    }
    return node;
}

} // namespace

NodeInfo::NodeInfo(const registry::ReferenceFrame &referenceFrame
                   , const TileId &tileId)
    : referenceFrame(&referenceFrame)
    , subtreeRoot(&referenceFrame.findSubtreeRoot(rfNodeId(tileId)))
    , node(makeNode(*subtreeRoot, tileId))
{}

NodeInfo NodeInfo::child(int childNum) const
{
    RFNode::Id childId(node.id);
    ++childId.lod;

    switch (childNum) {
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
            << "Invalid child number (" << childNum << ").";
        break;
    }

    if (const auto *childNode = referenceFrame->find(childId, std::nothrow)) {
        // we have new subtree root
        return { *referenceFrame, *childNode };
    }

    // divide current node's extents in half in both directions
    NodeInfo child(*this);
    child.node.id = childId;

    // size of extents
    auto es(size(node.extents));
    // and halve it
    es.width /= 2.0;
    es.height /= 2.0;

    // no need to check childNum since it was checked above
    auto &extents(child.node.extents);
    switch (childNum) {
    case 0: // upper-left
        extents.ur(0) = extents.ll(0) + es.width;
        extents.ur(1) = extents.ll(1) + es.height;
        break;

    case 1: // upper-right
        extents.ll(0) = extents.ur(0) - es.width;
        extents.ur(1) = extents.ll(1) + es.height;
        break;

    case 2: // lower-left
        extents.ur(0) = extents.ll(0) + es.width;
        extents.ll(1) = extents.ur(1) - es.height;
        break;

    case 3: // lower-right
        extents.ll(0) = extents.ur(0) - es.width;
        extents.ll(1) = extents.ur(1) - es.height;
        break;
    }

    return child;
}

} } // namespace vadstena::vts
