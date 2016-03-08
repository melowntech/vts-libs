#include <algorithm>

#include <boost/range/adaptor/reversed.hpp>

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

    return node;
}

} // namespace

NodeInfo::NodeInfo(const registry::ReferenceFrame &referenceFrame
                   , const TileId &tileId)
    : referenceFrame(&referenceFrame)
    , subtreeRoot(&referenceFrame.findSubtreeRoot(rfNodeId(tileId)))
    , node(makeNode(*subtreeRoot, tileId))
{}

NodeInfo NodeInfo::child(Child childDef) const
{
    // build child id from this node and index
    RFNode::Id childId(node.id);
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
            << node.id << ".";
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

    return child;
}

TileSetGlues::list glueOrder(const TileSetGlues::list &in)
{
    // creata tileset alphabet
    std::map<TilesetId, int> alphabet;
    {
        int d(0);
        for (const auto &tsg : boost::adaptors::reverse(in)) {
            alphabet[tsg.tilesetId] = d;
            LOG(info2) << "Depth <" << tsg.tilesetId << ">: " << d << ".";
            ++d;
        }
    }

    TileSetGlues::list out;

    for (const auto &tsg : in) {
        auto compareGlues([&](const Glue &l, const Glue &r) -> bool
        {
            auto lr(boost::adaptors::reverse(l.id));
            auto rr(boost::adaptors::reverse(r.id));
            auto lrb(begin(lr)), lre(end(lr));
            auto rrb(begin(rr)), rre(end(rr));

            while ((lrb != lre) && (rrb != rre)) {
                // grab characters
                auto lc(alphabet[*lrb++]);
                auto rc(alphabet[*rrb++]);

                if (rc < lc) {
                    return true;
                } else if (lc < rc) {
                    return false;
                }
                // same character at the same position -> next one
            }

            // one id is prefix or the other (or both are the same, which is
            // unlikely) -> longest is less
            return r.id.size() <= l.id.size();
        });

        out.push_back(tsg);
        std::sort(out.back().glues.begin(), out.back().glues.end()
                  , compareGlues);
    }

    return out;
}

} } // namespace vadstena::vts
