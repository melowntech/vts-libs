#include <algorithm>

#include "dbglog/dbglog.hpp"

#include "./nodeinfo.hpp"
#include "./tileop.hpp"
#include "./csconvertor.hpp"

namespace vadstena { namespace vts {

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
    auto nid(rfNodeId(tileId));

    // clone root
    auto node(subtreeRoot);

    // set id
    node.id = nid;

    // set extents
    node.extents = makeExtents(subtreeRoot, nid);

    return node;
}

bool checkPartial(const RFTreeSubtree &subtree_, RFNode &node)
{
    auto valid(subtree_.valid(node));
    if (valid) {
        return false;
    } else if (!valid) {
        // invalid node -> invalidate
        node.invalidate();
        return false;
    }

    // indeterminate -> valid but partial
    return true;
}

} // namespace

inline NodeInfo::NodeInfo(const registry::ReferenceFrame &referenceFrame
                          , const RFNode &node)
    : referenceFrame_(&referenceFrame), subtree_(node), node_(node)
    , partial_(checkPartial(subtree_, node_))
{}

NodeInfo::NodeInfo(const registry::ReferenceFrame &referenceFrame
                   , const TileId &tileId)
    : referenceFrame_(&referenceFrame)
    , subtree_(referenceFrame_->findSubtreeRoot(rfNodeId(tileId)))
    , node_(makeNode(subtree_.root(), tileId))
    , partial_(checkPartial(subtree_, node_))
{}

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

    // NB: this works only for manual division
    if (const auto *childNode = referenceFrame_->find(childId, std::nothrow)) {
        // we have new subtree root
        return { *referenceFrame_, *childNode };
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

class RFTreeSubtree::Sampler : boost::noncopyable {
public:
    Sampler(const RFNode &root)
        : conv_(root.srs, root.constraints->extentsSrs)
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
    if (!sampler_) { sampler_ = std::make_shared<Sampler>(*root_); }
    return true;
}

boost::tribool RFTreeSubtree::valid(const RFNode &node) const
{
    // try to init sampler; if sampler cannot be initialized, we are fully
    // inside
    if (!initSampler()) { return true; }

    class Checker {
    public:
        Checker(const Sampler &sampler)
            : sampler_(sampler), inside_(false), outside_(false)
        {}

        bool operator()(const math::Point2 &p) {
            if (sampler_.inside(p)) {
                ++inside_;
            } else {
                ++outside_;
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

NodeInfo::CoverageMask
NodeInfo::coverageMask(CoverageType type, const math::Size2 &size) const
{
    if (!valid()) {
        return CoverageMask(size, CoverageMask::InitMode::EMPTY);
    }

    if (!partial_) {
        return CoverageMask(size, CoverageMask::InitMode::FULL);
    }

    return subtree_.coverageMask(type, size, node_);
}

RFTreeSubtree::CoverageMask
RFTreeSubtree::coverageMask(CoverageType type, const math::Size2 &size
                            , const RFNode &node) const
{
    if (!initSampler()) {
        // no sampler -> no constraints -> full mask
        return CoverageMask(size, CoverageMask::InitMode::FULL);
    }

    CoverageMask mask(size, CoverageMask::InitMode::EMPTY);

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

    // number of steps in each direction
    const auto steps([&]() -> math::Size2
    {
        // pixel coordinates: leave
        if (type == CoverageType::pixel) { return size; }

        // grid: add one more step
        return { size.width + 1, size.height + 1 };
    }());

    const auto gs(math::size(grid));
    math::Size2f ps(gs.width / size.width, gs.height / size.height);
    const auto ref(ul(grid));

    // sample whole mask
    const auto &sampler(*sampler_);
    for (int j(0); j < steps.height; ++j) {
        double y(ref(1) - j * ps.height);
        for (int i(0); i < steps.width; ++i) {
            if (sampler.inside(math::Point2(ref(0) + i * ps.width, y))) {
                mask.set(i, j, true);
            }
        }
    }

    return mask;
}

} } // namespace vadstena::vts
