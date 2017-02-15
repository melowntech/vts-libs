#include "jsoncpp/json.hpp"

#include "./tileflags.hpp"
#include "./metaflags.hpp"
#include "./debug.hpp"

namespace vtslibs { namespace vts {

DebugNode getNodeDebugInfo(const TileIndex &tileIndex, const TileId &tileId)
{
    DebugNode node;

    // get tile flags
    const auto tflags(tileIndex.get(tileId));

    // get child information
    MetaNode::Flag::value_type mflags(0);
    for (const auto &childId : vts::children(tileId)) {
        MetaNode::setChildFromId
            (mflags, childId
             , tileIndex.validSubtree(childId));
    }

    node.indexFlags = tflags;
    node.metaFlags = mflags;

    return node;
}

void saveDebug(std::ostream &out, const DebugNode &debugNode)
{
    Json::Value value(Json::objectValue);

    auto &flags(value["flags"] = Json::arrayValue);

    forEachFlag(TileFlags(debugNode.indexFlags)
                , [&](const std::string &flag) {
                    flags.append(flag);
                });

    forEachFlag(MetaFlags(debugNode.metaFlags)
                , [&](const std::string &flag) {
                    flags.append(flag);
                });

    Json::StyledStreamWriter().write(out, value);
}

} } // namespace vtslibs::vts
