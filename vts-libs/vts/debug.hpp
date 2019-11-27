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
#ifndef vtslibs_vts_debug_hpp_included_
#define vtslibs_vts_debug_hpp_included_

#include "tileindex.hpp"
#include "mapconfig.hpp"

namespace vtslibs { namespace vts {

struct DebugNode {
    std::uint32_t indexFlags;
    std::uint32_t metaFlags;
};

template <typename TileIndexType>
DebugNode getNodeDebugInfo(const TileIndexType &tileIndex
                           , const TileId &tileId);

template <typename TileIndexType, typename FlagManipulator>
DebugNode getNodeDebugInfo(const TileIndexType &tileIndex
                           , const TileId &tileId
                           , const FlagManipulator &flagManipulator);

void saveDebug(std::ostream &out, const DebugNode &debugNode);

// inlines

template <typename TileIndexType>
DebugNode getNodeDebugInfo(const TileIndexType &tileIndex
                           , const TileId &tileId)
{
    return getNodeDebugInfo(tileIndex, tileId
                            , [](TileIndex::Flag::value_type f) {
                                return f;
                            });
}

template <typename TileIndexType, typename FlagManipulator>
DebugNode getNodeDebugInfo(const TileIndexType &tileIndex
                           , const TileId &tileId
                           , const FlagManipulator &flagManipulator)
{
    DebugNode node;

    // get tile flags
    const auto tflags(flagManipulator(tileIndex.get(tileId)));

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

} } // namespace vtslibs::vts

#endif // vtslibs_vts_debug_hpp_included_
