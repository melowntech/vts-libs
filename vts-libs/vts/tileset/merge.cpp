#include <boost/format.hpp>

#include "../tileset-detail.hpp"
#include "../merge.hpp"
#include "./dump.hpp"
#include "../tileindex-io.hpp"
#include "../tileopext.hpp"

namespace vadstena { namespace vts {

void TileSet::paste(const list &update)
{
    TileIndices changed;

    const auto thisId(getProperties().id);
    auto &det(detail());

    const utility::Progress::ratio_t reportRatio(1, 100);
    for (const auto &src : update) {
        const auto id(src->getProperties().id);
        const auto name(str(boost::format("Pasting <%s> into <%s> ")
                            % id % thisId));

        LOG(info2) << "Copying all tiles from <" << id << ">.";
        auto &sdet(src->detail());

        utility::Progress progress(sdet.tileIndex.count());

        // process all tiles
        traverseTiles(sdet.tileIndex, [&](const TileId &tileId)
        {
            const auto *metanode(sdet.findMetaNode(tileId));
            if (!metanode) {
                LOG(warn2)
                    << "Cannot find metanode for tile " << tileId << "; "
                    << "skipping.";
                return;
            }

            // copy mesh and atlas
            for (auto type : { TileFile::mesh, TileFile::atlas }) {
                copyFile(sdet.driver->input(tileId, type)
                         , det.driver->output(tileId, type));
            }

            det.setMetaNode(tileId, *metanode);
            (++progress).report(reportRatio, name);
        });

        changed.push_back(&sdet.tileIndex);
    }

    // filter heightmap in bordering tiles in all pasted tile sets
    // TODO: implement me
    // det.filterHeightmap(changed);
}

} } // namespace vadstena::vts
