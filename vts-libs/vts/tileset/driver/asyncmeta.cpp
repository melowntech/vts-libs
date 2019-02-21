/**
 * Copyright (c) 2019 Melown Technologies SE
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

#include "aggregated.hpp"
#include "runcallback.hpp"

namespace vtslibs { namespace vts { namespace driver {

namespace vs = vtslibs::storage;

namespace {

inline MetaNode::SourceReference
sourceReferenceFromFlags(TileIndex::Flag::value_type flags)
{
    return flags >> 16;
}

} // namespace

void AggregatedDriver::buildMeta(const TileId &tileId, std::time_t lastModified
                                 , const InputCallback &cb) const
{
    LOG(info4) << "Async buildMeta called.";

    const auto mbo(referenceFrame_.metaBinaryOrder);
    // parent tile at meta-binary-order levels above us
    const auto parentId(parent(tileId, mbo));
    // shrinked tile id to be used in meta-index
    const TileId shrinkedId(tileId.lod, parentId.x, parentId.y);

    // output metatile
    MetaTile ometa(tileId, mbo);

    const auto &tileIndex(tsi_.tileIndex);
    if (const auto *tree = tileIndex.tree(tileId.lod)) {
        tree->forEach
            (parentId.lod, parentId.x, parentId.y
             , [&](unsigned int x, unsigned int y, QTree::value_type value)
        {
            ometa.expectReference
                (TileId(tileId.lod, tileId.x + x, tileId.y + y)
                 , sourceReferenceFromFlags(value));
        }, QTree::Filter::white);
    }

    // load metatile, doesn't fail
    auto loadMeta([&](const TileId &tileId, const Driver::pointer &driver)
                  -> MetaTile
    {
        auto ms(driver->input(tileId, TileFile::meta, NullWhenNotFound));
        if (!ms) {
            // not found -> empty metanode (at right place)

            // TODO: check for node validity; all metatile nodes invalid ->
            // fine, empty metatile is ok; otherwise it should be an error
            return MetaTile(tileId, mbo);
        }
        return loadMetaTile(*ms, mbo, ms->name());
    });

    // start from zero so first round gets 1
    int idx(0);
    for (const auto &de : drivers_) {
        // next index
        ++idx;

        // check for metatile existence
        if (!de.metaIndex.get(shrinkedId)) { continue; }

        try {
            // load metatile from this tileset and update output metatile
            ometa.update(idx, loadMeta(tileId, de.driver));
        } catch (...) {
            return runCallback([&]() { return std::current_exception(); }, cb);
        }
    }

    if (ometa.empty()) {
        return runCallback([&]() -> std::exception_ptr
        {
            LOGTHROW(err1, vs::NoSuchFile)
                << "There is no metatile for " << tileId << ".";
            return {};
        }, cb);
    }

    // erase surface references if configured
    if (!surfaceReferences_) {
        ometa.for_each([&](const TileId&, MetaNode &node)
        {
            node.sourceReference = 0;
        });
    }

    return runCallback([&]()
    {
        // create in-memory stream
        auto fname(root() / str(boost::format("%s.%s")
                                % tileId % TileFile::meta));
        auto s(std::make_shared<StringIStream>
               (TileFile::meta, fname.string(), lastModified));

        // and serialize metatile
        ometa.save(s->sink());
        s->updateSize();

        // done
        return s;
    }, cb);
}

} } } // namespace vtslibs::vts::driver
