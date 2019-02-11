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
#ifndef vtslibs_vts_glue_hpp_included_
#define vtslibs_vts_glue_hpp_included_

#include <string>

#include "math/geometry_core.hpp"

#include "utility/supplement.hpp"
#include "utility/enum-io.hpp"

#include "basetypes.hpp"

namespace vtslibs { namespace vts {

struct Glue : public utility::Supplement<Glue> {
    typedef TilesetIdList Id;
    Id id;
    std::string path;

    typedef std::map<Id, Glue> map;
    typedef std::vector<Glue> list;
    typedef std::vector<Id> Ids;
    typedef std::set<Id> IdSet;


    Glue() {}
    Glue(const Id &id) : id(id) {}
    Glue(const Id &id, const std::string &path) : id(id), path(path) {}

    /** Returns true if glue references given tileset
     */
    bool references(const std::string &tilesetId) const;

    static bool references(const Glue::Id &id, const std::string &tilesetId);
};

/** Tileset with its glues.
 */
struct TileSetGlues : public utility::Supplement<TileSetGlues> {
    struct EnhancedGlue : Glue {
        template <typename ...Args>
        EnhancedGlue(Args &&...args) : Glue(std::forward<Args>(args)...) {}

        typedef std::vector<int> Indices;

        /** Glue-local surface index to storage surface index.
         */
        Indices indices;

        typedef std::vector<EnhancedGlue> list;
    };

    TilesetId tilesetId;

    EnhancedGlue::list glues;

    TileSetGlues(const TilesetId &tilesetId)
        : tilesetId(tilesetId)
    {}

    TileSetGlues(const TilesetId &tilesetId, const Glue::list glues)
        : tilesetId(tilesetId), glues(glues.begin(), glues.end())
    {}

    typedef std::vector<TileSetGlues> list;
};

/** Calculates glue priority order for given set of tilesets.
 *
 * Tile set order is given by order of input list!
 *
 * \param mapping between tilesets and their glues.
 */
TileSetGlues::list glueOrder(const TileSetGlues::list &in);

// inlines

} } // namespace vtslibs::vts

#endif // vtslibs_vts_glue_hpp_included_
