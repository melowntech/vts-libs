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
#ifndef vtslibs_vts_geomextents_hpp
#define vtslibs_vts_geomextents_hpp

#include <limits>

#include <boost/variant.hpp>

#include "math/geometry_core.hpp"

#include "half/half.hpp"

#include "../storage/range.hpp"

typedef half_float::half hfloat;

namespace vtslibs { namespace vts {

struct GeomExtents {
    typedef storage::Range<float> ZRange;

    /** (SDS) height range.
     */
    ZRange z;

    /** (SDS) surrogate value (i.e. representative height of tile)
     */
    float surrogate;

    GeomExtents()
        : z(ZRange::emptyRange()), surrogate(invalidSurrogate)
    {}

    GeomExtents(float min, float max, float surrogate)
        : z(min, max), surrogate(surrogate)
    {}

    /** Compute surrogate as average from height range.
     */
    void makeAverageSurrogate() {
        surrogate = (z.min + z.max) / 2.f;
    }

    static constexpr float invalidSurrogate
        = -std::numeric_limits<float>::infinity();

    static bool validSurrogate(float surrogate) {
        return surrogate != invalidSurrogate;
    }

    bool validSurrogate() const { return validSurrogate(surrogate); }
};

inline bool empty(const GeomExtents &ge) { return ge.z.empty(); }

inline void update(GeomExtents &ge, float value) {
    update(ge.z, value);
}

inline void update(GeomExtents &ge, const GeomExtents &update) {
    ge.z = unite(ge.z, update.z);
}

} } // namespace vtslibs::vts

#endif // vtslibs_vts_geomextents_hpp

