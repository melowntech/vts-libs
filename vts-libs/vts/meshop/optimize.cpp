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

#include <vector>
#include <unordered_map>

#include "dbglog/dbglog.hpp"

#include "../meshop.hpp"

namespace vtslibs { namespace vts {

/** Original code from vts-tools' importutil.
 * \author Jakub Cerveny <jakub.cerveny.melown.com>
 */
vts::SubMesh optimize(vts::SubMesh mesh)
{
    auto hash2 = [](const math::Point2 &p) -> std::size_t {
        return p(0)*218943212 + p(1)*168875421;
    };
    auto hash3 = [](const math::Point3 &p) -> std::size_t {
        return p(0)*218943212 + p(1)*168875421 + p(2)*385120205;
    };

    std::unordered_map<math::Point2, int, decltype(hash2)> map2(1024, hash2);
    std::unordered_map<math::Point3, int, decltype(hash3)> map3(1024, hash3);

    // assign unique indices to vertices and texcoords
    for (const auto &pt : mesh.vertices) {
        int &idx(map3[pt]);
        if (!idx) { idx = map3.size(); }
    }
    for (const auto &pt : mesh.tc) {
        int &idx(map2[pt]);
        if (!idx) { idx = map2.size(); }
    }

    // change face indices
    for (auto &f : mesh.faces) {
        for (int i = 0; i < 3; i++) {
            f(i) = map3[mesh.vertices[f(i)]] - 1;
        }
    }
    for (auto &f : mesh.facesTc) {
        for (int i = 0; i < 3; i++) {
            f(i) = map2[mesh.tc[f(i)]] - 1;
        }
    }

    // update vertices, tc
    mesh.vertices.resize(map3.size());
    for (const auto &item : map3) {
        mesh.vertices[item.second - 1] = item.first;
    }
    mesh.tc.resize(map2.size());
    for (const auto &item : map2) {
        mesh.tc[item.second - 1] = item.first;
    }

    return mesh;
}

} } // namespace vtslibs::vts
