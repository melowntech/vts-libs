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
#include <map>

#include "dbglog/dbglog.hpp"

#include "../meshop.hpp"

namespace vtslibs { namespace vts {

/** Adds skirt at mesh border.
 *
 *  If mesh has internal texture coordinates (tc) then skirt faces reuse them.
 *
 *  If mesh has external texture coordinates (etc) then skirt faces reuse them
 *  as well.
 */
void addSkirt(EnhancedSubMesh &em
              , const MeshVertexConvertor &convertor
              , const SkirtVectorCallback &skirtVector)
{
    auto &mesh(em.mesh);

    auto &faces(mesh.faces);
    auto &vertices(mesh.vertices);
    auto &projected(em.projected);
    auto *etc(mesh.etc.empty() ? nullptr : &mesh.etc);

    auto &facesTc(mesh.facesTc);
    bool hasTc(!facesTc.empty());

    enum class Status { fw, bw, bi };

    struct Edge {
        int v1;
        int v2;
        int t1;
        int t2;
        mutable Status status;

        Edge(int pv1, int pv2, int pt1 = 0, int pt2 = 0)
            : v1(std::min(pv1, pv2)), v2(std::max(pv1, pv2))
            , t1((pv1 <= pv2) ? pt1 : pt2), t2((pv1 <= pv2) ? pt2 : pt1)
            , status((pv1 <= pv2) ? Status::fw : Status::bw)
        {}

        void update(int pv1, int pv2) const {
            if ((pv1 <= pv2) && (status == Status::bw)) {
                status = Status::bi;
            }
            if ((pv1 > pv2) && (status == Status::fw)) {
                status = Status::bi;
            }
        }

        bool operator<(const Edge & edge) const {
            return (v1 < edge.v1) || ((v1 == edge.v1) && (v2 < edge.v2));
        }
    };


    typedef std::set<Edge> Edges;
    typedef std::map<int, int> DownMap;

    Edges edges;
    DownMap vdownmap;

    // find odd edges
    {
        auto ifacesTc(facesTc.begin());
        Face dummy;
        for (const Face &f : faces) {
            auto &ftc(hasTc ? *ifacesTc++ : dummy);

            edges.insert(Edge(f(0), f(1), ftc(0), ftc(1)))
                .first->update(f(0), f(1));
            edges.insert(Edge(f(1), f(2), ftc(1), ftc(2)))
                .first->update(f(1), f(2));
            edges.insert(Edge(f(2), f(0), ftc(2), ftc(0)))
                .first->update(f(2), f(0));
        }
    }

    // iterate through edges
    int even(0);
    int odd(0);

    const math::Point3d zero;

    // get down vertex for given vertex
    const auto getDownVertex([&](int v) -> int
    {
        // try to find
        auto fvdownmap(vdownmap.find(v));
        if (fvdownmap != vdownmap.end()) { return fvdownmap->second; }

        // find out whether we need to create new vertex
        auto vertex(projected[v]);
        const auto vector(skirtVector(vertex));
        if (vector == zero) { return vdownmap[v] = v; }

        // not found, add new, vertex, copy etc if enabled
        const auto index(vertices.size());

        vdownmap[v] = index;

        // create new (projected) vertex
        vertex += vector;
        // add to projected vertices
        projected.push_back(vertex);
        // and add physical version to mesh vertices
        vertices.push_back(convertor.vertex(vertex));

        if (etc) {
            etc->push_back(convertor.etc(projected.back()));
        }

        // done
        return index;
    });

    for (const Edge &edge : edges) {
        if (edge.status == Status::bi) {
            ++even;
            continue;
        }

        // get (and optionally add new) vertex for edge ends
        const auto dv1(getDownVertex(edge.v1));
        const auto dv2(getDownVertex(edge.v2));

        // add new faces
        switch (edge.status) {
        case Status::fw:
            if (dv1 != edge.v1) {
                faces.emplace_back(dv1, edge.v2, edge.v1);
                if (hasTc) {
                    facesTc.emplace_back(edge.t1, edge.t2, edge.t1);
                }
            }
            if (dv2 != edge.v2) {
                faces.emplace_back(dv1, dv2, edge.v2);
                if (hasTc) {
                    facesTc.emplace_back(edge.t1, edge.t2, edge.t2);
                }
            }
            break;

        case Status::bw:
            if (dv1 != edge.v1) {
                faces.emplace_back(dv1, edge.v1, edge.v2);
                if (hasTc) {
                    facesTc.emplace_back(edge.t1, edge.t1, edge.t2);
                }
            }
            if (dv2 != edge.v2) {
                faces.emplace_back(dv1, edge.v2, dv2);
                if (hasTc) {
                    facesTc.emplace_back(edge.t1, edge.t2, edge.t2);
                }
            }
            break;

        case Status::bi: break; // never reached
        }

        ++odd;
    }

    LOG(info1) << "Skirt: " << " even edges, " << odd << " odd edges.";

    (void) facesTc;
}

} } // namespace vtslibs::vts
