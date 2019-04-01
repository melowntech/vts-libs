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

#include <map>
#include <vector>

#include "optimizemesh.hpp"

namespace vtslibs { namespace vts { namespace tools {

namespace {

template <typename PointT>
class Optimize {
public:
    typedef std::vector<PointT> Vertices;

    Optimize(Faces &faces, Vertices &vertices) {
        vertices_.reserve(vertices.size());
        for (auto &face : faces) {
            face(0) = vertex(vertices, face(0));
            face(1) = vertex(vertices, face(1));
            face(2) = vertex(vertices, face(2));
        }
        vertices.swap(vertices_);
    }

private:
    typedef std::map<PointT, int> PointMap;

    int vertex(const Vertices &vertices, int index) {
        const auto &value(vertices[index]);
        auto fmap(map_.find(value));
        if (fmap != map_.end()) { return fmap->second; }
        const auto i(map_.size());
        map_.insert(typename PointMap::value_type(value, i));
        vertices_.push_back(value);
        return i;
    }

    PointMap map_;
    Vertices vertices_;
};

} // namespace

/** In-place mesh optimization.
 */
void optimize(vts::Mesh &mesh)
{
    for (auto &sm : mesh) {
        Optimize<math::Point3d>(sm.faces, sm.vertices);
        Optimize<math::Point2d>(sm.facesTc, sm.tc);
        if (!sm.etc.empty()) {
            LOG(warn2) << "External texture coordinates not empty!";
        }
    }
}

} } } // namespace vtslibs::vts::tools
