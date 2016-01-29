#include "../meshop.hpp"

namespace vadstena { namespace vts {

typedef std::tuple<Mesh::pointer, Atlas::pointer> MeshAtlas;

namespace {

/** TODO: observe other differences in submeshes (texture mode, external
 *  texture coordinates, texture layer, UV area factor etc.
 *  TODO: check for limits in vertex/face count
 *  Not so simple :)
 */
struct Range {
    std::size_t start;
    std::size_t end;
    SubMesh::SurfaceReference surface;
    bool textured;

    std::size_t size() const { return end - start; }

    typedef std::vector<Range> list;

    Range(std::size_t start, SubMesh::SurfaceReference surface
          , bool textured)
        : start(start), end(start), surface(surface)
        , textured(textured)
    {}
};

/** Precondition: submeshes from the same source are grouped.
 *
 *  TODO: Sort submeshes (and atlas!) so the above precondition is met.
 */
Range::list groupSubmeshes(const SubMesh::list &sms, std::size_t textured)
{
    Range::list out;

    auto process([&](std::size_t i, std::size_t e, bool textured)
    {
        while (i != e) {
            // find region that have the same source
            out.emplace_back(i, sms[i].surfaceReference, textured);
            auto &range(out.back());

            while ((range.end != e)
                   && (range.surface == sms[range.end].surfaceReference))
            {
                ++i;
                ++range.end;
            }
        }
    });

    // process in two parts: textured and untextured
    process(0, textured, true);
    process(textured, sms.size(), false);

    return out;
}

class MeshAtlasBuilder {
public:
    MeshAtlasBuilder(const Mesh::pointer &mesh
                     , const Atlas::pointer &atlas)
        : original_(mesh, atlas)
        , oSubmeshes_(mesh->submeshes)
        , end_(0)
    {
        merge(groupSubmeshes(mesh->submeshes, atlas->size()));
    }

    MeshAtlas result() const {
        if (changed()) {
            return MeshAtlas(mesh_, std::get<1>(original_));
        }
        return original_;
    }

private:
    void merge(const Range::list &ranges);
    void merge(const Range &range);
    void ensureChanged();
    bool changed() const { return bool(mesh_); }

    void pass(const Range &range);
    void mergeNonTextured(const Range &range);

    const MeshAtlas original_;
    const SubMesh::list oSubmeshes_;
    Mesh::pointer mesh_;
    Atlas::pointer atlas_;
    std::size_t end_;
};

void MeshAtlasBuilder::merge(const Range::list &ranges)
{
    for (const auto &range : ranges) {
        LOG(info4)
            << "Processing range ["
            << range.start << ", " << range.end
            << "]: surface=" << int(range.surface) << ", textured="
            << std::boolalpha << range.textured << ".";

        merge(range);
    }
}

void MeshAtlasBuilder::merge(const Range &range)
{
    // simple range, nothing to do
    if (range.size() <= 1) {
        pass(range);
        return;
    }

    if (range.textured) {
        // no support for textured meshes now
        pass(range);
        return;
    }

    mergeNonTextured(range);
}

void MeshAtlasBuilder::ensureChanged()
{
    if (changed()) { return; }
    // copy mesh and trim submeshes
    mesh_ = std::make_shared<Mesh>(*std::get<0>(original_));
    mesh_->submeshes.resize(end_);

    // no touching atlas
}

void MeshAtlasBuilder::pass(const Range &range)
{
    if (changed()) {
        // copy submeshes
        mesh_->submeshes.insert(mesh_->submeshes.end()
                                , &mesh_->submeshes[range.start]
                                , &mesh_->submeshes[range.end]);
    }

    // move marker
    end_ += range.size();
}

template <typename T>
inline void append(std::vector<T> &v1, const std::vector<T> &v2)
{
    v1.insert(v1.end(), v2.begin(), v2.end());
}

void appendFaces(Faces &out, const Faces &in, std::size_t shift)
{
    for (const auto &f : in) {
        out.emplace_back(f(0) + shift, f(1) + shift, f(2) + shift);
    }
}

void MeshAtlasBuilder::mergeNonTextured(const Range &range)
{
    // more than one submeshes that can be joined -> join them
    ensureChanged();

    // clone first submesh to join
    mesh_->submeshes.push_back(oSubmeshes_[range.start]);
    auto &sm(mesh_->submeshes.back());
    ++end_;

    for (auto i(range.start + 1); i < range.end; ++i) {
        auto &src(oSubmeshes_[i]);

        // append external texture coordinates
        append(sm.etc, src.etc);
        appendFaces(sm.faces, src.faces, sm.vertices.size());
        append(sm.vertices, src.vertices);

        // no texture coordinates nor texture faces -> cool
    }
}

} // namespace

MeshAtlas mergeSubmeshes(const Mesh::pointer &mesh
                         , const Atlas::pointer &atlas)
{
    return MeshAtlasBuilder(mesh, atlas).result();
}

} } // namespace vadstena::vts
