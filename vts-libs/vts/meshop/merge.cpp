#include <memory>
#include <array>
#include <set>
#include <algorithm>

#include <boost/format.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/photo/photo.hpp>

#include "utility/streams.hpp"

#include "imgproc/uvpack.hpp"
#include "imgproc/scanconversion.hpp"

#include "../opencv/atlas.hpp"

#include "../meshop.hpp"

namespace vadstena { namespace vts {

// TODO: move this stuff to own header
typedef std::vector<cv::Mat> MatList;

inline math::Point2d denormalize(const math::Point2d &p
                                 , const cv::Size &texSize)
{
    return { (p(0) * texSize.width), ((1.0 - p(1)) * texSize.height) };
}

inline math::Points2d denormalize(const math::Points2d &ps
                                  , const cv::Size &texSize)
{
    math::Points2d out;
    for (const auto &p : ps) {
        out.push_back(denormalize(p, texSize));
    }
    return out;
}

inline math::Point2d normalize(const imgproc::UVCoord &uv
                               , const cv::Size &texSize)
{
    return { uv.x / texSize.width
            , 1.0 - uv.y / texSize.height };
}

class TextureInfo {
public:
    TextureInfo(const SubMesh &sm, const cv::Mat &texture)
        : tc_(denormalize(sm.tc, texture.size()))
              , faces_(&sm.facesTc), texture_(texture)
    {}

    const math::Points2d& tc() const { return tc_; }
    const Faces& faces() const { return *faces_; }
    const cv::Mat& texture() const { return texture_; }

    const math::Point2d& uv(const Face &face, int index) const {
        return tc_[face(index)];
    }

    imgproc::UVCoord uvCoord(const Face &face, int index) const {
        const auto &p(uv(face, index));
        return imgproc::UVCoord(p(0), p(1));
    }

    typedef std::vector<TextureInfo> list;

private:
    math::Points2d tc_;
    const Faces *faces_;
    cv::Mat texture_;
};

namespace {
bool validMatPos(int x, int y, const cv::Mat &mat)
{
    return (x >= 0) && (x < mat.cols) && (y >= 0) && (y < mat.rows);
}

void clampMatPos(int &x, int &y, const cv::Mat &mat)
{
    if (x < 0) {
        x = 0;
    } else if (x >= mat.cols) {
        x = mat.cols - 1;
    }

    if (y < 0) {
        y = 0;
    } else if (y >= mat.rows) {
        y = mat.rows - 1;
    }
}

} // namespace

namespace {

bool inside(int x, int y, const cv::Mat &mat)
{
    return (x >= 0) && (x < mat.cols) && (y >= 0) && (y < mat.rows);
}

struct Component {
    std::set<int> faces;
    std::set<int> indices;
    imgproc::UVRect rect;

    typedef std::shared_ptr<Component> pointer;
    typedef std::vector<pointer> list;
    typedef std::set<pointer> set;

    Component() {}

    Component(int findex, const Face &face)
        : faces{findex}, indices{face(0), face(1), face(2)}
    {}

    void add(int findex, const Face &face) {
        faces.insert(findex);
        indices.insert({face(0), face(1), face(2)});
    }

    void add(const Component &other) {
        faces.insert(other.faces.begin(), other.faces.end());
        indices.insert(other.indices.begin(), other.indices.end());
    }

    void copy(cv::Mat &tex, const cv::Mat &texture) const;

    void mask(cv::Mat &mask, int block) const;

    imgproc::UVCoord adjustUV(const math::Point2 &p) const {
        imgproc::UVCoord uv(p(0), p(1));
        rect.adjustUV(uv);
        return uv;
    }
};

struct ComponentInfo {
    Component::set components;
    Component::list tcMap;
    Component::list fMap;
    const TextureInfo *tx;

    typedef std::vector<ComponentInfo> list;

    ComponentInfo(const TextureInfo &tx, float inflate);

    void copy(cv::Mat &tex) const;

    void mask(cv::Mat &mask, int block) const;
};

ComponentInfo::ComponentInfo(const TextureInfo &tx, float inflate)
    : tcMap(tx.tc().size()), fMap(tx.faces().size())
    , tx(&tx)
{
    const auto &tc(tx.tc());
    const auto &faces(tx.faces());

    typedef std::array<Component::pointer*, 3> FaceComponents;

    // sorts face per-vertex components by number of faces
    // no components are considered empty
    auto sortBySize([](FaceComponents &fc)
    {
        std::sort(fc.begin(), fc.end()
                  , [](const Component::pointer *l, Component::pointer *r)
                  -> bool
        {
            // prefer non-null
            if (!*l) { return !*r; }
            if (!*r) { return true; }

            // both are non-null; prefer longer
            return ((**l).faces.size() > (**r).faces.size());
        });
    });

    // NB: fc starts with valid data

    for (std::size_t i(0), e(faces.size()); i != e; ++i) {
        const auto &face(faces[i]);

        FaceComponents fc =
            { { &tcMap[face(0)], &tcMap[face(1)], &tcMap[face(2)] } };
        sortBySize(fc);

        auto assign([&](Component::pointer &owner, Component::pointer &owned)
                    -> void
        {
            // no-op
            if (owned == owner) { return; }

            if (!owned) {
                // no component assigned yet
                owned = owner;
                return;
            }

            // forget this component beforehand
            components.erase(owned);

            // grab owned by value to prevent overwrite
            const auto old(*owned);

            // merge components
            owner->add(old);

            // move everything to owner
            for (auto index : old.faces) { fMap[index] = owner; }
            for (auto index : old.indices) { tcMap[index] = owner; }
        });

        if (*fc[0] || *fc[1] || *fc[2]) {
            auto &owner(*fc[0]);
            fMap[i] = owner;
            owner->add(i, face);
            assign(owner, *fc[1]);
            assign(owner, *fc[2]);
        } else {
            // create new component
            components.insert
                (fMap[i] = *fc[0] = *fc[1] = *fc[2]
                 = std::make_shared<Component>(i, face));
        }
    }

    // compute bounding boxes
    {
        auto itcMap(tcMap.begin());
        for (const auto &p : tc) {
            (*itcMap++)->rect.update(imgproc::UVCoord(p(0), p(1)));
        }
    }

    if (inflate) {
        for (auto &c : components) {
            c->rect.inflate(inflate);
        }
    }
}

void ComponentInfo::copy(cv::Mat &tex) const
{
    for (const auto &c : components) {
        c->copy(tex, tx->texture());
    }
}

void Component::copy(cv::Mat &tex, const cv::Mat &texture)
    const
{
    // TODO: block copy via Mat::copyTo
    for (int y = 0; y < rect.height(); y++) {
        for (int x = 0; x < rect.width(); x++) {
            int sx(rect.x() + x), sy(rect.y() + y);
            clampMatPos(sx, sy, texture);

            int dx(rect.packX + x), dy(rect.packY + y);
            if (!validMatPos(dx, dy, tex)) {
                continue;
            }

            tex.at<cv::Vec3b>(dy, dx) = texture.at<cv::Vec3b>(sy, sx);
        }
    }
}

void ComponentInfo::mask(cv::Mat &mask, int block) const
{
    for (const auto &c : components) {
        c->mask(mask, block);
    }
}

void Component::mask(cv::Mat &mask, int block)
    const
{
    (void) mask;
    (void) block;

    // TODO: implement me
}

} // namespace

/** Joins textures into single texture.
 */
std::tuple<cv::Mat, math::Points2d, Faces>
joinTextures(const TextureInfo::list &texturing, float inflate = 0);

namespace {

typedef std::tuple<Mesh::pointer, Atlas::pointer> MeshAtlas;

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
    bool empty() const { return !size(); }

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
                     , const RawAtlas::pointer &atlas
                     , int textureQuality)
        : textureQuality_(textureQuality)
        , originalMesh_(mesh), originalAtlas_(atlas)
        , oSubmeshes_(mesh->submeshes)
        , meshEnd_(0), atlasEnd_(0)
    {
        merge(groupSubmeshes(mesh->submeshes, atlas->size()));
    }

    MeshAtlas result() const {
        return MeshAtlas
            (changedMesh() ? mesh_ : originalMesh_
             , changedAtlas() ? atlas_ : Atlas::pointer(originalAtlas_));
    }

private:
    void merge(const Range::list &ranges);
    void merge(const Range &range);
    void ensureChanged(const Range &range);
    bool changedMesh() const { return bool(mesh_); }
    bool changedAtlas() const { return bool(atlas_); }

    void pass(const Range &range);
    SubMesh& mergeNonTextured(const Range &range);
    void mergeTextured(const Range &range);

    TextureInfo::list texturing(const Range &range) const;

    const int textureQuality_;
    const Mesh::pointer originalMesh_;
    const RawAtlas::pointer originalAtlas_;
    const SubMesh::list oSubmeshes_;
    Mesh::pointer mesh_;
    opencv::HybridAtlas::pointer atlas_;
    std::size_t meshEnd_;
    std::size_t atlasEnd_;
};

TextureInfo::list MeshAtlasBuilder::texturing(const Range &range) const
{
    TextureInfo::list tx;
    for (auto i(range.start); i < range.end; ++i) {
        tx.emplace_back
            (originalMesh_->submeshes[i]
             , opencv::HybridAtlas::imageFromRaw(originalAtlas_->get(i)));
    }
    return tx;
}

void MeshAtlasBuilder::merge(const Range::list &ranges)
{
    for (const auto &range : ranges) { merge(range); }
}

void MeshAtlasBuilder::merge(const Range &range)
{
    // sanity check
    if (range.empty()) { return; }

    // textured range - always merge
    if (range.textured) {
        mergeTextured(range);
        return;
    }

    // non textured
    if (range.size() == 1) {
        // just one, pass as is
        pass(range);
        return;
    }

    // merge non-textured submeshes
    mergeNonTextured(range);
}

void MeshAtlasBuilder::ensureChanged(const Range &range)
{
    if (!changedMesh()) {
        // copy mesh and trim submeshes
        mesh_ = std::make_shared<Mesh>(*originalMesh_);
        mesh_->submeshes.resize(meshEnd_);
    }

    if (range.textured && !changedAtlas()) {
        // convert atlas
        atlas_ = std::make_shared<opencv::HybridAtlas>
            (atlasEnd_, *originalAtlas_, textureQuality_);
    }
}

void MeshAtlasBuilder::pass(const Range &range)
{
    if (changedMesh()) {
        // copy submeshes
        mesh_->submeshes.insert
            (mesh_->submeshes.end()
             , &originalMesh_->submeshes[range.start]
             , &originalMesh_->submeshes[range.end]);
    }

    if (range.textured && changedAtlas()) {
        // copy atlas
        for (auto i(range.start); i != range.end; ++i) {
            atlas_->add(originalAtlas_->get(i));
        }
    }

    // move markers
    meshEnd_ += range.size();
    if (range.textured) { atlasEnd_ += range.size(); }
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

SubMesh& MeshAtlasBuilder::mergeNonTextured(const Range &range)
{
    // more than one submeshes that can be joined -> join them
    ensureChanged(range);

    // clone first submesh to join
    mesh_->submeshes.push_back(oSubmeshes_[range.start]);
    auto &sm(mesh_->submeshes.back());

    // drop texturing info (if any)
    sm.tc.clear();
    sm.facesTc.clear();

    for (auto i(range.start + 1); i < range.end; ++i) {
        auto &src(oSubmeshes_[i]);

        // append external texture coordinates
        append(sm.etc, src.etc);
        appendFaces(sm.faces, src.faces, sm.vertices.size());
        append(sm.vertices, src.vertices);
    }

    meshEnd_ += range.size();

    return sm;
}

void MeshAtlasBuilder::mergeTextured(const Range &range)
{
    // more than one submeshes that can be joined -> join them

    // join non-texture information (same as above)
    auto &sm(mergeNonTextured(range));

    // and join textures
    cv::Mat texture;
    std::tie(texture, sm.tc, sm.facesTc) = joinTextures(texturing(range), 2);
    atlas_->add(texture);

    // housekeeping
    atlasEnd_ += range.size();
}

void rasterizeMask(cv::Mat &mask, const Faces &faces
                   , const math::Points2d &tc)
{
    cv::Point3f tri[3];
    for (const auto &face : faces) {
        for (int i : { 0, 1, 2 }) {
            const auto &p(tc[face(i)]);
            tri[i] = { float(p(0)), float(p(1)), 0.f };
            // make sure the vertices are always marked
            int x(round(tri[i].x)), y(round(tri[i].y));
            if (inside(x, y, mask)) {
                mask.at<std::uint8_t>(y, x) = 0x00;
            }
        }

        // rasterize triangle
        std::vector<imgproc::Scanline> scanlines;
        imgproc::scanConvertTriangle(tri, 0, mask.rows, scanlines);

        for (const auto &sl : scanlines) {
            imgproc::processScanline(sl, 0, mask.cols
                                     , [&](int x, int y, float)
            {
                mask.at<std::uint8_t>(y, x) = 0x00;
            });
        }
    }
}

} // namespace

MeshAtlas mergeSubmeshes(const Mesh::pointer &mesh
                         , const RawAtlas::pointer &atlas
                         , int textureQuality)
{
    return MeshAtlasBuilder(mesh, atlas, textureQuality).result();
}

std::tuple<cv::Mat, math::Points2d, Faces>
joinTextures(const TextureInfo::list &texturing, float inflate)
{
    const int inpaintMargin(4);

    std::tuple<cv::Mat, math::Points2d, Faces> res;
    auto &tex(std::get<0>(res));
    auto &tc(std::get<1>(res));
    auto &faces(std::get<2>(res));

    // rasterize faces to masks
    ComponentInfo::list cinfoList;
    for (const auto &tx : texturing) {
        cinfoList.emplace_back(tx, inflate);
    }

    // pack the patches
    imgproc::RectPacker packer;
    for (const auto &cinfo : cinfoList) {
        for (auto &c : cinfo.components) {
            packer.addRect(&c->rect);
        }
    }
    packer.pack();

    // create temporary texture a bit larger due to inpaint bug
    cv::Mat tmpTex(packer.height() + 2 * inpaintMargin
                    , packer.width() + 2 * inpaintMargin, CV_8UC3
                    , cv::Scalar(0x80, 0x80, 0x80));

    // fixes texture to point to proper matrix
    auto fixTexture([&](cv::Mat &ref)
    {
        // real texture
        tex = cv::Mat
            (ref, cv::Range(inpaintMargin, packer.height() + inpaintMargin)
             , cv::Range(inpaintMargin, packer.width() + inpaintMargin));
    });

    fixTexture(tmpTex);

    // denormalized result texture coordinates
    math::Points2d dnTc;

    // create temporary mask a bit larger due to inpaint bug
    cv::Mat tmpMask(tmpTex.rows, tmpTex.cols, CV_8U,cv::Scalar(0x00));

    // real mask
    cv::Mat mask
        (tmpMask, cv::Range(inpaintMargin, packer.height() + inpaintMargin)
         , cv::Range(inpaintMargin, packer.width() + inpaintMargin));

        // process patches
    {
        auto ts(tex.size());

        for (const auto &cinfo : cinfoList) {
            const auto &tx(*cinfo.tx);

            // copy patches from this texture
            cinfo.copy(tex);
            cinfo.mask(mask, 16);

            auto itcMap(cinfo.tcMap.cbegin());
            auto tcOffset(tc.size());

            for (const auto &oldUv : tx.tc()) {
                auto uv((*itcMap++)->adjustUV(oldUv));
                dnTc.emplace_back(double(uv.x), double(uv.y));
                tc.push_back(normalize(uv, ts));
            }

            // add texture faces from this texture
            appendFaces(faces, tx.faces(), tcOffset);
        }
    }

    // TODO: make faster, then make default
    if (0) {
        // rasterize valid triangles
        rasterizeMask(mask, faces, dnTc);

        cv::Mat tmp(tmpTex.rows, tmpTex.cols, CV_8UC3);
        cv::inpaint(tmpTex, tmpMask, tmp, 3, cv::INPAINT_TELEA);
        fixTexture(tmp);
    }

    // done
    return res;
}

} } // namespace vadstena::vts
