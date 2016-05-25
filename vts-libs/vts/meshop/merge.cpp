#include <memory>
#include <array>
#include <set>
#include <algorithm>

#include <boost/format.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/photo/photo.hpp>

#include "utility/streams.hpp"

#include "math/math.hpp"
#include "math/transform.hpp"
#include "math/filters.hpp"

#include "imgproc/uvpack.hpp"
#include "imgproc/scanconversion.hpp"
#include "imgproc/const-raster.hpp"
#include "imgproc/reconstruct.hpp"

#include "../opencv/atlas.hpp"

#include "../meshop.hpp"

/** Altas transformation background
 *
 * NB: patches in original textures are intended for OpenGL texturing which uses
 * bilinear interpolation thus we have no extra space around them. We need to
 * keep this in mind when transforming patches from one atlas space to the
 * another.
 *
 * Each patch is transformed from source atlas to destination atlas using map
 * from rotated rectangle to upright rectangle (using eclidean transformation
 * matrix).
 *
 * Trafo matrix is constructed in denormalized texture coordinates (i.e. grid
 * coordinates in the atlases.
 *
 * When mapping texture coordinates they are used as is.
 *
 * When mapping imagery coordinates are first converted from pixel registration
 * to grid registration and after convertion they are converted
 * back. I.e. shifted by 1/2 pixel to the bottom right and back again.
 *
 * Imagery is filtered by Catmul-Rom filter using information about source pixel
 * validity. Validity is established by rendering original texturing 2D mesh
 * into a binary mask which is dilated by one pixel.
 *
 * Since `scanconvert' algorithm is right-bottom bound we intentionally dilate
 * mesh only to the top and left side only.
 *
 * Face vertices are rendered into mask explicitly due to scanconvert's
 * deficiencies as well.
 *
 * NB: Reconstruct treats positive and negative weights from filter differently:
 * if full kernel are is not valid only positives are taken into account.
 */

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

/** Rasterizes triangle mesh into mask. Each pixel is dilated by one pixel to
 *  the left and top (because it is automatically dilated to the opposite side
 *  by scan convert rasterization algo.
 */
void rasterizeMask(cv::Mat &mask, const Faces &faces
                   , const math::Points2d &tc)
{
    const auto white(cv::Scalar(0xff));
    const cv::Rect bounds(0, 0, mask.cols, mask.rows);

    auto paint([&](int x, int y, int w, int h) -> void
    {
        cv::Rect r(x, y, w, h);
        cv::rectangle(mask, r & bounds, white, CV_FILLED, 4);
    });

    cv::Point3f tri[3];
    for (const auto &face : faces) {
        for (int i : { 0, 1, 2 }) {
            const auto &p(tc[face(i)]);
            tri[i] = { float(p(0)), float(p(1)), 0.f };
            paint(p(0), p(1), 1, 1);
        }

        // rasterize triangle
        std::vector<imgproc::Scanline> scanlines;
        imgproc::scanConvertTriangle(tri, 0, mask.rows, scanlines);

        for (const auto &sl : scanlines) {
            imgproc::processScanline(sl, 0, mask.cols
                                     , [&](int x, int y, float)
            {
                paint(x - 1, y - 1, 2, 2);
            });
        }
    }
}

class TextureInfo {
public:
    TextureInfo(const SubMesh &sm, const cv::Mat &texture)
        : tc_(denormalize(sm.tc, texture.size()))
        , faces_(&sm.facesTc), texture_(texture)
        , mask_(texture.rows, texture.cols, CV_8U, cv::Scalar(0x00))
    {
        rasterizeMask(mask_, *faces_, tc_);
    }

    const math::Points2d& tc() const { return tc_; }
    const Faces& faces() const { return *faces_; }
    const cv::Mat& texture() const { return texture_; }
    const cv::Mat& mask() const { return mask_; }

    const math::Point2d& uv(const Face &face, int index) const {
        return tc_[face(index)];
    }

    imgproc::UVCoord uvCoord(const Face &face, int index) const {
        const auto &p(uv(face, index));
        return imgproc::UVCoord(p(0), p(1));
    }

    const math::Point2d& uv(int index) const {
        return tc_[index];
    }

    typedef std::vector<TextureInfo> list;

private:
    math::Points2d tc_;
    const Faces *faces_;
    cv::Mat texture_;
    cv::Mat mask_;
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

/** Continuous mesh component.
 */
struct Component {
    /** Set of components's indices to faces
     */
    std::set<int> faces;

    /** Set of components's indices to texture coordinates
     */
    std::set<int> indices;

    /** UV rectangle.
     */
    imgproc::UVRect rect;

    /** Best rectangle circumscribed around this component.
     */
    cv::RotatedRect rrect;

    /** Transformation from rrect to output atlas.
     */
    math::Matrix4 rrectTrafo;

    typedef std::shared_ptr<Component> pointer;
    typedef std::vector<pointer> list;
    typedef std::set<pointer> set;

    Component() {}

    Component(int findex, const Face &face)
        : faces{findex}, indices{face(0), face(1), face(2)}
    {}

    void add(int findex, const Face &face) {
        faces.insert(findex);
        indices.insert({ face(0), face(1), face(2) });
    }

    void add(const Component &other) {
        faces.insert(other.faces.begin(), other.faces.end());
        indices.insert(other.indices.begin(), other.indices.end());
    }

    void copy(cv::Mat &tex, const cv::Mat &texture
              , const cv::Mat &mask) const;

    void mask(cv::Mat &mask, int block) const;

    imgproc::UVCoord adjustUV(const math::Point2 &p) const {
        auto np(math::transform(rrectTrafo, p));
        return imgproc::UVCoord(np(0) + rect.packX, np(1) + rect.packY);
    }

    void bestRect(const TextureInfo &tx);
};

/** Single submesh broken into components.
 */
struct ComponentInfo {
    /** Mesh broken to continuous components.
     */
    Component::set components;

    /** Mapping between texture coordinate index to owning component.
     */
    Component::list tcMap;

    /** Texturing information, i.e. the context of the problem.
     */
    const TextureInfo *tx;

    typedef std::vector<ComponentInfo> list;

    ComponentInfo(const TextureInfo &tx);

    void copy(cv::Mat &tex) const;

    void mask(cv::Mat &mask, int block) const;
};

ComponentInfo::ComponentInfo(const TextureInfo &tx)
    : tcMap(tx.tc().size()), tx(&tx)
{
    const auto &faces(tx.faces());
    Component::list fMap(tx.faces().size());

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

    // compute best fiting circumscribed rectangle for each component
    for (auto &c : components) {
        c->bestRect(tx);
    }
}

void ComponentInfo::copy(cv::Mat &tex) const
{
    for (const auto &c : components) {
        c->copy(tex, tx->texture(), tx->mask());
    }
}

void Component::bestRect(const TextureInfo &tx) {
    std::vector<cv::Point2f> points;
    points.reserve(indices.size());

    rect = {};

    math::Extents2 e(math::InvalidExtents{});

    // process all points
    for (const auto i : indices) {
        const auto &p(tx.uv(i));
        points.emplace_back(p(0), p(1));
        update(e, p);
    }

    if (1) {
        rrect = cv::minAreaRect(points);
    } else {
        auto c(math::center(e));
        auto s(math::size(e));

        rrect = cv::RotatedRect(cv::Point2f(c(0), c(1))
                                , cv::Size2f(s.width, s.height)
                                , 0.f);
    }

    rrect.size.width = std::ceil(rrect.size.width + 1.0);
    rrect.size.height = std::ceil(rrect.size.height + 1.0);

    {
        // calculate transformation matrix from original atlas to best
        // rectangle
        rrectTrafo = boost::numeric::ublas::identity_matrix<double>(4);

        const double a(rrect.angle * M_PI / 180.);
        rrectTrafo(0, 0) = rrectTrafo(1, 1) = std::cos(a);
        rrectTrafo(1, 0) = -(rrectTrafo(0, 1) = std::sin(a));

        const auto shift(math::transform
                         (rrectTrafo, math::Point2
                          (rrect.center.x, rrect.center.y)));

        rrectTrafo(0, 3) = -shift(0) + double(rrect.size.width) / 2.0;
        rrectTrafo(1, 3) = -shift(1) + double(rrect.size.height) / 2.0;
    }

    // create rectangle
    rect = {};
    rect.update({ 0.f, 0.f });
    // -2 due to: +-1 pixels added by packer for integral rectangles
    rect.update({ rrect.size.width - 2.f, rrect.size.height - 2.f });
}

/** Const raster for texture with mask.
 */
class TextureRaster : public imgproc::CvConstRaster<cv::Vec3b>
{
public:
    TextureRaster(const cv::Mat &texture, const cv::Mat &mask)
        : imgproc::CvConstRaster<cv::Vec3b>(texture), mask_(mask)
    {}

    bool valid(int x, int y) const {
        return (imgproc::CvConstRaster<cv::Vec3b>::valid(x, y)
                && mask_.at<std::uint8_t>(y, x));
    }

private:
    const cv::Mat &mask_;
};

void Component::copy(cv::Mat &tex, const cv::Mat &texture, const cv::Mat &mask)
    const
{
    const math::CatmullRom2 filter(2.0, 2.0);

    TextureRaster raster(texture, mask);

    // and invert: patch to original atlas
    math::Matrix4 trafo(math::matrixInvert(rrectTrafo));

    // copy texture from source best rectangle to output uv rectangle
    for (int y = 0; y < rrect.size.height; ++y) {
        for (int x = 0; x < rrect.size.width; ++x) {
            // compute destination point
            auto pos(math::transform
                     (trafo, math::Point2(x + 0.5, y + 0.5)));
            pos(0) -= 0.5; pos(1) -= 0.5;

            // reconstruct point using filter
            auto res(imgproc::reconstruct(raster, filter, pos));

            // store result
            tex.at<cv::Vec3b>(y + rect.packY, x + rect.packX)
                = cv::Vec3b(res[0], res[1], res[2]);
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
joinTextures(const TextureInfo::list &texturing);

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
        cv::imwrite(str(boost::format("%d-texture.png") % i)
                    , tx.back().texture());
        cv::imwrite(str(boost::format("%d-mask.png") % i)
                    , tx.back().mask());
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
    // mesh_->submeshes.push_back(oSubmeshes_[range.start]);
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
    std::tie(texture, sm.tc, sm.facesTc) = joinTextures(texturing(range));
    atlas_->add(texture);

    // housekeeping
    atlasEnd_ += range.size();
}

} // namespace

MeshAtlas mergeSubmeshes(const Mesh::pointer &mesh
                         , const RawAtlas::pointer &atlas
                         , int textureQuality)
{
    return MeshAtlasBuilder(mesh, atlas, textureQuality).result();
}

std::tuple<cv::Mat, math::Points2d, Faces>
joinTextures(const TextureInfo::list &texturing)
{
    const int inpaintMargin(4);

    std::tuple<cv::Mat, math::Points2d, Faces> res;
    auto &tex(std::get<0>(res));
    auto &tc(std::get<1>(res));
    auto &faces(std::get<2>(res));

    // rasterize faces to masks
    ComponentInfo::list cinfoList;
    for (const auto &tx : texturing) {
        cinfoList.emplace_back(tx);
    }

    // pack the patches
    imgproc::RectPacker packer;
    for (const auto &cinfo : cinfoList) {
        for (auto &c : cinfo.components) {
            packer.addRect(&c->rect);
        }
    }
    packer.pack();

    // background color
    const auto background(0
                          ? cv::Scalar(0x80, 0x80, 0x80)
                          : cv::Scalar(0x0, 0x0, 0x0));

    // create temporary texture a bit larger due to inpaint bug
    cv::Mat tmpTex(packer.height() + 2 * inpaintMargin
                    , packer.width() + 2 * inpaintMargin, CV_8UC3
                    , background);

    // fixes texture to point to proper matrix
    auto fixTexture([&](cv::Mat &ref)
    {
        // real texture
        tex = cv::Mat
            (ref, cv::Range(inpaintMargin, packer.height() + inpaintMargin)
             , cv::Range(inpaintMargin, packer.width() + inpaintMargin));
    });

    fixTexture(tmpTex);

    // create temporary mask a bit larger due to inpaint bug
    cv::Mat tmpMask(tmpTex.rows, tmpTex.cols, CV_8U, cv::Scalar(0x00));

    // real mask
    cv::Mat mask
        (tmpMask, cv::Range(inpaintMargin, packer.height() + inpaintMargin)
         , cv::Range(inpaintMargin, packer.width() + inpaintMargin));

    // denormalized result texture coordinates
    math::Points2d dnTc;

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
