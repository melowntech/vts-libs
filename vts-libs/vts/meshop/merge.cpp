#include <boost/format.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

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

typedef std::vector<imgproc::UVRect> UVRects;
typedef std::vector<int> Assigment;
typedef std::vector<cv::Point> CvPoints;

typedef std::vector<CvPoints> CvContours;

class Patches {
public:
    cv::Mat mask;
    UVRects rects;
    Assigment faceAssignment;
    Assigment tcAssignment;
    const cv::Mat& texture() const { return *texture_; }

    Patches(const cv::Mat &texture)
        : mask(texture.rows + 1, texture.cols + 1, CV_8U, cv::Scalar(0))
        , texture_(&texture)
    {}

    void copy(cv::Mat &tex, cv::Mat &mask) const;

    typedef std::vector<Patches> list;

private:
    const cv::Mat *texture_;
};

CvContours findContours(const cv::Mat &mat)
{
    cv::Mat tmp(mat.rows + 2, mat.cols + 2, mat.type(), cv::Scalar(0));
    cv::Rect rect(1, 1, mat.cols, mat.rows);
    mat.copyTo(cv::Mat(tmp, rect));

    CvContours contours;
    cv::findContours(tmp, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE,
                     cv::Point(-1, -1));

    return contours;
}

void Patches::copy(cv::Mat &tex, cv::Mat &mask) const
{
    for (const auto &rect : rects) {
        // TODO: block copy via Mat::copyTo
        for (int y = 0; y < rect.height(); y++) {
            for (int x = 0; x < rect.width(); x++) {
                int sx(rect.x() + x), sy(rect.y() + y);
                clampMatPos(sx, sy, *texture_);

                int dx(rect.packX + x), dy(rect.packY + y);
                if (!validMatPos(dx, dy, tex)) {
                    continue;
                }

                tex.at<cv::Vec3b>(dy, dx) = texture_->at<cv::Vec3b>(sy, sx);
            }
        }
    }

    (void) mask;
}

imgproc::UVRect makeRect(const CvPoints &points)
{
    imgproc::UVRect rect;
    for (const auto &pt : points) {
        rect.update(imgproc::UVCoord(pt.x, pt.y));
    }
    return rect;
}

std::size_t findRect(const TextureInfo &tx, const Face &face
                     , const UVRects &rects)
{
    const auto &pt(tx.uv(face, 0));
    for (unsigned i = 0; i < rects.size(); ++i) {
        const auto &r(rects[i]);
        const float safety(0.51);

        if (pt(0) > (r.min.x - safety)
            && pt(0) < (r.max.x + safety)
            && pt(1) > (r.min.y - safety)
            && pt(1) < (r.max.y + safety))
        {
            return i;
        }
    }

    LOG(err2) << "Rectangle not found (pt = " << pt << ").";
    return 0;
}

std::atomic<int> gen(0);

Patches findPatches(const TextureInfo &tx, float inflate = 0)
{
    Patches patches(tx.texture());

    cv::Point3f tri[3];
    for (const auto &face : tx.faces()) {
        for (int i : { 0, 1, 2 }) {
            const auto &p(tx.uv(face, i));
            tri[i] = { float(p(0)), float(p(1)), 0.f };
            // make sure the vertices are always marked
            int x(round(tri[i].x)), y(round(tri[i].y));
            if (inside(x, y, patches.mask)) {
                patches.mask.at<std::uint8_t>(y, x) = 0xff;
            }
        }

        // rasterize triangle
        std::vector<imgproc::Scanline> scanlines;
        imgproc::scanConvertTriangle(tri, 0, patches.mask.rows, scanlines);

        for (const auto &sl : scanlines) {
            imgproc::processScanline(sl, 0, patches.mask.cols
                                     , [&](int x, int y, float)
            {
                patches.mask.at<std::uint8_t>(y, x) = 0xff;
            });
        }
    }

    {
        auto fname(str(boost::format("dump/mask-%d.png") % gen++));
        LOG(info4) << "Writing mask into: " << fname;
        imwrite(fname, patches.mask);
    }

    // find contrours of patches
    for (const auto &contour : findContours(patches.mask)) {
        patches.rects.push_back(makeRect(contour));
    }
    LOG(info4) << "rects: " << patches.rects.size();

    // assign triangles and texture coordinates to patches
    {
        patches.tcAssignment.resize(tx.tc().size(), -1);
        for (const auto &face : tx.faces()) {
            auto rindex(findRect(tx, face, patches.rects));
            patches.faceAssignment.push_back(rindex);
            for (int i : { 0, 1, 2 }) {
                patches.tcAssignment[face(i)] = rindex;
            }
        }
    }

    LOG(info4) << "assignment: " << utility::join(patches.tcAssignment, ",");

    // update rectangles

    // make room
    for (auto &r : patches.rects) {
        r.clear();
    }

    // update
    {
        auto ifaceAssignment(patches.faceAssignment.begin());
        for (const auto &face : tx.faces()) {
            auto &rect(patches.rects[*ifaceAssignment++]);
            for (int i : { 0, 1, 2 }) {
                rect.update(tx.uvCoord(face, i));
            }
        }
    }

    if (inflate) {
        for (auto &r : patches.rects) {
            r.inflate(inflate);
        }
    }

    return patches;
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
                     , const RawAtlas::pointer &atlas)
        : originalMesh_(mesh), originalAtlas_(atlas)
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
    // simple range, nothing to do
    // FIXME: textured range of size 1 must processed as well!
    if (range.size() <= 1) {
        pass(range);
        return;
    }

    if (range.textured) {
        mergeTextured(range);
        return;
    }

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
            (atlasEnd_, *originalAtlas_);
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

} // namespace

MeshAtlas mergeSubmeshes(const Mesh::pointer &mesh
                         , const RawAtlas::pointer &atlas)
{
    return MeshAtlasBuilder(mesh, atlas).result();
}

std::tuple<cv::Mat, math::Points2d, Faces>
joinTextures(const TextureInfo::list &texturing, float inflate)
{
    std::tuple<cv::Mat, math::Points2d, Faces> res;
    auto &tex(std::get<0>(res));
    auto &tc(std::get<1>(res));
    auto &faces(std::get<2>(res));

    // rasterize faces to masks
    Patches::list patchesList;
    for (const auto &tx : texturing) {
        patchesList.push_back(findPatches(tx, inflate));
    }

    // pack the patches
    imgproc::RectPacker packer;
    for (auto &patches : patchesList) {
        for (auto &r : patches.rects) {
            packer.addRect(&r);
        }
    }
    packer.pack();

    // create texture
    tex.create(packer.height(), packer.width(), CV_8UC3);
    tex = cv::Scalar(0, 0, 0);

    // validity mask
    cv::Mat mask(tex.rows, tex.cols, CV_8U, cv::Scalar(0));

    {
        auto ts(tex.size());

        auto ipatchesList(patchesList.cbegin());
        for (const auto &tx : texturing) {
            const auto &patches(*ipatchesList++);

            // copy patches from this texture
            patches.copy(tex, mask);

            // transform texturing coordinates from source texture into new
            // texture
            auto itcAssignment(patches.tcAssignment.cbegin());
            for (const auto &oldUv : tx.tc()) {
                auto &rect(patches.rects[*itcAssignment++]);
                imgproc::UVCoord uv(oldUv(0), oldUv(1));
                rect.adjustUV(uv);
                tc.push_back(normalize(uv, ts));
            }

            // add texture faces from this texture
            appendFaces(faces, tx.faces(), faces.size());
        }
    }

    // TODO: inpainting

    // done
    return res;
}

} } // namespace vadstena::vts
