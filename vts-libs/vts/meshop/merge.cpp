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
#include <cstdlib>
#include <memory>
#include <array>
#include <stack>
#include <set>
#include <algorithm>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/variant.hpp>
#include <boost/range.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "utility/streams.hpp"
#include "utility/expect.hpp"
#include "utility/stl-helpers.hpp"

#include "math/math.hpp"
#include "math/transform.hpp"
#include "math/filters.hpp"

#include "imgproc/uvpack.hpp"
#include "imgproc/scanconversion.hpp"
#include "imgproc/const-raster.hpp"
#include "imgproc/reconstruct.hpp"
#include "imgproc/inpaint.hpp"

#include "../opencv/atlas.hpp"
#include "../opencv/texture.hpp"

#include "../meshop.hpp"
#include "../math.hpp"

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

namespace vtslibs { namespace vts {

namespace {

namespace fs = boost::filesystem;

const char *VTS_MESH_MERGE_DUMP_DIR(std::getenv("VTS_MESH_MERGE_DUMP_DIR"));
const char *NO_ATLAS_INPAINT(std::getenv("NO_ATLAS_INPAINT"));


const std::size_t DefaultMaxVertexCount(1 << 15);
const std::size_t DefaultMaxFaceCount(1 << 15);

typedef std::vector<cv::Mat> MatList;

inline math::Point2d normalize(const imgproc::UVCoord &uv
                               , const cv::Size &texSize)
{
    return { uv.x / texSize.width
            , 1.0 - uv.y / texSize.height };
}

class TextureInfo {
public:
    TextureInfo(const SubMesh &sm, const cv::Mat &texture
                , const SubmeshMergeOptions &options)
        : tc_(opencv::denormalize(sm.tc, texture.size()))
        , faces_(sm.facesTc), texture_(texture)
        , mask_(texture.rows, texture.cols, CV_8U, cv::Scalar(0x00))
    {
        switch (options.atlasPacking) {
        case SubmeshMergeOptions::AtlasPacking::legacy:
            opencv::rasterizeMaskLegacy(mask_, faces_, tc_);
            break;

        case SubmeshMergeOptions::AtlasPacking::progressive:
            opencv::rasterizeMask(mask_, faces_, tc_);
            break;
        }
    }

    const math::Points2d& tc() const { return tc_; }
    const Faces& faces() const { return faces_; }
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

    double area(int faceIndex) const {
        const auto &face((faces_)[faceIndex]);
        return triangleArea(tc_[face(0)], tc_[face(1)], tc_[face(2)]);
    }

    /** Face barycenter.
     */
    math::Point2 center(int faceIndex) const {
        const auto &face((faces_)[faceIndex]);
        return ((tc_[face(0)] + tc_[face(1)] + tc_[face(2)]) / 3.0);
    }

    const Face& face(int faceIndex) const {
        return (faces_)[faceIndex];
    }

    Face& ncface(int faceIndex) {
        return (faces_)[faceIndex];
    }

    // Duplicates existing texture coordinates in the systemand returns new
    // index.
    int duplicateTc(int index) {
        auto ni(tc_.size());
        tc_.push_back(tc_[index]);
        return ni;
    }

private:
    math::Points2d tc_;
    Faces faces_;
    cv::Mat texture_;
    cv::Mat mask_;
};

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

typedef Face::value_type VertexIndex;
typedef std::set<VertexIndex> VertexIndices;

/** Continuous mesh component.
 */
struct Component {
    /** Set of components's indices to faces
     */
    std::set<int> faces;

    /** Set of components's indices to texture coordinates
     */
    VertexIndices indices;

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

    imgproc::UVCoord adjustUV(const math::Point2 &p) const {
        auto np(math::transform(rrectTrafo, p));
        return imgproc::UVCoord(np(0) + rect.packX, np(1) + rect.packY);
    }

    void bestRectangle(const TextureInfo &tx);

    double area(const TextureInfo &tx) const;
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
    TextureInfo *tx;

    typedef std::vector<ComponentInfo> list;

    ComponentInfo(const TileId &tileId, int id, TextureInfo &tx);

    void copy(cv::Mat &tex) const;

    void debug(const fs::path &dir, const cv::Mat &outAtlas) const;

private:
    /** Breaks component into best fitting parts.
     */
    void breakComponent(const Component::pointer &c);

    /** Adds new components.
     */
    void addComponent(const Component::pointer &c);

    TileId tileId_;
    int id_;
};

ComponentInfo::ComponentInfo(const TileId &tileId, int id, TextureInfo &tx)
    : tcMap(tx.tc().size()), tx(&tx), tileId_(tileId), id_(id)
{
    const auto &faces(tx.faces());
    Component::list fMap(tx.faces().size());
    Component::list tMap(tx.tc().size());

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

    // temporary toplevel components, broken into smalled ones afterwars
    Component::set tlComponents;

    for (std::size_t i(0), e(faces.size()); i != e; ++i) {
        const auto &face(faces[i]);

        FaceComponents fc =
            { { &tMap[face(0)], &tMap[face(1)], &tMap[face(2)] } };
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
            tlComponents.erase(owned);

            // grab owned by value to prevent overwrite
            const auto old(*owned);

            // merge components
            owner->add(old);

            // move everything to owner
            for (auto index : old.faces) { fMap[index] = owner; }
            for (auto index : old.indices) { tMap[index] = owner; }
        });

        if (*fc[0] || *fc[1] || *fc[2]) {
            auto &owner(*fc[0]);
            fMap[i] = owner;
            owner->add(i, face);
            assign(owner, *fc[1]);
            assign(owner, *fc[2]);
        } else {
            // create new component
            tlComponents.insert
                (fMap[i] = *fc[0] = *fc[1] = *fc[2]
                 = std::make_shared<Component>(i, face));
        }
    }

    // cut components to best fitting subcomponets
    for (const auto &c : tlComponents) {
        breakComponent(c);
    }
}

class BrokenComponent {
public:
    Component::pointer component;
    double margin;

    typedef std::stack<BrokenComponent> stack;

    BrokenComponent(const TextureInfo &tx
                    , const Component::pointer &component
                    , double margin = -1.0)
        : component(component), margin(margin)
        , area_(component->area(tx))
    {
        component->bestRectangle(tx);
    }

    BrokenComponent()
        : component(std::make_shared<Component>())
        , margin(-1.0), area_(0.0)
    {}

    bool cuttable() const {
        return (component->faces.size() > 2);
    }

    const cv::RotatedRect& rrect() const { return component->rrect; }

    void cut(const TextureInfo &tx
             , bool vertical, double distance
             , BrokenComponent &best1, BrokenComponent &best2)
        const;

    bool empty() const { return component->faces.empty(); }

    double marginArea() const {
        const auto &cr(rrect());
        return ((cr.size.width + 2.0 * margin)
                * (cr.size.height + 2.0 * margin));
    }

    double efficiency() const {
        if (empty()) { return 0; }
        return (area_ / marginArea());
    }

    static double efficiency(const BrokenComponent &bc1
                             , const BrokenComponent &bc2)
    {
        if (bc1.empty() || bc2.empty()) { return 0.0; }

        return ((bc1.area_ + bc2.area_) /
                (bc1.marginArea() + bc2.marginArea()));
    }

private:
    double area_;
};

void BrokenComponent::cut(const TextureInfo &tx
                          , bool vertical, double distance
                          , BrokenComponent &best1, BrokenComponent &best2)
    const
{
    const auto &cr(rrect());

    // angle in radians
    const auto a((cr.angle * M_PI / 180.) + (vertical ? 0 : M_PI_2));
    const auto sa(std::sin(a));
    const auto ca(std::cos(a));
    const auto &rc(cr.center);
    const auto &rs(cr.size);
    const double xline(distance - ((vertical ? rs.width : rs.height) / 2.0));

    auto c1(std::make_shared<Component>());
    auto c2(std::make_shared<Component>());

    for (auto findex : component->faces) {
        auto barycenter(tx.center(findex));
        // rotate barycenter and check at which side it resides
        if ((ca * (barycenter(0) - rc.x) - sa * (barycenter(1) - rc.y))
            < xline)
        {
            c1->add(findex, tx.face(findex));
        } else {
            c2->add(findex, tx.face(findex));
        }
    }

    // construct components from cuts
    BrokenComponent cut1(tx, c1, margin);
    BrokenComponent cut2(tx, c2, margin);

    if (efficiency(cut1, cut2) > efficiency(best1, best2)) {
        // better cuts, reassign
        best1 = cut1;
        best2 = cut2;
    }
}

void ComponentInfo::breakComponent(const Component::pointer &start)
{
    const double margin(0.5);
    const double cutDistance(7.0);

    BrokenComponent::stack cuttable;
    cuttable.emplace(*tx, start, margin);

    while (!cuttable.empty()) {
        // get top of the stack
        const auto &top(cuttable.top());
        const auto &cr(top.rrect());

        auto add([&]()
        {
            LOG(info1) << "---- Patch uncuttable";
            addComponent(top.component);
            cuttable.pop();
        });

        if (!top.cuttable()) {
            add();
            continue;
        }

        unsigned int vCuts(std::floor(cr.size.width / cutDistance));
        unsigned int hCuts(std::floor(cr.size.height / cutDistance));
        double vCutDist(cr.size.width / vCuts);
        double hCutDist(cr.size.height / hCuts);

        if (!vCuts && !hCuts) {
            // patch is uncuttable -> move to output
            add();
            continue;
        }

        BrokenComponent best1, best2;

        // vertical cuts
        for (uint i = 1; i < vCuts; ++i) {
            top.cut(*tx, true, i * vCutDist, best1, best2);
        }

        // horizontal cuts
        for (uint i = 1; i < hCuts; ++i) {
            top.cut(*tx, false, i * hCutDist, best1, best2);
        }

        auto ne(BrokenComponent::efficiency(best1, best2));
        if (ne > top.efficiency()) {
            // split patches have better efficiency than original patch ->
            // replace it with them and go on
            LOG(info1) << "---- Patches with better efficiency found: " << ne;
            cuttable.pop();
            cuttable.push(best1);
            cuttable.push(best2);
        } else {
            // patch is uncuttable -> move to output
            add();
            continue;
        }
    }
}

void ComponentInfo::addComponent(const Component::pointer &c)
{
    components.insert(c);

    typedef std::map<int, int> Remap;
    Remap tcRemap;

    // process component's faces
    for (auto findex : c->faces) {
        auto &face(tx->ncface(findex));

        // process all face's vertices
        for (auto &index : face) {
            auto &currentComponent(tcMap[index]);

            if (currentComponent && (currentComponent != c)) {
                // vertex shared with another component

                // check whether we have already remapped it
                int ni;
                auto ftcRemap(tcRemap.find(index));
                if (ftcRemap == tcRemap.end()) {
                    // duplicate and remember
                    ni = tx->duplicateTc(index);
                    tcRemap.insert(Remap::value_type(index, ni));
                    tcMap.push_back(c);
                } else {
                    // alredy remapped -> reuse
                    ni = ftcRemap->second;
                }

                // update (NB: index is a reference!)
                index = ni;
            }

            // map component to texture coordinate
            tcMap[index] = c;
        }
    }

    auto remap([&](VertexIndices &set, const Remap &r)
    {
        for (const auto &item : r) {
            set.erase(item.first);
            set.insert(item.second);
        }
    });

    // remap faces and tc (if any were changed
    remap(c->indices, tcRemap);
}

void ComponentInfo::copy(cv::Mat &tex) const
{
    for (const auto &c : components) {
        c->copy(tex, tx->texture(), tx->mask());
    }
}


void ComponentInfo::debug(const fs::path &dir, const cv::Mat &outAtlas)
    const
{
    int border(16);

    const auto &texture(tx->texture());

    cv::Mat dr(texture.rows + outAtlas.rows + 3 * border
               , std::max(texture.cols, outAtlas.cols) + 2 * border, CV_8UC3
               , cv::Scalar(0x80, 0x80, 0x80));

    cv::Mat drTx(dr, cv::Rect(border, border, texture.cols, texture.rows));

    // combine input texture with its mask and put into drawable
    {
        cv::Mat colorMask;
        const auto &mask(tx->mask());
        cv::cvtColor(mask, colorMask, cv::COLOR_GRAY2BGR);
        addWeighted(texture, 0.7, colorMask, 0.3, 0.0, drTx);
    }

    // put output atlas into drawable
    cv::Mat drAt(dr, cv::Rect
                 (border, texture.rows + 2 * border
                  , outAtlas.cols, outAtlas.rows));
    outAtlas.convertTo(drAt, -1, 0.7);

    // draw both input and output rectangles
    auto &rng(cv::theRNG());

    for (const auto &c : components) {
        cv::Scalar color(rng(223) + 32, rng(223) + 32, rng(223) + 32);

        cv::Point2f pts[4];
        c->rrect.points(pts);

        std::vector<std::vector<cv::Point2i>> contours;
        {
            contours.emplace_back();
            auto &contour(contours.back());
            for (int i : { 0, 1, 2, 3 }) {
                contour.emplace_back(int(pts[i].x), int(pts[i].y));
            }
        }

        cv::drawContours(drTx, contours, 0, color, 1); //CV_FILLED);

        const auto &pr(c->rect);
        cv::Rect r(pr.packX, pr.packY, pr.width(), pr.height());
        cv::rectangle(drAt, r, color, 1); //CV_FILLED);
    }

    fs::create_directories(dir);
    cv::imwrite((dir / str(boost::format("%s-%d-repacked.png")
                           % tileId_ % id_)).string()
                 , dr);
}

imgproc::UVRect uvRect(float width, float height)
{
    imgproc::UVRect r;
    r.update({0.f, 0.f});
    r.update({width, height});
    return r;
}

void Component::bestRectangle(const TextureInfo &tx) {
    // find best rectangle
    if (!indices.empty()) {
        std::vector<cv::Point2f> points;
        points.reserve(indices.size());

        // process all points
        for (const auto i : indices) {
            const auto &p(tx.uv(i));
            points.emplace_back(p(0), p(1));
        }

        // find best rectangle fitting around this patch
        rrect = cv::minAreaRect(points);

        // calculate UV rectangle for this rotated rect
        rect = uvRect(rrect.size.width, rrect.size.height);
    }

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

        rrectTrafo(0, 3) = -shift(0) + double(rect.width()) / 2.0;
        rrectTrafo(1, 3) = -shift(1) + double(rect.height()) / 2.0;
    }
}

double Component::area(const TextureInfo &tx) const
{
    double area(0.0);
    for (const auto &face : faces) {
        area += tx.area(face);
    }
    return area;
}

/** Const raster for texture with cv::Mat-based mask.
 */
class TextureRaster : public imgproc::CvConstRaster<cv::Vec3b>
{
public:
    TextureRaster(const cv::Mat &texture, const cv::Mat &mask)
        : imgproc::CvConstRaster<cv::Vec3b>(texture), mask_(mask)
    {}

    bool valid(int x, int y) const {
        // queries matrix for bounds and mask for pixel validity
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

    const math::Size2 size(rect.width(), rect.height());

    // copy texture from source best rectangle to output uv rectangle
    for (int y = 0; y < size.height; ++y) {
        // destination point + bound check
        const int yy(y + rect.packY);
        for (int x = 0; x < size.width; ++x) {
            // destination point + bound check
            const int xx(x + rect.packX);

            // compute source point
            // NB: we have mapped corners of rectanges, not pixel centers
            auto pos(math::transform
                     (trafo, math::Point2(x + 0.5, y + 0.5)));
            pos(0) -= 0.5; pos(1) -= 0.5;

            // reconstruct point using filter
            auto res(imgproc::reconstruct(raster, filter, pos));

            // store result
            tex.at<cv::Vec3b>(yy, xx) = cv::Vec3b(res[0], res[1], res[2]);
        }
    }
}

/** Joins textures into single texture.
 */
std::tuple<cv::Mat, math::Points2d, Faces>
joinTextures(const TileId &tileId, TextureInfo::list texturing);

typedef std::tuple<Mesh::pointer, Atlas::pointer> MeshAtlas;

typedef std::vector<int> Indices;

/** TODO: observe other differences in submeshes (texture mode, external
 *  texture coordinates, texture layer, UV area factor etc.
 *  Not so simple :)
 */
struct Range {
    const Indices *indices;
    std::size_t indexStart;
    std::size_t indexEnd;
    SubMesh::SurfaceReference surface;
    double uvAreaScale;
    SubMesh::ZIndex zIndex;
    bool textured;

    std::size_t size() const { return indexEnd - indexStart; }
    bool empty() const { return !size(); }
    Indices::const_iterator begin() const {
        return indices->begin() + indexStart;
    }
    Indices::const_iterator end() const {
        return indices->begin() + indexEnd;
    }
    int front() const { return (*indices)[indexStart]; }

    typedef std::vector<Range> list;

    Range(const Indices &indices, const SubMesh &sm, bool textured)
        : indices(&indices), surface(sm.surfaceReference)
        , uvAreaScale(sm.uvAreaScale), zIndex(sm.zIndex)
        , textured(textured)
    {}

    bool compatible(const SubMesh &sm) const {
        return ((surface == sm.surfaceReference)
                && (uvAreaScale == sm.uvAreaScale)
                && (zIndex == sm.zIndex));
    }
};

template<typename CharT, typename Traits>
inline std::basic_ostream<CharT, Traits>&
operator<<(std::basic_ostream<CharT, Traits> &os, const Range &r)
{
    os << "Range{sr=" << int(r.surface)
       << ", z=" << r.zIndex
       << ", uvas=" << r.uvAreaScale
       << ", tx=" << r.textured
       << ", start=" << r.indexStart << ", end=" << r.indexEnd
       << ", sm=" << utility::join
        (boost::make_iterator_range(r.begin(), r.end()), ",")
       << "}";

    return os;
}

struct EntityCounter {
    std::size_t count;
    std::size_t vertexCount;
    std::size_t tcCount;
    std::size_t faceCount;

    EntityCounter(std::size_t vertexCount = 0, std::size_t faceCount = 0)
        : count(0), vertexCount(vertexCount), tcCount(vertexCount)
        , faceCount(faceCount)
    {}

    EntityCounter(const SubMesh &sm)
        : count(1), vertexCount(sm.vertices.size()), tcCount(sm.tc.size())
        , faceCount(sm.faces.size())
    {}

    void update(const SubMesh &sm) {
        vertexCount += sm.vertices.size();
        tcCount += sm.tc.size();
        faceCount += sm.faces.size();
        ++count;
    }

    bool check(const EntityCounter &limits, bool atLeastOne = false) const {
        if (atLeastOne && (count == 1)) { return true; }
        return ((vertexCount <= limits.vertexCount)
                && (tcCount <= limits.tcCount)
                && (faceCount <= limits.faceCount));
    }

    bool available(const SubMesh &sm) const {
        if ((sm.vertices.size() + 3) > vertexCount) { return false; }
        if ((sm.tc.size() + 3) > tcCount) { return false; }
        if ((sm.faces.size() + 1) > faceCount) { return false; }
        return true;
    }

    void reset() { vertexCount = faceCount = tcCount = count = 0; }
};

/** Precondition: submeshes from the same source are grouped.
 */
Range::list groupSubmeshes(const SubMesh::list &sms, Indices &indices
                           , std::size_t textured
                           , const EntityCounter &meshLimits)
{
    // compare submeshes to group together submeshes with the same
    // surfaceReference, zIndex and uvAreaScale (i.e. compatible ones)
    const auto compareSubmeshes([&](int li, int ri) -> bool
    {
        const auto &l(sms[li]);
        const auto &r(sms[ri]);

        // order: surface reference, z-index, UV area scale
        return (std::tie(l.surfaceReference, l.zIndex, l.uvAreaScale)
                < std::tie(r.surfaceReference, r.zIndex, r.uvAreaScale));
    });

    Range::list out;

    const auto splitToRanges([&](std::size_t begin, std::size_t end
                                 , bool textured)
    {
        bool first(true);
        EntityCounter ec;
        for (auto i(begin); i != end; ++i) {
            const auto &sm(sms[indices[i]]);
            ec.update(sm);
            if (first || !out.back().compatible(sm)
                || !ec.check(meshLimits, true))
            {
                out.emplace_back(indices, sm, textured);
                out.back().indexStart = i;
                ec.reset();
                first = false;
            }
            out.back().indexEnd = i + 1;
        }
    });

    // prepare indices
    indices.reserve(sms.size());
    for (std::size_t i(0), e(sms.size()); i != e; ++i) {
        indices.push_back(i);
    }

    // sanitize input
    if (textured > sms.size()) { textured = sms.size(); }

    // sort both parts
    if (textured) {
        std::sort(indices.begin(), indices.begin() + textured
                  , compareSubmeshes);
        splitToRanges(0, textured, true);
    }

    // non-textured
    if (sms.size() > textured) {
        std::sort(indices.begin() + textured, indices.end(), compareSubmeshes);
        splitToRanges(textured, sms.size(), false);
    }

    return out;
}

typedef opencv::HybridAtlas HybridAtlas;

typedef boost::variant<RawAtlas::pointer
                       , HybridAtlas::pointer> InputAtlas;

struct GetAtlas : public boost::static_visitor<Atlas::pointer>
{
    Atlas::pointer operator()(const RawAtlas::pointer &atlas) const {
        return atlas;
    }

    Atlas::pointer operator()(const HybridAtlas::pointer &atlas) const {
        return atlas;
    }
};

class AsHybridAtlas
    : public boost::static_visitor<HybridAtlas::pointer>
{
public:
    AsHybridAtlas(std::size_t count, int quality)
        : count_(count), quality_(quality)
    {}

    HybridAtlas::pointer operator()(const RawAtlas::pointer &atlas) const {
        return std::make_shared<HybridAtlas>(count_, *atlas, quality_);
    }

    HybridAtlas::pointer operator()(const HybridAtlas::pointer &atlas) const {
        return std::make_shared<HybridAtlas>(count_, *atlas, quality_);
    }

private:
    std::size_t count_;
    int quality_;
};

class GetImage : public boost::static_visitor<cv::Mat>
{
public:
    GetImage(std::size_t index) : index_(index) {}

    cv::Mat operator()(const RawAtlas::pointer &atlas) const {
        return HybridAtlas::imageFromRaw(atlas->get(index_));
    }

    cv::Mat operator()(const HybridAtlas::pointer &atlas) const {
        return atlas->get(index_);
    }

private:
    std::size_t index_;
};

class AddItem : public boost::static_visitor<>
{
public:
    AddItem(const HybridAtlas::pointer &atlas, std::size_t index)
        : atlas_(atlas), index_(index)
    {}

    void operator()(const RawAtlas::pointer &atlas) const {
        atlas_->add(atlas->get(index_));
    }

    void operator()(const HybridAtlas::pointer &atlas) const {
        atlas_->add(atlas->entry(index_));
    }

private:
    HybridAtlas::pointer atlas_;
    std::size_t index_;
};

inline std::size_t applyDefault(std::size_t value, std::size_t dflt)
{
    return value ? value : dflt;
}

class MeshAtlasBuilder {
public:
    MeshAtlasBuilder(const TileId &tileId
                     , const Mesh::pointer &mesh
                     , const InputAtlas &atlas
                     , int textureQuality
                     , const SubmeshMergeOptions &options)
        : tileId_(tileId), textureQuality_(textureQuality), options_(options)
        , originalMesh_(mesh), originalAtlas_(atlas)
        , oSubmeshes_(mesh->submeshes)
        , meshLimits_(applyDefault(options.maxVertexCount
                                   , DefaultMaxVertexCount)
                      , (applyDefault(options.maxFaceCount
                                      , DefaultMaxFaceCount)))
        , meshEnd_(), atlasEnd_()
    {
        merge(groupSubmeshes
              (mesh->submeshes, smIndices_
               , boost::apply_visitor(GetAtlas(), atlas)->size()
               , meshLimits_));
    }

    MeshAtlas result() const {
        return MeshAtlas
            (changedMesh() ? mesh_ : originalMesh_
             , changedAtlas()
             ? atlas_
             : boost::apply_visitor(GetAtlas(), originalAtlas_));
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

    void breakSubmesh();

    void makeSharedFaces();

    TextureInfo::list texturing(const Range &range) const;

    const TileId tileId_;
    const int textureQuality_;
    const SubmeshMergeOptions &options_;
    const Mesh::pointer originalMesh_;
    const InputAtlas originalAtlas_;
    const SubMesh::list oSubmeshes_;
    const EntityCounter meshLimits_;
    Mesh::pointer mesh_;
    HybridAtlas::pointer atlas_;
    std::size_t meshEnd_;
    std::size_t atlasEnd_;
    Indices smIndices_;
};

TextureInfo::list MeshAtlasBuilder::texturing(const Range &range) const
{
    TextureInfo::list tx;
    for (auto i : range) {
        tx.emplace_back
            (originalMesh_->submeshes[i]
             , boost::apply_visitor(GetImage(i), originalAtlas_), options_);
    }
    return tx;
}

void MeshAtlasBuilder::merge(const Range::list &ranges)
{
    // LOG(info4) << "Ranges: " << utility::join(ranges, ", ");
    for (const auto &range : ranges) { merge(range); }
}

void MeshAtlasBuilder::merge(const Range &range)
{
    // sanity check
    if (range.empty()) { return; }

    // textured range - always merge (even if it is only one mesh because we
    // want to repack atlas)
    if (range.textured) { return mergeTextured(range); }

    // non textured submeshes

    // single submes -> pass as is
    if (range.size() == 1) { return pass(range); }

    // merge non-textured submeshes
    mergeNonTextured(range);
}

void MeshAtlasBuilder::ensureChanged(const Range &range)
{
    if (!changedMesh()) {
        // copy mesh and trim submeshes
        mesh_ = std::make_shared<Mesh>(*originalMesh_, meshEnd_);
    }

    if (range.textured && !changedAtlas()) {
        // convert atlas
        atlas_ = boost::apply_visitor
            (AsHybridAtlas(atlasEnd_, textureQuality_), originalAtlas_);
    }
}

void MeshAtlasBuilder::pass(const Range &range)
{
    if (changedMesh()) {
        // copy submeshes
        for (auto i : range) {
            mesh_->submeshes.push_back(originalMesh_->submeshes[i]);
        }
    }

    if (range.textured && changedAtlas()) {
        // copy atlas
        for (auto i : range) {
            boost::apply_visitor(AddItem(atlas_, i), originalAtlas_);
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
    mesh_->submeshes.push_back(oSubmeshes_[range.front()]);
    auto &sm(mesh_->submeshes.back());

    // drop texturing info (if any)
    sm.tc.clear();
    sm.facesTc.clear();

    for (auto irange(range.begin() + 1), erange(range.end());
         irange != erange; ++irange)
    {
        auto &src(oSubmeshes_[*irange]);

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
    std::tie(texture, sm.tc, sm.facesTc)
        = joinTextures(tileId_, texturing(range));
    atlas_->add(texture);
    // housekeeping
    atlasEnd_ += range.size();

    if (options_.sharedFaces) {
        makeSharedFaces();
    }

    if (!EntityCounter(sm).check(meshLimits_)) {
        // breaks last submesh into compliant submeshes
        breakSubmesh();
    }
}

class MeshBuilder {
public:
    MeshBuilder(const EntityCounter &meshLimits, const SubMesh &src)
        : meshLimits_(meshLimits), src_(src)
        , vertexMap_(src_.vertices.size(), -1)
        , tcMap_(src_.tc.size(), -1)
        , ifaces_(src_.faces.begin())
        , efaces_(src_.faces.end())
        , ifacesTc_(src_.facesTc.begin())
    {}

    std::size_t run(Mesh &mesh);

private:
    bool run(SubMesh &dst);

    void reset() {
        vertexMap_.assign(src_.vertices.size(), -1);
        tcMap_.assign(src_.tc.size(), -1);
    }

    int addVertex(SubMesh &dst, int v) {
        auto& mv(vertexMap_[v]);
        if (mv < 0) {
            mv = dst.vertices.size();
            dst.vertices.push_back(src_.vertices[v]);

            // copy external texture coordinate if present
            if (!src_.etc.empty()) { dst.etc.push_back(src_.etc[v]); }
        }


        return mv;
    }

    int addTc(SubMesh &dst, int v) {
        auto& mv(tcMap_[v]);
        if (mv < 0) {
            mv = dst.tc.size();
            dst.tc.push_back(src_.tc[v]);
        }
        return mv;
    }

    void addFace(SubMesh &dst, const Face &f, const Face &ftc) {
        dst.faces.emplace_back(addVertex(dst, f(0))
                               , addVertex(dst, f(1))
                               , addVertex(dst, f(2)));
        dst.facesTc.emplace_back(addTc(dst, ftc(0))
                                 , addTc(dst, ftc(1))
                                 , addTc(dst, ftc(2)));
    }

    /** TODO: implement capacity check here
     */
    bool check(const SubMesh &dst) const {
        return meshLimits_.available(dst);
    }

    const EntityCounter &meshLimits_;
    const SubMesh &src_;
    std::vector<int> vertexMap_;
    std::vector<int> tcMap_;

    Faces::const_iterator ifaces_;
    Faces::const_iterator efaces_;
    Faces::const_iterator ifacesTc_;
};

std::size_t MeshBuilder::run(Mesh &mesh)
{
    LOG(info1)
        << "Breaking submesh apart ["
        << src_.vertices.size() << "/" << src_.tc.size()
        << "/" << src_.faces.size() << "].";

    const auto start(mesh.submeshes.size());
    while (run(utility::append(mesh.submeshes))) { reset(); }
    return (mesh.submeshes.size() - start);
}

bool MeshBuilder::run(SubMesh &dst)
{
    for (; (ifaces_ != efaces_) && check(dst); ++ifaces_, ++ifacesTc_) {
        addFace(dst, *ifaces_, *ifacesTc_);
    }

    return (ifaces_ != efaces_);
}

void MeshAtlasBuilder::breakSubmesh()
{
    // steal source mesh from the back of the submeshes
    auto src(std::move(mesh_->submeshes.back()));
    mesh_->submeshes.resize(mesh_->submeshes.size() - 1);

    // duplicate texture
    for (auto smCount(MeshBuilder(meshLimits_, src).run(*mesh_));
         smCount > 1; --smCount)
    {
        atlas_->duplicate();
    }
}

void MeshAtlasBuilder::makeSharedFaces()
{
    // get last submesh
    auto &sm(mesh_->submeshes.back());

    math::Points3d vertices;
    math::Points2d tc;
    Faces faces;

    typedef std::pair<int, int> VertexTcPair;
    typedef std::map<VertexTcPair, int> VertexMap;

    VertexMap vmap;

    auto ifaces(sm.faces.begin());
    for (const auto &tface : sm.facesTc) {
        const auto &face(*ifaces++);

        faces.emplace_back();
        auto &oface(faces.back());

        for (int i(0); i < 3; ++i) {
            const VertexTcPair pair(face(i), tface(i));

            auto fvmap(vmap.find(pair));
            if (fvmap == vmap.end()) {
                // unknown vertex/tc pair
                const auto idx(vertices.size());
                vertices.emplace_back(sm.vertices[pair.first]);
                tc.emplace_back(sm.tc[pair.second]);

                vmap.insert(VertexMap::value_type(pair, idx));
                oface(i) = idx;
            } else {
                oface(i) = fvmap->second;
            }
        }
    }

    // fill in new data
    sm.vertices.swap(vertices);
    sm.tc.swap(tc);
    // and use the same data for both 3D and 2D face
    sm.faces.swap(faces);
    sm.facesTc = sm.faces;
}

std::tuple<cv::Mat, math::Points2d, Faces>
joinTextures(const TileId &tileId, TextureInfo::list texturing)
{
    std::tuple<cv::Mat, math::Points2d, Faces> res;
    auto &tex(std::get<0>(res));
    auto &tc(std::get<1>(res));
    auto &faces(std::get<2>(res));

    // break texturing mesh into components
    ComponentInfo::list cinfoList;
    {
        int id(0);
        for (auto &tx : texturing) {
            cinfoList.emplace_back(tileId, id++, tx);
        }
    }

    // pack patches into new atlas
    auto packedSize([&]() -> math::Size2
    {
        // pack the patches
        imgproc::RectPacker packer;
        for (const auto &cinfo : cinfoList) {
            for (auto &c : cinfo.components) {
                packer.addRect(&c->rect);
            }
        }
        packer.pack();
        return math::Size2(packer.width(), packer.height());
    }());

    // background color
    const auto background(0
                          ? cv::Scalar(0x80, 0x80, 0x80)
                          : cv::Scalar(0x0, 0x0, 0x0));

    tex.create(packedSize.height, packedSize.width, CV_8UC3);
    tex.setTo(background);

    // denormalized result texture coordinates
    math::Points2d dnTc;

    // process patches
    {
        auto ts(tex.size());

        for (const auto &cinfo : cinfoList) {
            const auto &tx(*cinfo.tx);

            // copy patches from this texture
            cinfo.copy(tex);

            auto itcMap(cinfo.tcMap.cbegin());
            auto tcOffset(tc.size());

            int idx(0);
            for (const auto &oldUv : tx.tc()) {
                auto uv((*itcMap++)->adjustUV(oldUv));
                dnTc.emplace_back(double(uv.x), double(uv.y));
                tc.push_back(normalize(uv, ts));
                ++idx;
            }

            // add texture faces from this texture
            appendFaces(faces, tx.faces(), tcOffset);
        }

        if (VTS_MESH_MERGE_DUMP_DIR) {
            for (const auto &cinfo : cinfoList) {
                cinfo.debug(VTS_MESH_MERGE_DUMP_DIR, tex);
            }
        }
    }

    if (!NO_ATLAS_INPAINT) {
        // rasterize valid triangles
        cv::Mat mask(tex.rows, tex.cols, CV_8U, cv::Scalar(0x00));
        opencv::rasterizeMask(mask, faces, dnTc);

        // inpaint JPEG blocks
        imgproc::jpegBlockInpaint(tex, mask);
    }

    // done
    return res;
}

} // namespace

MeshAtlas mergeSubmeshes(const TileId &tileId, const Mesh::pointer &mesh
                         , const RawAtlas::pointer &atlas
                         , int textureQuality
                         , const SubmeshMergeOptions &options)
{
    return MeshAtlasBuilder(tileId, mesh, atlas, textureQuality, options)
        .result();
}

MeshAtlas mergeSubmeshes(const TileId &tileId, const Mesh::pointer &mesh
                         , const HybridAtlas::pointer &atlas
                         , int textureQuality
                         , const SubmeshMergeOptions &options)
{
    return MeshAtlasBuilder(tileId, mesh, atlas, textureQuality, options)
        .result();
}

} } // namespace vtslibs::vts
