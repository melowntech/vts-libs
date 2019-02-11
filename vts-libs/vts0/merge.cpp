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
#include <algorithm>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "math/math.hpp"
#include "math/geometry.hpp"
#include "math/transform.hpp"
#include "imgproc/scanconversion.hpp"
#include "imgproc/binterpolate.hpp"

#include "geometry/faceclip.hpp"
#include "geometry/pointindex.hpp"
#include "imgproc/uvpack.hpp"

#include "geometry/mesh.hpp"
#include "geometry/meshop.hpp"

#include "tileset.hpp"
#include "merge.hpp"
#include "io.hpp"

#include "geometry/binmesh.hpp"

namespace ublas = boost::numeric::ublas;
namespace va = vtslibs;
namespace fs = boost::filesystem;

#ifndef BUILDSYS_CUSTOMER_BUILD
//#define DEBUG 1 // save debug images
#endif

namespace cv {

// comparison operators on cv::Point2 and cv::Point3 for va::PointIndex
template<typename T>
inline bool operator< (const Point_<T> &lhs, const Point_<T> &rhs)
{
    if (lhs.x != rhs.x) return lhs.x < rhs.x;
    return lhs.y < rhs.y;
}

template<typename T>
inline bool operator< (const Point3_<T> &lhs, const Point3_<T> &rhs)
{
    if (lhs.x != rhs.x) return lhs.x < rhs.x;
    if (lhs.y != rhs.y) return lhs.y < rhs.y;
    return lhs.z < rhs.z;
}

} // namespace cv

namespace vtslibs { namespace vts0 {

namespace {

const auto QBufferCvType = CV_16S;
typedef signed short QBufferCType;

double triangleArea(const math::Point3 &a, const math::Point3 &b,
                    const math::Point3 &c)
{
    return norm_2(math::crossProduct(b - a, c - a)) * 0.5;
}

//! Returns a trasformation from tile local coordinates to qbuffer coordinates.
//!
math::Matrix4 tileToBuffer(const math::Size2f &tileSize, int bufferSize)
{
    math::Matrix4 trafo(ublas::identity_matrix<double>(4));
    trafo(0,0) = double(bufferSize) / tileSize.width;
    trafo(1,1) = double(bufferSize) / tileSize.height;
    trafo(0,3) = trafo(1,3) = double(bufferSize) / 2;
    return trafo;
}

//! "Draws" all faces of a tile into the qbuffer, i.e., stores the tile's index
//! everywhere its mesh lies.
//!
void rasterizeTile(const Tile &tile, const math::Matrix4 &trafo,
                   int index, cv::Mat &qbuffer)
{
    std::vector<imgproc::Scanline> scanlines;

    // draw all faces into the qbuffer
    for (const auto face : tile.mesh.facets)
    {
        cv::Point3f tri[3];
        for (int i = 0; i < 3; i++) {
            auto pt(transform(trafo, tile.mesh.vertices[face.v[i]]));
            tri[i] = {float(pt(0)), float(pt(1)), float(pt(2))};
        }

        scanlines.clear();
        imgproc::scanConvertTriangle(tri, 0, qbuffer.rows, scanlines);

        for (const auto& sl : scanlines) {
            imgproc::processScanline(sl, 0, qbuffer.cols,
                [&](int x, int y, float)
                    {
                        qbuffer.at<QBufferCType>(y, x) = index;
                    } );
        }
    }
}

//! Calculates tile Inverse Geometry Coarseness, defined as the ratio of mesh area and area
//! of the covered tile part
//!
double tileInvGeometryCoarseness(const Tile &tile
                                 , const math::Size2f &tileSize)
{
    // calculate the total area of the faces in both the XYZ and UV spaces
    double xyzArea(0);
    const Mesh &mesh(tile.mesh);

    //clip mesh to the tile extents
    geometry::opencv::ClipTriangle::list triangles;
    for(auto &face : mesh.facets){
        triangles.emplace_back( 0, 0,
                                mesh.vertices[face.v[0]],
                                mesh.vertices[face.v[1]],
                                mesh.vertices[face.v[2]],
                                math::Point2(0,0),
                                math::Point2(0,0),
                                math::Point2(0,0));
    }

    //cut small border of the tile to get rid of the skirts
    double eps = 0.0001;
    math::Size2f halfsize(0.5*tileSize.width - tileSize.width*eps
                          , 0.5*tileSize.height - tileSize.height*eps);

    geometry::opencv::ClipPlane planes[4];
    planes[0] = {+1.,  0., 0., halfsize.width};
    planes[1] = {-1.,  0., 0., halfsize.width};
    planes[2] = { 0., +1., 0., halfsize.height};
    planes[3] = { 0., -1., 0., halfsize.height};

    for (int i = 0; i < 4; i++) {
        triangles = geometry::opencv::clipTriangles(triangles, planes[i]);
    }

    //compute the mesh area from clipped triangles
    for (const auto &face : triangles)
    {
        xyzArea += triangleArea(
            math::Point3(face.pos[0].x, face.pos[0].y, face.pos[0].z),
            math::Point3(face.pos[1].x, face.pos[1].y, face.pos[1].z),
            math::Point3(face.pos[2].x, face.pos[2].y, face.pos[2].z));
    }

    const int cBSize(512);
    cv::Mat cbuffer(cBSize, cBSize, CV_32S, cv::Scalar(0));

    math::Matrix4 trafo(tileToBuffer(tileSize, cBSize));
    rasterizeTile(tile, trafo,1, cbuffer);
    double coveredArea = static_cast<double>(cv::countNonZero(cbuffer))
                            /(cBSize*cBSize);

    return coveredArea ? xyzArea/coveredArea : 0;
}

//! Returns true if x and y are valid column and row indices, respectively.
//!
bool validMatPos(int x, int y, const cv::Mat &mat)
{
    return x >= 0 && x < mat.cols && y >= 0 && y < mat.rows;
}

//! Adjusts x and y so that they are valid column and row indices, respectively.
//!
void clampMatPos(int &x, int &y, const cv::Mat &mat)
{
    if (x < 0) x = 0;
    else if (x >= mat.cols) x = mat.cols-1;

    if (y < 0) y = 0;
    else if (y >= mat.rows) y = mat.rows-1;
}

template <int Steps, int Radius>
void close(cv::Mat &mat)
{
    auto element
        (cv::getStructuringElement
         (cv::MORPH_RECT, { 1 + 2 * Radius, 1 + 2 * Radius }));
    cv::Mat tmp(mat.size(), mat.type());
    cv::morphologyEx(mat, tmp, cv::MORPH_CLOSE, element
                     , { -1, -1 }, Steps, cv::BORDER_REPLICATE);
    swap(mat, tmp);
}

//! Returns true if at least one element in the area of the qbuffer
//! corresponding to the given face belongs to tile number 'index'.
//!
bool faceCovered(const Mesh &mesh, const Mesh::Facet &face,
                 const math::Matrix4 &trafo, int index,
                 const cv::Mat/*<QBufferCType>*/ &qbuffer)
{
    cv::Point3f tri[3];
    for (int i = 0; i < 3; i++) {
        auto pt(transform(trafo, mesh.vertices[face.v[i]]));
        tri[i] = {float(pt(0)), float(pt(1)), float(pt(2))};
    }

    std::vector<imgproc::Scanline> scanlines;
    imgproc::scanConvertTriangle(tri, 0, qbuffer.rows, scanlines);

    for (const auto& sl : scanlines) {
        bool covered(false);
        imgproc::processScanline(sl, 0, qbuffer.cols, [&](int x, int y, float)
        {
            if (qbuffer.at<QBufferCType>(y, x) == index) {
                covered = true;
            }
        });
        if (covered) { return true; }
    }

    // do one more check in case the triangle is thinner than one pixel
    for (int i = 0; i < 3; i++) {
        int x(round(tri[i].x)), y(round(tri[i].y));
        clampMatPos(x, y, qbuffer);
        if (qbuffer.at<QBufferCType>(y, x) == index) { return true; }
    }

    return false;
}

//! Draws a face into a matrix. Used to mark useful areas of an atlas.
//!
void markFace(const geometry::opencv::ClipTriangle &face
              , cv::Mat/*<char>*/ &mat)
{
    cv::Point3f tri[3];
    for (int i = 0; i < 3; i++) {
        tri[i] = {face.uv[i].x, face.uv[i].y, 0};

        // make sure the vertices are always marked
        int x(round(tri[i].x)), y(round(tri[i].y));
        if (validMatPos(x, y, mat)) {
            mat.at<char>(y, x) = 1;
        }
    }

    std::vector<imgproc::Scanline> scanlines;
    imgproc::scanConvertTriangle(tri, 0, mat.rows, scanlines);

    for (const auto& sl : scanlines) {
        imgproc::processScanline(sl, 0, mat.cols,
            [&](int x, int y, float){ mat.at<char>(y, x) = 1; } );
    }
}

//! A wrapper for cv::findContours to solve for the 1-pixel border that the cv
//! function ignores.
//!
void findContours(const cv::Mat &mat,
                  std::vector<std::vector<cv::Point> > &contours)
{
    cv::Mat tmp(mat.rows + 2, mat.cols + 2, mat.type(), cv::Scalar(0));
    cv::Rect rect(1, 1, mat.cols, mat.rows);
    mat.copyTo(cv::Mat(tmp, rect));

    cv::findContours(tmp, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE,
                     cv::Point(-1, -1));
}

//! Clips faces to tile boundaries.
//!
void clipFaces(geometry::opencv::ClipTriangle::list &faces
               , const math::Size2f &tileSize)
{
    math::Size2f ts2(tileSize.width / 2.0, tileSize.height / 2.0);

    geometry::opencv::ClipPlane planes[4] = {
        { 1.,  0., 0., ts2.width},
        {-1.,  0., 0., ts2.width},
        { 0.,  1., 0., ts2.height},
        { 0., -1., 0., ts2.height}
    };

    for (int i = 0; i < 4; i++) {
        faces = clipTriangles(faces, planes[i]);
    }
}

//! Convert from normalized to pixel-based texture coordinates.
//!
imgproc::UVCoord denormalizeUV(const math::Point3d &uv, cv::Size texSize)
{
    return {float(uv(0) * texSize.width),
            //float(texSize.height-1 - uv(1)*texSize.height)
            float((1.0 - uv(1)) * texSize.height)};
}

//! Convert from pixel-based to normalized texture coordinates.
//!
math::Point3d normalizeUV(const imgproc::UVCoord &uv, cv::Size texSize)
{
    return {double(uv.x) / texSize.width,
            //(texSize.height-1 - double(uv.y)) / texSize.height,
            1.0 - double(uv.y) / texSize.height,
            0.0};
}


// helpers to convert between math points and cv points
cv::Point3d vertex(const math::Point3d &v)
    { return {v(0), v(1), v(2)}; }

math::Point3d vertex(const cv::Point3d &v)
    { return {v.x, v.y, v.z}; }    

cv::Point2d tCoord(const math::Point2 &t)
    { return { t(0), t(1)}; }

math::Point2 tCoord(const cv::Point2d &t)
    { return {t.x, t.y}; }        

//! Calculate a bounding rectangle of a list of points in the UV space.
//!
imgproc::UVRect makeRect(const std::vector<cv::Point> &points)
{
    imgproc::UVRect rect;
    for (const auto &pt : points) {
        rect.update(imgproc::UVCoord(pt.x, pt.y));
    }
    return rect;
}

//! Finds a UV rectangle containing the given face.
//!
unsigned findRect(const geometry::opencv::ClipTriangle &face
                  , const std::vector<imgproc::UVRect> &rects)
{
    const auto &pt(face.uv[0]);
    for (unsigned i = 0; i < rects.size(); i++)
    {
        const imgproc::UVRect &r(rects[i]);
        const float safety(0.51);

        if (pt.x > (r.min.x - safety) && pt.x < (r.max.x + safety) &&
            pt.y > (r.min.y - safety) && pt.y < (r.max.y + safety))
        {
            return i;
        }
    }
    LOGONCE(err2) << "Rectangle not found (pt = " << pt << ").";
    return 0;
}

//! Copies a rectangle from one atlas to another (src -> dst).
//!
void copyRect(const imgproc::UVRect &rect, const cv::Mat &src, cv::Mat &dst)
{
    for (int y = 0; y < rect.height(); y++)
    for (int x = 0; x < rect.width(); x++)
    {
        int sx(rect.x() + x), sy(rect.y() + y);
        clampMatPos(sx, sy, src);

        int dx(rect.packX + x), dy(rect.packY + y);
        if (!validMatPos(dx, dy, dst)) continue;

        dst.at<cv::Vec3b>(dy, dx) = src.at<cv::Vec3b>(sy, sx);
    }
}

//! Returns a trasformation of the (2x larger) fallback tile so that its given
//! quadrant aligns with tile of size `tileSize`.
//!
math::Matrix4 quadrantTransform(const math::Size2f &tileSize, int fallbackQuad)
{
    math::Matrix4 trafo(ublas::identity_matrix<double>(4));
    math::Size2f shift(tileSize.width / 2.0, tileSize.height / 2.0);

    switch (fallbackQuad) {
    case 0: trafo(0,3) = +shift.width, trafo(1,3) = +shift.height; break;
    case 1: trafo(0,3) = -shift.width, trafo(1,3) = +shift.height; break;
    case 2: trafo(0,3) = +shift.width, trafo(1,3) = -shift.height; break;
    case 3: trafo(0,3) = -shift.width, trafo(1,3) = -shift.height; break;
    }
    return trafo;
}

math::Matrix4 tileTransform(const math::Size2f &dstTileSize
                            , const math::Point2 dst
                            , const math::Size2f &srcTileSize
                            , const math::Point2 src)
{
    math::Matrix4 trafo(ublas::identity_matrix<double>(4));

    math::Size2f shiftSrc(srcTileSize.width / 2.0, srcTileSize.height / 2.0);
    math::Size2f shiftDst(dstTileSize.width / 2.0, dstTileSize.height / 2.0);

    math::Point2 quadrantShift(src - dst);

    trafo(0,3) = shiftSrc.width+quadrantShift(0)-shiftDst.width;
    trafo(1,3) = shiftSrc.height+quadrantShift(1)-shiftDst.height;

    return trafo;
}

MergeInput::list sortMergeInput(const MergeInput::list &mergeInput
                                , const math::Size2f &tileSize)
{
    /** Sorts via coarseness. Geometry-based coarsness is computed in lazy way.
     */
    struct TileSortInfo {
        std::size_t id;
        double coarseness;

        TileSortInfo(std::size_t id, const Tile &tile
                     , const math::Size2f &tileSize)
            : id(id), coarseness(tile.metanode.coarseness)
            , tile_(&tile), tileSize_(tileSize)
            , invGeometryCoarseness_(-1)
        {}

        double invGeometryCoarseness() const {
            if (invGeometryCoarseness_ < 0) {
                invGeometryCoarseness_
                    = tileInvGeometryCoarseness(*tile_, tileSize_);
            }
            return invGeometryCoarseness_;
        }

    private:
        const Tile *tile_;
        math::Size2f tileSize_;
        mutable double invGeometryCoarseness_;
    };

    std::vector<TileSortInfo> sortingInfos;
    bool sortinvGeometryCoarseness = false;
    for (unsigned i = 0; i < mergeInput.size(); ++i) {
        if (mergeInput[i].tile().metanode.coarseness < 0){
            sortinvGeometryCoarseness = true;
        }
        sortingInfos.emplace_back(i, mergeInput[i].tile(), tileSize);
    }

    auto compareCoarseness
        ([](const TileSortInfo &a, const TileSortInfo &b) -> bool
    {
        // tiles with lower coarseness comes last
        if (a.coarseness > b.coarseness) {
            return true;
        } else if (a.coarseness < b.coarseness) {
            return false;
        }

        // fallback if coarsenesses aren't set or are equal
        return (a.invGeometryCoarseness() < b.invGeometryCoarseness());
    });

    auto compareinvGeometryCoarseness
        ([](const TileSortInfo &a, const TileSortInfo &b) -> bool
    {
        // tiles with higher geometry quality comes last
        return (a.invGeometryCoarseness() < b.invGeometryCoarseness());
    });

    if (sortinvGeometryCoarseness){
        std::sort(sortingInfos.begin(), sortingInfos.end()
                  , compareinvGeometryCoarseness);
    } else{
        std::sort(sortingInfos.begin(), sortingInfos.end()
                  , compareCoarseness);
    }

    MergeInput::list result;
    for (const auto &tq : sortingInfos) {
        result.push_back(mergeInput[tq.id]);
    }

    return result;
}

Mesh removeDuplicateVertexes(geometry::opencv::ClipTriangle::list & faces
                             , cv::Size atlasSize)
{
    Mesh result;
    geometry::PointIndex<geometry::opencv::ClipTriangle::Point> vindex;
    geometry::PointIndex<geometry::opencv::ClipTriangle::TCoord> tindex;

    for (const auto &face1 : faces)
    {
        result.facets.emplace_back();
        Mesh::Facet &face2(result.facets.back());

        for (int i = 0; i < 3; i++) {
            face2.v[i] = vindex.assign(face1.pos[i]);
            if (unsigned(face2.v[i]) >= result.vertices.size()) {
                result.vertices.push_back(vertex(face1.pos[i]));
            }

            face2.t[i] = tindex.assign(face1.uv[i]);
            if (unsigned(face2.t[i]) >= result.texcoords.size()) {
                auto uv(normalizeUV(face1.uv[i], atlasSize));
                result.texcoords.push_back(uv);
            }
        }
    }

    return result;
}


Atlas mergeAtlases( const std::vector<Atlas*> & atlases
                  , geometry::opencv::ClipTriangle::list & faces
                    , float inflate = 0)
{
    std::vector<cv::Mat> marks;
    cv::Size safety(1, 1);

    for(uint i=0; i<atlases.size(); ++i){
        marks.emplace_back(atlases[i]->size() + safety, CV_8U, cv::Scalar(0));
    }
    for (const geometry::opencv::ClipTriangle &face : faces) {
        markFace(face, marks[face.id1]);       
    }

    // determine UV rectangles that will get repacked
    std::vector<std::vector<imgproc::UVRect> > rects;
    for (const auto &m : marks)
    {
        std::vector<std::vector<cv::Point> > contours;
        findContours(m, contours);

        // add one UVRect for each connected component in the mark matrix
        rects.emplace_back();
        for (const auto& c : contours) {
            rects.back().push_back(makeRect(c));
        }

#if DEBUG
        cv::Mat dbg(m.size(), CV_8UC3, cv::Scalar(0,0,0));
        for (const auto &c : contours) {
            cv::Vec3b color(rand() % 256, rand() % 256, rand() % 256);
            for (const cv::Point &pt : c) {
                dbg.at<cv::Vec3b>(pt.y, pt.x) = color;
            }
        }
        static int idx = 0;
        auto filename(str(boost::format("merge%03d.png") % (idx++)));
        cv::imwrite(filename, dbg);
#endif
    }

    // recalculate the rectangles directly from face UVs for subpixel precision
    for (auto &face : faces) {
        face.id2 = findRect(face, rects[face.id1]);
    }
    for (auto &rlist : rects) {
        for (auto &r : rlist)
            r.clear();
    }
    for (const auto &face : faces) {
        for (int i = 0; i < 3; i++)
            rects[face.id1][face.id2].update(face.uv[i]);
    }

    //inflate rectangles
    if(inflate>0){
        for (auto &rlist : rects) {
            for (auto &r : rlist)
                r.inflate(inflate);
        }
    }

    // pack the rectangles
    imgproc::RectPacker packer;
    for (auto &rlist : rects) {
        for (auto &r : rlist)
            packer.addRect(&r);
    }
    packer.pack();

    // copy rectangles to final atlas
    cv::Mat atlas(packer.height(), packer.width(), CV_8UC3, cv::Scalar(0,0,0));
    for (unsigned i = 0; i < atlases.size(); i++) {
        for (const auto &rect : rects[i]) {
            // TODO: mask out pixels that are not needed!
            copyRect(rect, *atlases[i], atlas);
        }
    }

    // adjust face UVs to point to the new atlas
    for (auto &face : faces) {
        for (int i = 0; i < 3; i++)
            rects[face.id1][face.id2].adjustUV(face.uv[i]);
    }

    return atlas;
}

int sameIndices(const cv::Mat &qbuffer)
{
    int index = -1;
    for (auto it = qbuffer.begin<QBufferCType>();
         it != qbuffer.end<QBufferCType>(); ++it)
    {
        const auto value(*it);
        if (value < 0) { continue; }
        if (value != index) {
            if (index < 0) {
                index = value;
            } else {
                return -1;
            }
        }
    }
    return index;
}

//! Returns true if 'qbuffer' contains at least one element equal to 'index'.
//!
bool haveIndices(const cv::Mat &qbuffer, int index)
{
    for (auto it = qbuffer.begin<QBufferCType>();
         it != qbuffer.end<QBufferCType>(); ++it)
    {
        if (*it == index) return true;
    }
    return false;
}

//! Bilinearly upsamples the specified quadrant of the fallback tile heightmap.
//!
void getFallbackHeightmap(const Tile &fallback, int fallbackQuad,
                          float heightmap[MetaNode::HMSize][MetaNode::HMSize])
{
    const int hms(MetaNode::HMSize);

    double x(0.), y(0.);
    if (fallbackQuad & 1) x += double(hms - 1)*0.5;
    if (fallbackQuad & 2) y += double(hms - 1)*0.5;

    for (int i = 0; i < hms; i++)
    for (int j = 0; j < hms; j++)
    {
        math::Point2 point(x + 0.5*j, y + 0.5*i);
        heightmap[i][j] = 0;
        imgproc::bilinearInterpolate(
                   (float*) fallback.metanode.heightmap, hms, hms, hms, 1,
                   point, &heightmap[i][j]);
    }
}

MergeInput clipQuad( const MergeInput & mergeInput, int fallbackQuad
                     , const math::Size2f &tileSize)
{

    math::Matrix4 shift = quadrantTransform(tileSize, fallbackQuad);
    const Tile &tile(mergeInput.tile());
    const Mesh &mesh(tile.mesh);
    cv::Size asize(tile.atlas.size());
    asize.width *= 2;
    asize.height *= 2;

    geometry::opencv::ClipTriangle::list faces;
    for (unsigned j = 0; j < mesh.facets.size(); j++) {
        const Mesh::Facet &face(mesh.facets[j]);
        faces.emplace_back(0,0,
                vertex(transform(shift,mesh.vertices[face.v[0]])),
                vertex(transform(shift,mesh.vertices[face.v[1]])),
                vertex(transform(shift,mesh.vertices[face.v[2]])),
                denormalizeUV(mesh.texcoords[face.t[0]], asize),
                denormalizeUV(mesh.texcoords[face.t[1]], asize),
                denormalizeUV(mesh.texcoords[face.t[2]], asize));
    }
    clipFaces(faces, tileSize); 
    
    Atlas atlas(asize.height, asize.width, tile.atlas.type());
    cv::resize( tile.atlas, atlas, atlas.size()
              , 0, 0, cv::INTER_LANCZOS4);


    std::vector<Atlas*> usedAtlases;
    usedAtlases.push_back(&atlas);
    
    Tile resultTile;
    resultTile.atlas = mergeAtlases(usedAtlases, faces, 2);
    resultTile.mesh = removeDuplicateVertexes(faces, resultTile.atlas.size());
    resultTile.metanode = tile.metanode;
    getFallbackHeightmap(tile, fallbackQuad, resultTile.metanode.heightmap);

    return { resultTile, &mergeInput.tileSet()
           , mergeInput.tileId(), mergeInput.srcFaceCount() };
}

} // namespace

//! Merges several tiles. The process has four phases:
//!
//! 1. AncestorTiles are clipped and transformed according to 'quad'.
//!    The meshes of all tiles are rasterized into a buffer which determines 
//!    the source of geometry at each point of the result. The tiles are 
//!    rasterized from worst to best (in terms of their geometry quality). 
//!    This means that the best tile will be complete in the result and only 
//!    the remaining space will be filled with other tiles.
//!
//! 2. Faces that will appear in the output are collected, according to the
//!    contents of the buffer. A face from a tile is used if at least one
//!    underlying element of the qbuffer indicates that the tile should be used
//!    at that place.
//!
//! 3. Pixels in atlases that are needed by the collected faces are marked.
//!    Rectangles that cover the marked areas are identified and then repacked
//!    into a single new atlas.
//!
//! 4. The isolated faces are converted into a standard mesh with vertices and
//!    texture vertices that don't repeat.
//!
MergedTile merge( const TileId &tileId, const math::Size2f &tileSize
                , const MergeInput::list &mergeInput
                , int quad
                , const MergeInput::list &ancestorTiles
                , MergeInput::list &incidentTiles)
{
    LOG(info2)<<"Merging tile "<<tileId<<" from "<<mergeInput.size()<<" sets";

    // sort tiles by quality
    auto sortedMergeInput = sortMergeInput(mergeInput, tileSize);
    // create qbuffer, rasterize meshes in increasing order of quality
    const int QBSize(512);
    cv::Mat qbuffer(QBSize, QBSize, QBufferCvType, cv::Scalar(-1));
    math::Matrix4 trafo(tileToBuffer(tileSize, QBSize));

    std::set<const TileSet*> mergedTileSetMap;
    for (const auto &mi : sortedMergeInput) {
        mergedTileSetMap.insert(&mi.tileSet());
    }

    incidentTiles.clear();
    //clip and repack ancestor tiles
    for (const auto &fb : ancestorTiles){
        if (mergedTileSetMap.find(&fb.tileSet()) == mergedTileSetMap.end()) {
            MergeInput mi = clipQuad(fb, quad, tileSize);
            if (!mi.tile().mesh.facets.empty()) {
                incidentTiles.push_back(mi);
            }
        }
    }
    incidentTiles.insert(incidentTiles.end(), sortedMergeInput.begin()
                         , sortedMergeInput.end());

    for (uint i = 0; i<incidentTiles.size(); ++i) {
        rasterizeTile(incidentTiles[i].tile(), trafo, i, qbuffer);
    }

    // close small areas of bad stuff with good stuff
    close<1, 6>(qbuffer);

    int index(sameIndices(qbuffer));
    if (index > -1) {
        return { incidentTiles[index].tile(), incidentTiles[index].tileSet() };
    }

    std::vector<Atlas*> usedAtlases;
    geometry::opencv::ClipTriangle::list mergedFaces;
    const int hms(MetaNode::HMSize);
    float minGsd = std::numeric_limits<float>::max();
    float heightmap[hms][hms];
    cv::Mat hmask(hms, hms, CV_32S, cv::Scalar(0));

    // result tile
    MergedTile result;

    for(int i=incidentTiles.size() - 1; i>=0; --i) {
        if(haveIndices(qbuffer,i)){

            const Tile &tile(incidentTiles[i].tile());
            cv::Size asize(tile.atlas.size());

            geometry::Obj mesh = tile.mesh;

            //refine mesh if necessary
            if (incidentTiles[i].srcFaceCount() > tile.mesh.facets.size()) {
                // tile has been clipped -> refine
                auto cmesh = *geometry::asMesh(mesh);
                auto rmesh(geometry::refine( cmesh
                                       , incidentTiles[i].srcFaceCount()));
                mesh = geometry::asObj(rmesh);
            }

            usedAtlases.emplace_back(&incidentTiles[i].tile().atlas);
            for (unsigned j = 0; j < mesh.facets.size(); j++) {
                const Mesh::Facet &face(mesh.facets[j]);
                if (faceCovered(mesh, face, trafo, i, qbuffer)) {
                    mergedFaces.emplace_back(usedAtlases.size()-1,0,
                            vertex(mesh.vertices[face.v[0]]),
                            vertex(mesh.vertices[face.v[1]]),
                            vertex(mesh.vertices[face.v[2]]),
                            denormalizeUV(mesh.texcoords[face.t[0]], asize),
                            denormalizeUV(mesh.texcoords[face.t[1]], asize),
                            denormalizeUV(mesh.texcoords[face.t[2]], asize) );
                }
            }

            //update metadata
            minGsd = std::min(tile.metanode.gsd, minGsd);
            for (int c = 0; c < hms; c++)
            for (int r = 0; r < hms; r++)
            {
                int what = qbuffer.at<QBufferCType>( c * (QBSize-1) / hms
                                                     , r * (QBSize-1) / hms);
                if(what==i || hmask.at<int>(c,r)==0){
                    hmask.at<int>(c,r)=1;
                    heightmap[c][r] = tile.metanode.heightmap[c][r];
                }
            }

            // remember tile's set as origin of this tile
            result.sources.push_back(&incidentTiles[i].tileSet());
        }
    }

    // almost done
    result.atlas = mergeAtlases(usedAtlases, mergedFaces);
    result.mesh = removeDuplicateVertexes(mergedFaces, result.atlas.size());

    if(sortedMergeInput.size()>0){
        result.metanode = sortedMergeInput.front().tile().metanode;
    }

    result.metanode.gsd = minGsd;
    //coarseness is not used after merge - set to invalid value
    result.metanode.coarseness = -1;

    // copy heightmap
    std::copy(&heightmap[0][0], &heightmap[hms - 1][hms]
              , &result.metanode.heightmap[0][0]);

    return result;
}

boost::optional<double> MergedTile::pixelSize() const
{
    // pixel size is valid only when tile is from single source and has valid
    // pixelsize
    if (singleSource() && metanode.exists()) {
        return metanode.pixelSize[0][0];
    }
    return boost::none;
}

} } // namespace vtslibs::vts0
