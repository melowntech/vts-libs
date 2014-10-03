#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "math/geometry.hpp"
#include "math/transform.hpp"
#include "imgproc/scanconversion.hpp"
#include "imgproc/binterpolate.hpp"

#include "vadstena-libs/faceclip.hpp"
#include "vadstena-libs/pointindex.hpp"
#include "vadstena-libs/uvpack.hpp"

#include "./merge.hpp"
#include "./io.hpp"

#include "../binmesh.hpp"

namespace ublas = boost::numeric::ublas;

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

namespace vadstena { namespace tilestorage {

namespace {

double triangleArea(const math::Point3 &a, const math::Point3 &b,
                    const math::Point3 &c)
{
    return norm_2(math::crossProduct(b - a, c - a)) * 0.5;
}

//! Calculates tile quality, defined as the ratio of texel area and mesh area.
//!
double tileQuality(const Tile &tile)
{
    // calculate the total area of the faces in both the XYZ and UV spaces
    double xyzArea(0), uvArea(0);
    const Mesh &mesh(tile.mesh);

    for (const auto &face : mesh.facets)
    {
        xyzArea += triangleArea(mesh.vertices[face.v[0]],
                                mesh.vertices[face.v[1]],
                                mesh.vertices[face.v[2]]);

        uvArea += triangleArea(mesh.texcoords[face.t[0]],
                               mesh.texcoords[face.t[1]],
                               mesh.texcoords[face.t[2]]);
    }
    uvArea *= tile.atlas.cols * tile.atlas.rows;

    // return the number of texels per unit area
    return uvArea / xyzArea;
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
                   { qbuffer.at<int>(y, x) = index; } );
        }
    }
}

//! "Draws" all faces of a tile into a zbuffer. Only pixels allowed by the
//! qbuffer are modified.
//!
void rasterizeHeights(const Tile &tile, const math::Matrix4 &trafo,
                      int index, const cv::Mat &qbuffer, cv::Mat &zbuffer)
{
    std::vector<imgproc::Scanline> scanlines;

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
                [&](int x, int y, float z) {
                   if (qbuffer.at<int>(y, x) == index) {
                       float &height(zbuffer.at<float>(y, x));
                       if (z > height) height = z;
                   };
                } );
        }
    }
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

//! Dilates indices of a qbuffer to reduce areas with index -1 (fallback).
//!
template<typename MatType>
void dilateIndices(cv::Mat &mat)
{
    cv::Mat result(mat.size(), mat.type());
    for (int y = 0; y < mat.rows; y++)
    for (int x = 0; x < mat.cols; x++)
    {
        MatType maximum(-1);

        for (int i = -3; i <= 3; i++) {
            if (y+i < 0 || y+i >= mat.rows) continue;

            for (int j = -3; j <= 3; j++) {
                if (x+j < 0 || x+j >= mat.cols) continue;

                MatType value = mat.at<MatType>(y+i, x+j);
                maximum = std::max(value, maximum);
            }
        }
        result.at<MatType>(y, x) = maximum;
    }
    result.copyTo(mat);
}

//! Inverse to dilateIndices.
//!
template<typename MatType>
void erodeIndices(cv::Mat &mat)
{
    cv::Mat result(mat.size(), mat.type());
    for (int y = 0; y < mat.rows; y++)
    for (int x = 0; x < mat.cols; x++)
    {
        MatType minimum(mat.at<MatType>(y, x));

        for (int i = -3; i <= 3; i++) {
            if (y+i < 0 || y+i >= mat.rows) continue;

            for (int j = -3; j <= 3; j++) {
                if (x+j < 0 || x+j >= mat.cols) continue;

                MatType value = mat.at<MatType>(y+i, x+j);
                if (value < 0) minimum = -1;
            }
        }
        result.at<MatType>(y, x) = minimum;
    }
    result.copyTo(mat);
}

//! Returns true if at least one element in the area of the qbuffer
//! corresponding to the given face belongs to tile number 'index'.
//!
bool faceCovered(const Mesh &mesh, const Mesh::Facet &face,
                 const math::Matrix4 &trafo, int index,
                 const cv::Mat/*<int>*/ &qbuffer)
{
    cv::Point3f tri[3];
    for (int i = 0; i < 3; i++) {
        auto pt(transform(trafo, mesh.vertices[face.v[i]]));
        tri[i] = {float(pt(0)), float(pt(1)), float(pt(2))};
    }

    std::vector<imgproc::Scanline> scanlines;
    imgproc::scanConvertTriangle(tri, 0, qbuffer.rows, scanlines);

    bool covered(false);
    for (const auto& sl : scanlines) {
        imgproc::processScanline(sl, 0, qbuffer.cols,
            [&](int x, int y, float)
               { if (qbuffer.at<int>(y, x) == index) covered = true; } );
    }

    if (!covered) {
        // do one more check in case the triangle is thinner than one pixel
        for (int i = 0; i < 3; i++) {
            int x(round(tri[i].x)), y(round(tri[i].y));
            clampMatPos(x, y, qbuffer);
            if (qbuffer.at<int>(y, x) == index) covered = true;
        }
    }

    return covered;
}

//! Draws a face into a matrix. Used to mark useful areas of an atlas.
//!
void markFace(const ClipTriangle &face, cv::Mat/*<char>*/ &mat)
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
void clipFaces(ClipTriangle::list &faces, long tileSize)
{
    double ts2(double(tileSize)/2);

    ClipPlane planes[4] = {
        { 1.,  0., 0., ts2},
        {-1.,  0., 0., ts2},
        { 0.,  1., 0., ts2},
        { 0., -1., 0., ts2}
    };

    for (int i = 0; i < 4; i++) {
        faces = clipTriangles(faces, planes[i]);
    }
}

//! Returns a trasformation from tile local coordinates to qbuffer coordinates.
//!
math::Matrix4 tileToBuffer(long tileSize, int bufferSize)
{
    math::Matrix4 trafo(ublas::identity_matrix<double>(4));
    trafo(0,0) = trafo(1,1) = double(bufferSize) / tileSize;
    trafo(0,3) = trafo(1,3) = double(bufferSize) / 2;
    return trafo;
}

//! Returns a trasformation of the (2x larger) fallback tile so that its given
//! quadrant aligns with tile of size `tileSize`.
//!
math::Matrix4 quadrantTransform(long tileSize, int fallbackQuad)
{
    math::Matrix4 trafo(ublas::identity_matrix<double>(4));
    double shift(double(tileSize) / 2);

    switch (fallbackQuad) {
    case 0: trafo(0,3) = +shift, trafo(1,3) = +shift; break;
    case 1: trafo(0,3) = -shift, trafo(1,3) = +shift; break;
    case 2: trafo(0,3) = +shift, trafo(1,3) = -shift; break;
    case 3: trafo(0,3) = -shift, trafo(1,3) = -shift; break;
    }
    return trafo;
}

//! Returns true if 'qbuffer' is filled with the same value (returned in 'index')
//!
bool sameIndices(const cv::Mat &qbuffer, int& index)
{
    index = qbuffer.at<int>(0, 0);
    for (auto it = qbuffer.begin<int>(); it != qbuffer.end<int>(); ++it) {
        if (*it != index) return false;
    }
    return true;
}

//! Returns true if 'qbuffer' contains at least one element equal to 'index'.
//!
bool haveIndices(const cv::Mat &qbuffer, int index)
{
    for (auto it = qbuffer.begin<int>(); it != qbuffer.end<int>(); ++it) {
        if (*it == index) return true;
    }
    return false;
}

//! Calculate a bounding rectangle of a list of points in the UV space.
//!
UVRect makeRect(const std::vector<cv::Point> &points)
{
    UVRect rect;
    for (const auto &pt : points) {
        rect.update(UVCoord(pt.x, pt.y));
    }
    return rect;
}

//! Finds a UV rectangle containing the given face.
//!
unsigned findRect(const ClipTriangle &face, const std::vector<UVRect> &rects)
{
    const auto &pt(face.uv[0]);
    for (unsigned i = 0; i < rects.size(); i++)
    {
        const UVRect &r(rects[i]);
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
void copyRect(const UVRect &rect, const cv::Mat &src, cv::Mat &dst)
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

//! Convert from normalized to pixel-based texture coordinates.
//!
UVCoord denormalizeUV(const math::Point3d &uv, cv::Size texSize)
{
    return {float(uv(0) * texSize.width),
            //float(texSize.height-1 - uv(1)*texSize.height)
            float((1.0 - uv(1)) * texSize.height)};
}

//! Convert from pixel-based to normalized texture coordinates.
//!
math::Point3d normalizeUV(const UVCoord &uv, cv::Size texSize)
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

} // namespace

//! Merges several tiles. The process has four phases:
//!
//! 1. The meshes are rasterized into a buffer which determines the source
//!    of geometry at each point of the result. The tiles are rasterized from
//!    worst to best (in terms of their texture resolution). This means that the
//!    best tile will be complete in the result and only the remaining space
//!    will be filled with other tiles. The fallback tile is used only where
//!    no other geometry exists.
//!
//! 2. Faces that will appear in the output are collected, according to the
//!    contents of the buffer. A face from a tile is used if at least one
//!    underlying element of the qbuffer indicates that the tile should be used
//!    at that place. In addition, the fallback faces are transformed according
//!    to `fallbackQuad` and clipped.
//!
//! 3. Pixels in atlases that are needed by the collected faces are marked.
//!    Rectangles that cover the marked areas are identified and then repacked
//!    into a single new atlas.
//!
//! 4. The isolated faces are converted into a standard mesh with vertices and
//!    texture vertices that don't repeat.
//!
MergedTile merge(const TileId &tileId, long tileSize, const Tile::list &tiles
                 , const Tile &fallback, int fallbackQuad)
{
    (void) tileId;
#if DEBUG
    {
        LOG(info2)
            << "Saving fallback tile on merge. Info:"
            << "\ntileSize: " << tileSize
            << "\nfallbackQuad: " << fallbackQuad
            ;
        writeBinaryMesh("./merge-fallback.bin", fallback.mesh);
        cv::imwrite("./merge-fallback.png", fallback.atlas);
    }
#endif

    // sort tiles by quality
    typedef std::pair<unsigned, double> TileQuality;
    std::vector<TileQuality> qualities;

    for (unsigned i = 0; i < tiles.size(); i++) {
        qualities.emplace_back(i, tileQuality(tiles[i]));
    }
    std::sort(qualities.begin(), qualities.end(),
              [](const TileQuality &a, const TileQuality &b)
                { return a.second < b.second; } );

    // create qbuffer, rasterize meshes in increasing order of quality
    const int QBSize(512);
    cv::Mat qbuffer(QBSize, QBSize, CV_32S, cv::Scalar(-1));

    math::Matrix4 trafo(tileToBuffer(tileSize, QBSize));
    for (const TileQuality &tq : qualities) {
        rasterizeTile(tiles[tq.first], trafo, tq.first, qbuffer);
    }
#if DEBUG
    std::ofstream("qbuffer.m") << "QB = " << qbuffer << ";\n";
#endif

    // close small holes in the meshes so that the fallback doesn't show through
    // unneccessarily
    const int closeSteps(2);
    for (int i = 0; i < closeSteps; i++) {
        dilateIndices<int>(qbuffer);
    }
    for (int i = 0; i < closeSteps; i++) {
        erodeIndices<int>(qbuffer);
    }
#if DEBUG
    std::ofstream("qbufferclosed.m") << "QB = " << qbuffer << ";\n";
#endif

    // optimization: if a tile covers the entire area, return it and do nothing
    int index;
    if (sameIndices(qbuffer, index) && index >= 0) {
        return { tiles[index], true };
    }

    // create a z-buffer
    /*const float InvalidHeight(-123);
    cv::Mat zbuffer(QBSize, QBSize, CV_32F, cv::Scalar(InvalidHeight));
    for (unsigned i = 0; i < tiles.size(); i++) {
        rasterizeHeights(tiles[i], trafo, i, qbuffer, zbuffer);
    }*/

    // collect faces that are allowed by the qbuffer
    ClipTriangle::list faces;
    for (unsigned i = 0; i < tiles.size(); i++) {
        const Tile &tile(tiles[i]);
        const Mesh &mesh(tile.mesh);
        cv::Size asize(tile.atlas.size());

        for (unsigned j = 0; j < mesh.facets.size(); j++) {
            const Mesh::Facet &face(mesh.facets[j]);

            if (faceCovered(mesh, face, trafo, i, qbuffer)) {
                faces.emplace_back(i, 0,
                        vertex(mesh.vertices[face.v[0]]),
                        vertex(mesh.vertices[face.v[1]]),
                        vertex(mesh.vertices[face.v[2]]),
                        denormalizeUV(mesh.texcoords[face.t[0]], asize),
                        denormalizeUV(mesh.texcoords[face.t[1]], asize),
                        denormalizeUV(mesh.texcoords[face.t[2]], asize) );
            }
        }
    }

    // get faces also from the fallback tile, if available and if necessary
    bool doFb(fallbackQuad >= 0 && haveIndices(qbuffer, -1));
    if (doFb)
    {
        const Mesh &mesh(fallback.mesh);
        cv::Size asize(fallback.atlas.size());

        math::Matrix4 quadtr(quadrantTransform(tileSize, fallbackQuad));
        math::Matrix4 trafo2(prod(trafo, quadtr));

        ClipTriangle::list fbFaces;
        unsigned fbIndex(tiles.size());

        for (unsigned j = 0; j < mesh.facets.size(); j++) {
            const Mesh::Facet &face(mesh.facets[j]);

            if (faceCovered(mesh, face, trafo2, -1, qbuffer)) {
                fbFaces.emplace_back(fbIndex, 0,
                          vertex(transform(quadtr, mesh.vertices[face.v[0]])),
                          vertex(transform(quadtr, mesh.vertices[face.v[1]])),
                          vertex(transform(quadtr, mesh.vertices[face.v[2]])),
                          denormalizeUV(mesh.texcoords[face.t[0]], asize),
                          denormalizeUV(mesh.texcoords[face.t[1]], asize),
                          denormalizeUV(mesh.texcoords[face.t[2]], asize) );
            }
        }

        clipFaces(fbFaces, tileSize);
        faces.insert(faces.end(), fbFaces.begin(), fbFaces.end());
    }

    // mark areas in the atlases that will need to be repacked
    std::vector<cv::Mat> marks;
    cv::Size safety(1, 1);
    for (const auto& tile : tiles) {
        marks.emplace_back(tile.atlas.size() + safety, CV_8U, cv::Scalar(0));
    }
    if (doFb) {
        marks.emplace_back(fallback.atlas.size() + safety, CV_8U, cv::Scalar(0));
    }
    for (const ClipTriangle &face : faces) {
        markFace(face, marks[face.id1]);
    }

    // determine UV rectangles that will get repacked
    std::vector<std::vector<UVRect> > rects;
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

    // pack the rectangles
    RectPacker packer;
    for (auto &rlist : rects) {
        for (auto &r : rlist)
            packer.addRect(&r);
    }
    packer.pack();

    // copy rectangles to final atlas
    cv::Mat atlas(packer.height(), packer.width(), CV_8UC3, cv::Scalar(0,0,0));
    for (unsigned i = 0; i < tiles.size(); i++) {
        for (const auto &rect : rects[i]) {
            // TODO: mask out pixels that are not needed!
            copyRect(rect, tiles[i].atlas, atlas);
        }
    }
    if (doFb) {
        for (const auto &rect : rects.back()) {
            // TODO: mask out pixels that are not needed!
            copyRect(rect, fallback.atlas, atlas);
        }
    }

    // adjust face UVs to point to the new atlas
    for (auto &face : faces) {
        for (int i = 0; i < 3; i++)
            rects[face.id1][face.id2].adjustUV(face.uv[i]);
    }

    // almost done
    MergedTile result;
    result.atlas = atlas;

    // create final geometry, with duplicate vertices removed
    Mesh &mesh(result.mesh);
    PointIndex<ClipTriangle::Point> vindex;
    PointIndex<ClipTriangle::TCoord> tindex;

    for (const auto &face1 : faces)
    {
        mesh.facets.emplace_back();
        Mesh::Facet &face2(mesh.facets.back());

        for (int i = 0; i < 3; i++) {
            face2.v[i] = vindex.assign(face1.pos[i]);
            if (unsigned(face2.v[i]) >= mesh.vertices.size()) {
                mesh.vertices.push_back(vertex(face1.pos[i]));
            }

            face2.t[i] = tindex.assign(face1.uv[i]);
            if (unsigned(face2.t[i]) >= mesh.texcoords.size()) {
                auto uv(normalizeUV(face1.uv[i], atlas.size()));
                mesh.texcoords.push_back(uv);
            }
        }
    }

    // make sure no vertex is higher than the zbuffer
    /*for (auto &v : mesh.vertices) {
        auto tv(transform(trafo, v));
        int x(round(tv(0))), y(round(tv(1)));

        if (validMatPos(x, y, zbuffer)) {
            const float &height(zbuffer.at<float>(y, x));
            if (height != InvalidHeight) {
                if (v(2) > height) v(2) = height;
            }
        }
    }*/

    // one more thing: merge the 5x5 heightfields
    if (tiles.size()) {
        result.metanode = tiles[qualities.back().first].metanode;
    }

    const int hms(MetaNode::HMSize);
    float fbheight[hms][hms];
    if (doFb) getFallbackHeightmap(fallback, fallbackQuad, fbheight);

    for (int i = 0; i < hms; i++)
    for (int j = 0; j < hms; j++)
    {
        int what = qbuffer.at<int>(i * (QBSize-1) / hms, j * (QBSize-1) / hms);
        if (what >= 0)
            result.metanode.heightmap[i][j] = tiles[what].metanode.heightmap[i][j];
        else if (doFb)
            result.metanode.heightmap[i][j] = fbheight[i][j];
    }

    // TODO: calculate this value
    result.singleSource = false;

    return result;
}

boost::optional<double> MergedTile::pixelSize() const
{
    if (singleSource) { return metanode.pixelSize[0][0]; }
    return boost::none;
}

} } // namespace vadstena::tilestorage
