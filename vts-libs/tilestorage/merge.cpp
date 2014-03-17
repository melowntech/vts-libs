#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "math/geometry.hpp"
#include "math/transform.hpp"
#include "imgproc/scanconversion.hpp"

#include "vadstena-libs/faceclip.hpp"
#include "vadstena-libs/pointindex.hpp"
#include "vadstena-libs/uvpack.hpp"

#include "./merge.hpp"

namespace ublas = boost::numeric::ublas;

#ifndef BUILDSYS_CUSTOMER_BUILD
#   define DEBUG 1 // save debug images
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

//! Returns true if at least one element in the area of the qbuffer
//! corresponding to the given face belongs to tile number 'index'.
//!
bool faceCovered(const Mesh &mesh, const Mesh::Facet &face,
                 const math::Matrix4 &trafo, int index, const cv::Mat &qbuffer)
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
        // do one more check in case the triangle is smaller than one pixel
        int x(round(tri[0].x)), y(round(tri[0].y));
        if (x >= 0 && x < qbuffer.cols && y >= 0 && y < qbuffer.rows) {
            covered = (qbuffer.at<int>(y, x) == index);
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
        tri[i] = {face.uv[i].x * mat.cols, face.uv[i].y * mat.rows, 0};

        // make sure the vertices are always marked
        int x(round(tri[i].x)), y(round(tri[i].y));
        if (x >= 0 && x < mat.cols && y >= 0 && y < mat.rows) {
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

//! Returns a trasformation from tile local coordinates to qbuffer coordinates.
//!
math::Matrix4 tileToBuffer(long tileSize, int bufferSize)
{
    math::Matrix4 trafo(ublas::identity_matrix<double>(4));
    trafo(0,0) = trafo(1,1) = double(bufferSize) / tileSize;
    trafo(0,3) = trafo(1,3) = double(bufferSize) / 2;
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

//! Calculate a bounding rectangle of a list of points in the UV space.
//!
UVRect makeRect(const std::vector<cv::Point> points)
{
    UVRect rect;
    for (const auto &pt : points) {
        rect.update(UVCoord(pt.x, pt.y));
    }
    return rect;
}

//!
//!
/*const UVRect& findRect(const ClipTriangle &face,
                       const std::vector<UVRect> &rects)
{
}*/

//! Convert from normalized to pixel-based texture coordinates.
//!
UVCoord denormalizeUV(const math::Point3d &uv, cv::Size texSize)
{
    return {float(uv(0)*texSize.width),
            float(texSize.height-1 - uv(1)*texSize.height)};
}

//! Convert from pixel-based to normalized texture coordinates.
//!
math::Point3d normalizeUV(const UVCoord &uv, cv::Size texSize)
{
    return {double(uv.x)/texSize.width,
            (texSize.height-1 - double(uv.y))/texSize.height,
            0.0};
}

// helpers to convert between math points and cv points
cv::Point3d vertex(const math::Point3d &v)
    { return {v(0), v(1), v(2)}; }

math::Point3d vertex(const cv::Point3d &v)
    { return {v.x, v.y, v.z}; }

} // namespace


//! TODO: describe
//!
//!
Tile merge(long tileSize, const Tile::list &tiles
           , const Tile &fallback, int fallbackQuad)
{
#if 0
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

    // optimization: if a tile covers the entire area, return it and do nothing
    int index;
    if (sameIndices(qbuffer, index) && index >= 0) {
        return tiles[index];
    }

    // collect faces that are allowed by the qbuffer
    ClipTriangle::list faces;
    for (unsigned i = 0; i < tiles.size(); i++) {
        const Tile &tile(tiles[i]);
        cv::Size asize(tile.atlas.size);

        for (unsigned j = 0; j < tile.mesh.facets.size(); j++) {
            const Mesh::Facet &face(tile.mesh.facets[j]);

            if (faceCovered(tile.mesh, face, trafo, i, qbuffer)) {
                faces.emplace_back(i, j,
                        vertex(tile.mesh.vertices[face.v[0]]),
                        vertex(tile.mesh.vertices[face.v[1]]),
                        vertex(tile.mesh.vertices[face.v[2]]),
                        denormalizeUV(tile.mesh.texcoords[face.t[0]], asize),
                        denormalizeUV(tile.mesh.texcoords[face.t[1]], asize),
                        denormalizeUV(tile.mesh.texcoords[face.t[2]], asize) );
            }
        }
    }

    // mark areas in the atlases that will need to be repacked
    std::vector<cv::Mat> marks;
    for (const auto& tile : tiles) {
        marks.emplace_back(tile.atlas.size(), CV_8U, cv::Scalar(0));
    }
    for (const ClipTriangle &face : faces) {
        markFace(face, marks[face.id1]);
    }

    // determine UV rectangles that will get repacked
    std::vector<std::vector<UVRect> > rects;
    for (const auto &m : marks)
    {
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(m, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

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

    // pack the rectangles
    RectPacker packer;
    for (auto &rlist : rects) {
        for (auto &r : rlist)
            packer.addRect(&r);
    }
    packer.pack();

    // copy rectangles to final atlas
    cv::Mat atlas(packer.height(), packer.width(), CV_8UC3, cv::Scalar(0,0,0));
    /*for (const auto &rlist : rects) {
        for (const auto &r : rlist) {
            copyRect(r, );
        }
    }*/



    // almost done
    Tile result;
    result.atlas = atlas;

    // create final geometry, with duplicate vertices removed
    Mesh &mesh(result.mesh);
    PointIndex<ClipTriangle::Point> vindex;
    PointIndex<ClipTriangle::TCoord> tindex;

    for (const ClipTriangle &face1 : faces)
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
                mesh.texcoords.push_back(tcoord(face1.uv[i]));
            }
        }
    }

    return result;

#else
    // simple no-merge algo :P

    LOG(info2) << "merging " << tiles.size() << " tiles";

    // find tile with largest triangle area
    const Tile *t(nullptr);
    double area(0);
    for (const auto &tile : tiles) {
        auto a(tileQuality(tile));
        if (a > area) {
            area = a;
            t = &tile;
        }
    }

    if (t) { return *t; }
    return {};
#endif

    (void) tileSize;
    (void) fallback;
    (void) fallbackQuad;
    (void) tiles;
}

} } // namespace vadstena::tilestorage
