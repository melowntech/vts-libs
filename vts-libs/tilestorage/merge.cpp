#include "math/geometry.hpp"

#include "./merge.hpp"

namespace vadstena { namespace tilestorage {

namespace {

double triangleArea(const math::Point3 &a, const math::Point3 &b,
                    const math::Point3 &c)
{
    return norm_2(math::crossProduct(b - a, c - a)) * 0.5;
}

double tileArea(const Mesh &mesh)
{
    double xyzArea(0);
    for (const auto &face : mesh.facets) {
        xyzArea += triangleArea(mesh.vertices[face.v[0]],
                                mesh.vertices[face.v[1]],
                                mesh.vertices[face.v[2]]);
    }
    return xyzArea;
}

} // namespace

Tile merge(long tileSize, const Tile::list &tiles
           , const Tile &fallback, int fallbackQuad)
{
    // TODO: implement me

    // simple no-merge algo :P

    // find tile with largest triangle area
    const Tile *t(nullptr);
    double area(0);
    for (const auto &tile : tiles) {
        auto a(tileArea(tile.mesh));
        if (a > area) {
            area = a;
            t = &tile;
        }
    }

    if (t) { return *t; }
    return {};

    (void) tileSize;
    (void) fallback;
    (void) fallbackQuad;
    (void) tiles;
}

} } // namespace vadstena::tilestorage
