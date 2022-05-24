/**
 * Copyright (c) 2020 Melown Technologies SE
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
#include <string>
#include <iostream>
#include <algorithm>
#include <iterator>

#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"
#include "utility/streams.hpp"

#include "utility/buildsys.hpp"
#include "utility/gccversion.hpp"
#include "utility/progress.hpp"
#include "utility/streams.hpp"
#include "utility/openmp.hpp"
#include "utility/progress.hpp"

#include "service/cmdline.hpp"

#include "geometry/meshop.hpp"

#include "../vts.hpp"
#include "../vts/io.hpp"
#include "../vts/csconvertor.hpp"
#include "../vts/meshop.hpp"
#include "../vts/opencv/atlas.hpp"

namespace po = boost::program_options;
namespace vs = vtslibs::storage;
namespace vr = vtslibs::registry;
namespace vts = vtslibs::vts;
namespace ba = boost::algorithm;
namespace fs = boost::filesystem;
namespace ublas = boost::numeric::ublas;

namespace {

class Repack : public service::Cmdline
{
public:
    Repack()
        : service::Cmdline("repack", BUILD_TARGET_VERSION)
    {
    }

private:
    virtual void configuration(po::options_description &cmdline
                               , po::options_description &config
                               , po::positional_options_description &pd)
        UTILITY_OVERRIDE;

    virtual void configure(const po::variables_map &vars)
        UTILITY_OVERRIDE;

    virtual bool help(std::ostream &out, const std::string &what) const
        UTILITY_OVERRIDE;

    virtual int run() UTILITY_OVERRIDE;

    fs::path output_;
    std::vector<fs::path> input_;
};

void Repack::configuration(po::options_description &cmdline
                             , po::options_description &config
                             , po::positional_options_description &pd)
{
    cmdline.add_options()
        ("output", po::value(&output_)->required()
         , "Path to output (vts) tile set.")
        ("input", po::value(&input_)->required()
         , "Path to input (vts) tile set.")
        ;

    pd.add("output", 1);
    pd.add("input", -1);

    (void) config;
}

void Repack::configure(const po::variables_map &vars)
{
    (void) vars;
}

bool Repack::help(std::ostream &out, const std::string &what) const
{
    if (what.empty()) {
        out << R"RAW(repalc
usage
    repack INPUT OUTPUT [OPTIONS]

)RAW";
    }
    return false;
}


// parse mtl file
typedef std::map<std::string, fs::path> Textures;

Textures loadMtlTextures(const fs::path &path)
{
    std::ifstream is;
    is.exceptions(std::ios::badbit);
    is.open(path.string());

    Textures textures;

    std::string material;

    std::string line;
    while (std::getline(is, line)) {
        if (ba::starts_with(line, "#")) { continue; }

        std::vector<std::string> tokens;
        ba::split(tokens, line, ba::is_any_of(" \t"), ba::token_compress_on);
        if (tokens.empty()) { continue; }

        if (tokens.front() == "newmtl") {
            material = tokens[1];
            continue;
        }

        if (tokens.front() == "map_Kd") {
            textures[material] = path.parent_path() / tokens[1];
        }
    }

    return textures;
}

class ObjLoader : public geometry::ObjParserBase {
public:
    ObjLoader(const fs::path &path, vts::Mesh &mesh
              , vts::opencv::HybridAtlas &atlas)
        : path_(path), mesh_(mesh), atlas_(atlas)
        , smOffset_(mesh.submeshes.size())
        , textureId_(), vMap_(), tcMap_()
    {
        useMaterial(0);
    }

private:
    typedef std::vector<int> VertexMap;
    typedef std::vector<VertexMap> VertexMaps;

    typedef std::map<std::string, unsigned int> Material2Submesh;

    virtual void addVertex(const Vector3d &v) {
        vertices_.emplace_back(v.x, v.y, v.z);
    }

    virtual void addTexture(const Vector3d &t) {
        tc_.emplace_back(t.x, t.y);
    }

    template <typename VertexType>
    void addFace(const int f[3], vts::Face &face
                 , const std::vector<VertexType> &vertices
                 , std::vector<VertexType> &out
                 , VertexMap &vmap)
    {
        for (int i(0); i < 3; ++i) {
            const std::size_t src(f[i]);
            // ensure space in map
            if (vmap.size() <= src) { vmap.resize(src + 1, -1); }

            auto &dst(vmap[src]);
            if (dst < 0) {
                // new mapping
                dst = out.size();
                out.push_back(vertices[src]);
            }
            face(i) = dst;
        }
    }

    virtual void addFacet(const Facet &f) {
        auto &sm(mesh_.submeshes[smOffset_ + textureId_]);
        sm.faces.emplace_back();
        addFace(f.v, sm.faces.back(), vertices_, sm.vertices, *vMap_);

        sm.facesTc.emplace_back();
        addFace(f.t, sm.facesTc.back(), tc_, sm.tc, *tcMap_);
    }

    virtual void useMaterial(const std::string &m) {
        auto fm2s(m2s.find(m));
        if (fm2s != m2s.end()) {
            useMaterial(fm2s->second);
            return;
        }

        auto ftextures(textures.find(m));
        if (ftextures == textures.end()) {
            LOGTHROW(err3, std::runtime_error)
                << "Cannot find texture for material <" << m << ">.";
        }

        atlas_.add(ftextures->second);

        // new material
        auto index(mesh_.submeshes.size());
        mesh_.submeshes.emplace_back();
        m2s.insert(Material2Submesh::value_type(m, index));

        // ignore for first (default) material
        if (m2s.size() > 1) { useMaterial(index); }
    }

    void useMaterial(unsigned int textureId) {
        textureId_ = textureId;

        auto index(textureId_ + smOffset_);

        // ensure space in all lists
        if (mesh_.submeshes.size() <= index) {
            mesh_.submeshes.resize(index + 1);
            vMaps_.resize(index + 1);
            tcMaps_.resize(index + 1);

            vMap_ = &vMaps_[index];
            tcMap_ = &tcMaps_[index];
        }
    }

    virtual void materialLibrary(const std::string &name) {
        const auto update(loadMtlTextures(path_ / name));
        textures.insert(update.begin(), update.end());
    }

    virtual void addNormal(const Vector3d&) { /*ignored*/ }

    fs::path path_;
    Textures textures;

    math::Points3 vertices_;
    math::Points2 tc_;
    VertexMaps vMaps_;
    VertexMaps tcMaps_;

    vts::Mesh &mesh_;
    vts::opencv::HybridAtlas &atlas_;

    Material2Submesh m2s;
    unsigned int smOffset_;
    unsigned int textureId_;

    VertexMap *vMap_;
    VertexMap *tcMap_;
};

int Repack::run()
{
    auto mesh(std::make_shared<vts::Mesh>());
    auto atlas(std::make_shared<vts::opencv::HybridAtlas>());

    for (const auto &input : input_) {
        LOG(info3) << "Loading input mesh " << input << ".";
        ObjLoader loader(fs::absolute(input).parent_path(), *mesh, *atlas);
        geometry::loadObj(loader, input);
    }

    LOG(info3) << "Merging meshes...";

    vts::SubmeshMergeOptions mo;
    // mo.maxVertexCount = 1 << 30;
    // mo.maxFaceCount = 1 << 30;

    vts::Mesh::pointer oMesh;
    vts::Atlas::pointer oAtlas;
    std::tie(oMesh, oAtlas)
        = vts::mergeSubmeshes({}, mesh, atlas, 95, mo);

    LOG(info3) << "Merging meshes... done.";

    // TODO: write output

    // all done
    LOG(info4) << "All done.";
    return EXIT_SUCCESS;
}

} // namespace

int main(int argc, char *argv[])
{
    return Repack()(argc, argv);
}
