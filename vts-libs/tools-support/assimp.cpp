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

#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "dbglog/dbglog.hpp"

#include "../storage/error.hpp"
#include "../vts/meshop.hpp"

#include "assimp.hpp"

namespace fs = boost::filesystem;

namespace vtslibs { namespace vts { namespace tools {

namespace {

const aiScene* readScene(Assimp::Importer &imp
                         , const fs::path &path
                         , unsigned int flags)
{
    const auto *scene(imp.ReadFile(path.string(), flags));

    if (!scene) {
        LOGTHROW(err2, storage::Error)
            << "Error loading scene " << path
            << ": <" << imp.GetErrorString() << ">.";
    }

    return scene;
}

inline fs::path textureFile(const aiScene *scene, const aiMesh *mesh
                            , int channel)
{
    const auto *mat(scene->mMaterials[mesh->mMaterialIndex]);
    aiString texFile;
    mat->Get(AI_MATKEY_TEXTURE_DIFFUSE(channel), texFile);
    return { texFile.C_Str() };
}

inline math::Point3 point3(const aiVector3D &vec)
{
    return { vec.x, vec.y, vec.z };
}

void asMesh(vts::Mesh &mesh, const aiScene *scene
            , const math::Point3 &origin)
{
    for (unsigned m = 0; m < scene->mNumMeshes; ++m) {
        vts::SubMesh sm;

        const auto aimesh(scene->mMeshes[m]);
        const bool hasUv(aimesh->GetNumUVChannels());

        for (unsigned i = 0; i < aimesh->mNumVertices; ++i) {
            sm.vertices.push_back(origin + point3(aimesh->mVertices[i]));

            if (hasUv) {
                const aiVector3D &tc(aimesh->mTextureCoords[0][i]);
                sm.tc.emplace_back(tc.x, tc.y);
            }
        }

        for (unsigned i = 0; i < aimesh->mNumFaces; ++i) {
            assert(aimesh->mFaces[i].mNumIndices == 3);
            unsigned int* idx(aimesh->mFaces[i].mIndices);
            sm.faces.emplace_back(idx[0], idx[1], idx[2]);
            sm.facesTc.emplace_back(idx[0], idx[1], idx[2]);
        }

        // remove duplicate vertices introduced by AssImp and add to mesh
        mesh.add(vts::optimize(sm));
    }
}

TexturePaths asTextures(const fs::path &scenePath, const aiScene *scene)
{
    TexturePaths textures;
    for (unsigned m = 0; m < scene->mNumMeshes; ++m) {
        const auto aimesh(scene->mMeshes[m]);
        textures.push_back
            (scenePath.parent_path() / textureFile(scene, aimesh, 0));
    }
    return textures;
}

TextureStreams asTextures(const roarchive::RoArchive &archive
                          , const fs::path &scenePath, const aiScene *scene)
{
    TextureStreams textures;
    for (unsigned m = 0; m < scene->mNumMeshes; ++m) {
        const auto aimesh(scene->mMeshes[m]);
        textures.push_back
            (archive.istream
             (scenePath.parent_path() / textureFile(scene, aimesh, 0)));
    }
    return textures;
}

} // namespace

std::tuple<Mesh, TexturePaths>
loadAssimpScene(Assimp::Importer &imp, const boost::filesystem::path &path
                , const math::Point3 &origin)
{
    const auto *scene(readScene(imp, path, aiProcess_Triangulate));
    if (!scene) {
        LOGTHROW(err3, std::runtime_error)
                << "Error loading scene " << path
                << "( " << imp.GetErrorString() << " ).";
    }

    std::tuple<Mesh, TexturePaths> result;

    asMesh(std::get<0>(result), scene, origin);
    std::get<1>(result) = asTextures(path, scene);

    return result;
}

std::tuple<Mesh, TextureStreams>
loadAssimpScene(Assimp::Importer &imp, const roarchive::RoArchive &archive
                , const boost::filesystem::path &path
                , const math::Point3 &origin)
{
    const auto buf(archive.istream(path)->read());

    const auto *scene
        (imp.ReadFileFromMemory(buf.data(), buf.size()
                                , aiProcess_Triangulate));
    if (!scene) {
        LOGTHROW(err3, std::runtime_error)
                << "Error loading scene " << path
                << "( " << imp.GetErrorString() << " ).";
    }

    std::tuple<Mesh, TextureStreams> result;

    asMesh(std::get<0>(result), scene, origin);
    std::get<1>(result) = asTextures(archive, path, scene);

    return result;
}

} } } // namespace vtslibs::vts::tools


