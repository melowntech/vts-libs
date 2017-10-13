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

#ifndef vts_tools_assimp_hpp_included
#define vts_tools_assimp_hpp_included

#include <utility>

#include <boost/filesystem/path.hpp>

#include <assimp/Importer.hpp>

#include "roarchive/roarchive.hpp"

#include "../vts/mesh.hpp"

namespace vtslibs { namespace vts { namespace tools {

typedef std::vector<boost::filesystem::path> TexturePaths;
typedef std::vector<roarchive::IStream::pointer> TextureStreams;

std::tuple<Mesh, TexturePaths>
loadAssimpScene(Assimp::Importer &imp, const boost::filesystem::path &path
                , const math::Point3 &origin = math::Point3());

std::tuple<Mesh, TextureStreams>
loadAssimpScene(Assimp::Importer &imp, const roarchive::RoArchive &archive
                , const boost::filesystem::path &path
                , const math::Point3 &origin = math::Point3());

} } } // namespace vtslibs::vts::tools

#endif // vts_tools_assimp_hpp_included


