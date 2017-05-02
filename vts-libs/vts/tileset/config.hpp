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
/**
 * \file vts/config.hpp
 * \author Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef vtslibs_vts_tileset_config_hpp_included_
#define vtslibs_vts_tileset_config_hpp_included_

#include <iostream>

#include <boost/any.hpp>
#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

#include "../tileset.hpp"

namespace vtslibs { namespace vts { namespace tileset {

FullTileSetProperties loadConfig(std::istream &in
                                 , const boost::filesystem::path &path
                                 = "unknown");

void saveConfig(std::ostream &out, const FullTileSetProperties &properties);

FullTileSetProperties loadConfig(const boost::filesystem::path &path);

void saveConfig(const boost::filesystem::path &path
                , const FullTileSetProperties &properties);

ExtraTileSetProperties loadExtraConfig(std::istream &in
                                       , const boost::filesystem::path &path
                                       = "UNKNOWN");

ExtraTileSetProperties loadExtraConfig(const boost::filesystem::path &path);

boost::optional<unsigned int> loadRevision(std::istream &in);

boost::optional<unsigned int>
loadRevision(const boost::filesystem::path &path);

FullTileSetProperties loadConfig(const Driver &driver);

FullTileSetProperties loadConfig(const IStream::pointer &file);

// bare driver support

void saveDriver(std::ostream &out, const boost::any &driver);

void saveDriver(const boost::filesystem::path &path
                , const boost::any &driver);

boost::any loadDriver(std::istream &in, const boost::filesystem::path &path
                      = "unknown");

boost::any loadDriver(const boost::filesystem::path &path);


} } } // namespace vtslibs::vts::tileset

#endif // vtslibs_vts_tileset_config_hpp_included_
