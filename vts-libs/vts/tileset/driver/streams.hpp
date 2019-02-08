/**
 * Copyright (c) 2019 Melown Technologies SE
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

#ifndef vtslibs_vts_tileset_driver_streams_hpp_included_
#define vtslibs_vts_tileset_driver_streams_hpp_included_

#include "utility/expected.hpp"

#include "../../../storage/streams.hpp"

namespace vtslibs { namespace vts {

using storage::OStream;
using storage::IStream;
using storage::StringIStream;
using storage::File;
using storage::TileFile;
using storage::FileStat;
using storage::Resources;
using storage::NullWhenNotFound_t;
using storage::NullWhenNotFound;

/** Expected input stream. Contains a valid stream or an error.
 */
typedef utility::Expected<std::shared_ptr<IStream>> EIStream;
typedef std::function<void (const EIStream&)> InputCallback;

/** Expected file status. Contains a valid file status or an error.
 */
typedef utility::Expected<FileStat> EFileStat;
typedef std::function<void (const EFileStat&)> StatCallback;

} } // namespace vtslibs::vts

#endif // vtslibs_vts_tileset_driver_streams_hpp_included_
