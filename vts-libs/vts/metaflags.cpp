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
#include "metaflags.hpp"

namespace vtslibs { namespace vts {

std::vector<MetaFlags::MetaFlag> MetaFlags::mapping = {
    MetaFlag(vts::MetaNode::Flag::geometryPresent, "geometry")
    , MetaFlag(vts::MetaNode::Flag::alien, "alien")
    , MetaFlag(vts::MetaNode::Flag::navtilePresent, "navtile")
    , MetaFlag(vts::MetaNode::Flag::applyTexelSize, "texelSize")
    , MetaFlag(vts::MetaNode::Flag::applyDisplaySize, "displaySize")
    , MetaFlag(vts::MetaNode::Flag::ulChild, "ul")
    , MetaFlag(vts::MetaNode::Flag::urChild, "ur")
    , MetaFlag(vts::MetaNode::Flag::llChild, "ll")
    , MetaFlag(vts::MetaNode::Flag::lrChild, "lr")
};

} } // namespace vtslibs::vts
