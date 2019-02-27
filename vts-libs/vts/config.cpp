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

#include <sys/types.h>
#include <unistd.h>

#include <exception>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/path.hpp"

#include "config.hpp"

namespace fs = boost::filesystem;

namespace vtslibs { namespace vts {

namespace {
fs::path buildTmpPath(const fs::path &path)
{
    char hostname[256];
    ::gethostname(hostname, sizeof(hostname));
    hostname[sizeof(hostname) - 1] = '\0';

    // temporary file extenstion
    const auto ext(str(boost::format(".tmp-%s-%d")
                       % ::getpid() % hostname));
    return utility::addExtension(path, ext);
}

} // namespace

ConfigFileGuard::ConfigFileGuard(const fs::path &path)
    : path_(path), tmpPath_(buildTmpPath(path))
{
    LOG(info1) << "Config " << path_ << " temporary file name is "
               << tmpPath_ << ".";
}

ConfigFileGuard::~ConfigFileGuard() noexcept(false)
{
    if (std::uncaught_exception()) {
        try { rollback(); } catch (...) {}
    } else {
        commit();
    }
}

void ConfigFileGuard::commit()
{
    if (tmpPath_.empty()) { return; }
    fs::rename(tmpPath_, path_);
    tmpPath_.clear();
}

void ConfigFileGuard::rollback()
{
    if (tmpPath_.empty()) { return; }
    fs::remove(tmpPath_);
    tmpPath_.clear();
}

} } // namespace vtslibs::vts
