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

#ifndef vtslibs_vts_config_hpp_included_
#define vtslibs_vts_config_hpp_included_

#include <boost/filesystem/path.hpp>

namespace vtslibs { namespace vts {

/** Simple config file guard.
 *
 * * ctor create temporary file which is accessible by caller to write
 * * commit move temporary file over real file
 * * rollback removes temporary file
 * * dtor:
 *   * manual rollback/commit was performed -> nothing happens
 *   * otherwise:
 *     * pending exception: rollback
 *     * no pending exceptio: commit
 *
 * NB: Dtor can throw if commit throws. This happens only when there is no
 * pending exception. Instances are expected to be tmp object on stack.
 *
 * Temporary file is not meant as a security measure. Just to make sure data are
 * written to unique file and placed at proper place only when everything is OK.
 */
class ConfigFileGuard {
public:
    ConfigFileGuard(const boost::filesystem::path &path);
    ~ConfigFileGuard() noexcept(false);

    void commit();
    void rollback();

    /** Path of file to write to.
     */
    const boost::filesystem::path path() const { return tmpPath_; }

private:
    const boost::filesystem::path path_;
    boost::filesystem::path tmpPath_;
};

} } // namespace vtslibs::vts

#endif // vtslibs_vts_config_hpp_included_
