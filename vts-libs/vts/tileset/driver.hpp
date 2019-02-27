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
#ifndef vtslibs_vts_tileset_driver_hpp_included_
#define vtslibs_vts_tileset_driver_hpp_included_

#include <set>
#include <map>
#include <memory>

#include <boost/noncopyable.hpp>

#include "utility/runnable.hpp"
#include "utility/expected.hpp"

#include "../../storage/streams.hpp"
#include "../../storage/resources.hpp"

#include "../../vts.hpp"
#include "../options.hpp"
#include "driver/options.hpp"
#include "driver/streams.hpp"
#include "tilesetindex.hpp"

namespace vtslibs { namespace vts {

class Driver : boost::noncopyable {
public:
    /** Driver capabilities.
     */
    struct Capabilities {
        /** Driver flattens existing multisurface setup into single surface.
         */
        bool flattener;

        /** Driver preferes asychronous operations when set.
         */
        bool async;

        Capabilities() : flattener(false), async(false) {}
    };

    typedef std::shared_ptr<Driver> pointer;
    typedef std::vector<pointer> list;

    virtual ~Driver();

    /** Creates driver for new dataset.
     */
    static pointer create(const boost::filesystem::path &root
                          , const boost::any &options
                          , const CloneOptions &cloneOptions);

    /** Creates in-memory driver.
     *  Only some drivers provide this feature.
     */
    static pointer create(const boost::any &options
                          , const CloneOptions &cloneOptions);

    /** Opens driver for existing dataset.
     */
    static pointer open(const boost::filesystem::path &root
                        , const OpenOptions &openOptions = OpenOptions());

    /** Opens driver for existing dataset, with driver options available.
     */
    static pointer open(const boost::filesystem::path &root
                        , const boost::any &options
                        , const OpenOptions &openOptions = OpenOptions());

    struct BareConfigTag {};

    /** Opens driver for existing dataset, mark config as bare (just driver
     *  options)
     */
    static pointer open(const boost::filesystem::path &root
                        , const BareConfigTag&
                        , const OpenOptions &openOptions = OpenOptions());

    /** Opens config-reading driver for existing dataset.
     *  NP: May return full driver for some tileset types.
     */
    static pointer configReader(const boost::filesystem::path &root
                                , const OpenOptions &openOptions
                                = OpenOptions());

    /** Opens config-reading driver for existing dataset.
     *  NP: May return full driver for some tileset types.
     */
    static pointer configReader(const boost::filesystem::path &root
                                , const boost::any &options
                                , const OpenOptions &openOptions
                                = OpenOptions());

    /** Async driver open. Uses provided callback to report success/error and to
     *  obtain dependencies (e.g. storage and other drivers for aggregated
     *  driver)
     */
    static void open(const boost::filesystem::path &root
                     , const boost::any &genericOptions
                     , const OpenOptions &openOptions
                     , const DriverOpenCallback::pointer &callback);

    /** Check for valid configuration.
     */
    static bool check(const boost::filesystem::path &root);

    static bool check(const boost::filesystem::path &root
                      , const std::string &mime);

    /** Creates driver for new dataset as exact copy of the original one.
     */
    pointer clone(const boost::filesystem::path &root
                  , const CloneOptions &cloneOptions) const;

    OStream::pointer output(File type);

    IStream::pointer input(File type) const;

    IStream::pointer input(File type, const NullWhenNotFound_t&) const;

    OStream::pointer output(const TileId &tileId, TileFile type);

    IStream::pointer input(const TileId &tileId, TileFile type) const;

    IStream::pointer input(const TileId &tileId, TileFile type
                           , const NullWhenNotFound_t&) const;

    /** Same as input(tileId, type) but fetches file asynchronously. Calls
     *  callback when stream is ready.
     *
     * N.B.: notFound is passed as a raw pointer. It must be valid until the
     * asynchronous opetation uses it. Must be either global or can be a member
     * of the callback.
     *
     * \param tileId tile ID
     * \param type tile file type
     * \param cb callback called when input stream is ready
     * \param notFound value passed to callback when given file is not found
     */
    void input(const TileId &tileId, TileFile type, const InputCallback &cb
               , const IStream::pointer *notFound = nullptr) const;

    FileStat stat(File type) const;

    FileStat stat(const TileId &tileId, TileFile type) const;

    void stat(const TileId &tileId, TileFile type
              , const StatCallback &cb) const;

    /** Extra files provided by driver.
     */
    IStream::pointer input(const std::string &name) const;

    /** Extra files provided by driver.
     */
    IStream::pointer input(const std::string &name
                           , const NullWhenNotFound_t&) const;

    /** Extra files provided by driver.
     */
    FileStat stat(const std::string &name) const;

    Resources resources() const;

    void flush();

    bool externallyChanged() const;

    const boost::any& options() const { return options_; }

    template <typename T>
    const T& options() const { return boost::any_cast<const T&>(options_); }

    template <typename T>
    static const T& options(const boost::any &options) {
        return boost::any_cast<const T&>(options);
    }

    void wannaWrite(const std::string &what) const;

    /** Drop storage.
     */
    void drop();

    /** Sets runnable that is observed during access operations.
     *  If stopped runnable is encountered operation throws Interrupted.
     *  Pass nullptr to stop watching runnable.
     */
    void watch(utility::Runnable *runnable);

    /** Gets old revision (if any)
     */
    boost::optional<unsigned int> oldRevision() const {
        return oldRevision_;
    }

    /** Returns old revision for provided root path.
     */
    static boost::optional<unsigned int>
    oldRevision(const boost::filesystem::path &root);

    const boost::filesystem::path& root() const { return root_; }

    /** Returns time of last modification. Recorded at read-only open.
     */
    std::time_t lastModified() const { return lastModified_; }

    bool readOnly() const;

    /** Information.
     */
    std::string info() const;

    /** Returns parsed read-only tileindex. Some drives need tileindex for their
     *  operation and it whould be a waste of resources to load it again from
     *  disk. Also, some drivers may hold tileindex only in-memory and to get
     *  them as a file we'd have to serialize it first.
     */
    tileset::Index* getTileIndex();

    /** Returns parsed read-only tileindex. Some drives need tileindex for their
     *  operation and it whould be a waste of resources to load it again from
     *  disk. Also, some drivers may hold tileindex only in-memory and to get
     *  them as a file we'd have to serialize it first.
     */
    const tileset::Index* getTileIndex() const;

    /** Returns drivers capabilities.
     */
    inline const Capabilities& capabilities() const { return capabilities_; }
    inline const Capabilities& ccapabilities() const { return capabilities_; }

    /** Relocates referenced resources.
     */
    static void relocate(const boost::filesystem::path &root
                         , const RelocateOptions &relocateOptions
                         , const std::string &prefix = "");

    /** Recursive tileset reencode.
     */
    static void reencode(const boost::filesystem::path &root
                         , const ReencodeOptions &options
                         , const std::string &prefix = "");

    inline const OpenOptions& openOptions() const { return openOptions_; }

protected:
    /** Creates new storage. Existing storage is overwritten only if mode ==
     *  CreateMode::overwrite.
     */
    Driver(const boost::filesystem::path &root
           , const OpenOptions &openOptions
           , const boost::any &options
           , CreateMode mode);

    /** Creates in-memory storage.
     */
    Driver(const OpenOptions &openOptions, const boost::any &options
           , CreateMode mode);

    /** Opens storage.
     */
    Driver(const boost::filesystem::path &root, const OpenOptions &openOptions
           , const boost::any &options);

    void readOnly(bool value);

    inline const FileStat& configStat() const { return configStat_; }

    /** Allow driver to change its capabilities.
     */
    inline Capabilities& capabilities() { return capabilities_; }

private:
    virtual OStream::pointer output_impl(const File type) = 0;

    virtual IStream::pointer input_impl(File type) const = 0;

    virtual IStream::pointer input_impl(File type, const NullWhenNotFound_t&)
        const = 0;

    virtual OStream::pointer
    output_impl(const TileId &tileId, TileFile type) = 0;

    virtual IStream::pointer
    input_impl(const TileId &tileId, TileFile type) const = 0;

    virtual IStream::pointer
    input_impl(const TileId &tileId, TileFile type
               , const NullWhenNotFound_t&) const = 0;

    /** Default version calls cb(input_impl(tileId, type)) immediately. Override
     *  only when needed.
     */
    virtual void input_impl(const TileId &tileId, TileFile type
                            , const InputCallback &cb
                            , const IStream::pointer *notFound) const;

    virtual void drop_impl() = 0;

    virtual void flush_impl() = 0;

    virtual FileStat stat_impl(File type) const = 0;

    virtual FileStat stat_impl(const TileId &tileId, TileFile type)
        const = 0;

    virtual void stat_impl(const TileId &tileId, TileFile type
                           , const StatCallback &cb) const;

    /** Extra files provided by driver. Optional.
     */
    virtual IStream::pointer input_impl(const std::string &name) const;

    /** Extra files provided by driver. Optional.
     */
    virtual IStream::pointer input_impl(const std::string &name
                                   , const NullWhenNotFound_t&) const;

    /** Extra files provided by driver. Optional.
     */
    virtual FileStat stat_impl(const std::string &name) const;

    virtual Resources resources_impl() const = 0;

    virtual pointer clone_impl(const boost::filesystem::path &root
                               , const CloneOptions &cloneOptions) const = 0;

    virtual std::string info_impl() const = 0;

    virtual tileset::Index* getTileIndex_impl() { return nullptr; }

    virtual const tileset::Index* getTileIndex_impl() const { return nullptr; }

    void checkRunning() const;

    void notRunning() const;

    /** Backing root.
     */
    const boost::filesystem::path root_;

    bool readOnly_;

    /** Driver capabilities.
     */
    Capabilities capabilities_;

    /** Path to config
     */
    boost::filesystem::path configPath_;

    /** Path to extra-config
     */
    boost::filesystem::path extraConfigPath_;

    /** Path to registry
     */
    boost::filesystem::path registryPath_;

    /** Open options.
     */
    const OpenOptions openOptions_;

    /** Driver options.
     */
    boost::any options_;

    /** Information about root when tileset was open in read-only mode.
     */
    FileStat rootStat_;

    /** Information about config when tileset was open in read-only mode.
     */
    FileStat configStat_;

    /** Information about extra-config when tileset was open in read-only mode.
     */
    FileStat extraConfigStat_;

    /** Information about registry when tileset was open in read-only mode.
     */
    FileStat registryStat_;

    /** Runnable associated with the driver.
     */
    utility::Runnable *runnable_;

    /** Revision read from old config (if any).
     */
    boost::optional<unsigned int> oldRevision_;

    /** Time of last modification (recorded at read-only open)
     */
    std::time_t lastModified_;
};

inline void Driver::watch(utility::Runnable *runnable)
{
    runnable_ = runnable;
}

inline void Driver::checkRunning() const
{
    if (!runnable_ || *runnable_) { return; }
    notRunning();
}

inline OStream::pointer Driver::output(File type)
{
    checkRunning();
    return output_impl(type);
}

inline IStream::pointer Driver::input(File type) const
{
    checkRunning();
    return input_impl(type);
}

inline IStream::pointer Driver::input(File type, const NullWhenNotFound_t&)
    const
{
    checkRunning();
    return input_impl(type, NullWhenNotFound);
}

inline OStream::pointer Driver::output(const TileId &tileId, TileFile type)
{
    checkRunning();
    return output_impl(tileId, type);
}

inline IStream::pointer Driver::input(const TileId &tileId, TileFile type)
    const
{
    checkRunning();
    return input_impl(tileId, type);
}

inline IStream::pointer Driver::input(const TileId &tileId, TileFile type
                                      , const NullWhenNotFound_t&)
    const
{
    checkRunning();
    return input_impl(tileId, type, NullWhenNotFound);
}

inline void Driver::input(const TileId &tileId, TileFile type
                          , const InputCallback &cb
                          , const IStream::pointer *notFound) const
{
    checkRunning();
    return input_impl(tileId, type, cb, notFound);
}

inline FileStat Driver::stat(File type) const
{
    checkRunning();
    return stat_impl(type);
}

inline FileStat Driver::stat(const TileId &tileId, TileFile type) const
{
    checkRunning();
    return stat_impl(tileId, type);
}

inline void Driver::stat(const TileId &tileId, TileFile type
                         , const StatCallback &cb) const
{
    checkRunning();
    return stat_impl(tileId, type, cb);
}

inline storage::Resources Driver::resources() const
{
    return resources_impl();
}

inline void Driver::drop()
{
    checkRunning();
    return drop_impl();
}

inline void Driver::flush()
{
    // flush and make read-only
    flush_impl();
    readOnly(true);
}

inline Driver::pointer Driver::clone(const boost::filesystem::path &root
                                     , const CloneOptions &cloneOptions)
    const
{
    return clone_impl(root, cloneOptions);
}

inline std::string Driver::info() const
{
    return info_impl();
}

inline tileset::Index* Driver::getTileIndex()
{
    return getTileIndex_impl();
}

inline const tileset::Index* Driver::getTileIndex() const
{
    return getTileIndex_impl();
}

inline IStream::pointer Driver::input(const std::string &name) const
{
    checkRunning();
    return input_impl(name);
}

inline IStream::pointer Driver::input(const std::string &name
                                      , const NullWhenNotFound_t&)
    const
{
    checkRunning();
    return input_impl(name, NullWhenNotFound);
}

inline FileStat Driver::stat(const std::string &name) const
{
    checkRunning();
    return stat_impl(name);
}

} } // namespace vtslibs::vts

#endif // vtslibs_vts_tileset_driver_hpp_included_
