#ifndef vadstena_libs_tilestorage_driver_fstreams_hpp_included_
#define vadstena_libs_tilestorage_driver_fstreams_hpp_included_

#include <fstream>
#include <functional>

#include <boost/filesystem/path.hpp>

#include "utility/streams.hpp"

#include "../streams.hpp"

namespace vadstena { namespace tilestorage {

class FileOStream : public OStream {
public:
    typedef std::function<void(bool)> OnClose;

    FileOStream(const boost::filesystem::path &path
                , OnClose onClose = OnClose())
        : path_(path), f_(), onClose_(onClose)
    {
        try {
            f_.exceptions(std::ios::badbit | std::ios::failbit);
            f_.open(path.string()
                    , std::ios_base::out | std::ios_base::trunc);
        } catch (const std::exception &e) {
            LOGTHROW(err1, std::runtime_error)
                << "Unable to open file " << path << " for writing.";
        }
    }

    virtual ~FileOStream() {
        if (!std::uncaught_exception() && f_.is_open()) {
            LOG(warn3) << "File was not closed!";
        }

        try {
            if (f_.is_open() && onClose_) {
                onClose_(false);
            }
        } catch (...) {}
    }

    virtual std::ostream& get() UTILITY_OVERRIDE { return f_; }

    virtual void close() UTILITY_OVERRIDE {
        // TODO: call onClose in case of failure (when exception is thrown)
        // via utility::ScopeGuard
        f_.close();
        if (onClose_) { onClose_(true); }
    }

    virtual std::string name() const UTILITY_OVERRIDE {
        return path_.string();
    };

    virtual FileStat stat() const UTILITY_OVERRIDE {
        return FileStat::stat(path_);
    }

private:
    boost::filesystem::path path_;
    utility::ofstreambuf f_;
    OnClose onClose_;
};

class FileIStream : public IStream {
public:
    FileIStream(const boost::filesystem::path &path)
        : path_(path), f_()
    {
        try {
            f_.exceptions(std::ios::badbit | std::ios::failbit);
            f_.open(path.string());
        } catch (const std::exception &e) {
            LOGTHROW(err1, std::runtime_error)
                << "Unable to open file " << path << " for reading.";
        }
    }

    virtual ~FileIStream() {
        if (!std::uncaught_exception() && f_.is_open()) {
            LOG(warn3) << "File was not closed!";
        }
    }

    virtual std::istream& get() UTILITY_OVERRIDE { return f_; }

    virtual void close() UTILITY_OVERRIDE { f_.close(); }

    virtual std::string name() const UTILITY_OVERRIDE {
        return path_.string();
    };

    virtual FileStat stat() const UTILITY_OVERRIDE {
        return FileStat::stat(path_);
    }

private:
    boost::filesystem::path path_;
    utility::ifstreambuf f_;
};

} } // namespace vadstena::tilestorage

#endif // vadstena_libs_tilestorage_driver_fstreams_hpp_included_
