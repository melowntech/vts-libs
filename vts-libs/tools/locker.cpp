#include <signal.h>
#include <unistd.h>

#include <system_error>

#include <boost/asio.hpp>
#include <boost/filesystem.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/process.hpp"
#include "utility/steady-clock.hpp"

#include "./locker.hpp"

namespace asio = boost::asio;
namespace fs = boost::filesystem;
namespace bs = boost::system;
namespace local = boost::asio::local;

using boost::asio::local::stream_protocol;

extern "C" {

static void vts_libs_tools_locker_sigchild_handler(int sig)
{
    // child died, terminate myself
    if (sig == SIGCHLD) {
        // this could be ok since we are using low-level IO in dbglog
        LOG(fatal)
            << "External lock program unexpectedly terminated. Terminating.";
        ::raise(SIGTERM);
    }
}

} // extern "C"

namespace {

void dieIfLockerDies()
{
    // locker runs, we want to terminate when child dies
    ::signal(SIGCHLD, &vts_libs_tools_locker_sigchild_handler);
}

int runLocker1(::pid_t ppid, int in[2], int out[2]
               , const fs::path &storage
               , const fs::path &lockerPath
               , const boost::optional<std::string> &lock)
{
    (void) ppid;

    // close other ends of pipes
    ::close(out[1]);
    ::close(in[0]);

    // move to own process group so terminal controls do not kill this process
    // before it can do something useful
    ::setpgid(0, 0);

    // we have to absolutize path to locking program do wo chdir
    utility::exec(absolute(lockerPath).string()
                  , (lock ? *lock : std::string())
                  , utility::Stdin(out[0])
                  , utility::Stdout(in[1])
                  , utility::ChangeCwd(storage)
                  );

    return EXIT_FAILURE;
}

void lock1(const fs::path &lockerPath
           , const fs::path &storage
           , const boost::optional<std::string> &lock)
{
    int in[2];
    if (-1 == ::pipe(in)) {
        std::system_error e(errno, std::system_category());
        LOG(fatal) << "Cannot create a pipe: <"
                   << e.code() << ", " << e.what() << ">.";
        std::exit(EXIT_FAILURE);
    }

    int out[2];
    if (-1 == ::pipe(out)) {
        std::system_error e(errno, std::system_category());
        LOG(fatal) << "Cannot create a pipe: <"
                   << e.code() << ", " << e.what() << ">.";
        std::exit(EXIT_FAILURE);
    }

    int pid(0);
    try {
        auto ppid(::getppid());
        pid = utility::spawn([=]() mutable -> int {
                return runLocker1(ppid, in, out, storage, lockerPath, lock);
            });

        // close other ends of pipe
        ::close(out[0]);
        ::close(in[1]);
    } catch (const std::exception &e) {
        LOG(fatal) << "Locker execution failed: <" << e.what() << ">.";
        std::exit(EXIT_FAILURE);
    }

    enum Result { ok, noLock, error };
    Result result(Result::ok);

    asio::io_service ios;
    asio::posix::stream_descriptor fromLock(ios, in[0]);

    // TODO: make configurable
    asio::deadline_timer timer(ios);
    timer.expires_from_now(boost::posix_time::seconds(5));

    timer.async_wait([&](const bs::error_code &ec)
    {
        if (!ec) {
            LOG(fatal) << "Timed out while waiting for lock acquisition.";
            result = Result::error;
            fromLock.close();
        } else if (ec != asio::error::operation_aborted) {
            LOG(fatal) << "Error while waiting for lock acquisition.";
        }
    });

    asio::streambuf sb(1024);
    asio::async_read_until(fromLock, sb, "\n"
                           , [&](const bs::error_code &ec
                                 , std::size_t bytes)
    {
        // cancel timer
        timer.cancel();

        if (ec) {
            if (ec != asio::error::operation_aborted) {
                LOG(err3) << "Error reading from lock program.";
                result = Result::error;
            }
            return;
        }

        if (!bytes) {
            result = Result::error;
            LOG(err3) << "Nothing read from lock program.";
            return;
        }

        std::istream is(&sb);
        std::string line;
        std::getline(is, line);

        if (line == "OK") {
            // success
            result = Result::ok;
            return;
        }

        // error
        LOG(err3)
            << "Error while acquiring lock by external lock program: <"
            << line << ">.";
        result = Result::noLock;
    });

    // let the machinery run
    ios.run();

    switch (result) {
    case Result::ok: break;

    case Result::error:
        // error, terminate lock program and go on
        ::kill(pid, SIGTERM);
        std::exit(EXIT_FAILURE);
        break;

    case Result::noLock:
        std::exit(EXIT_FAILURE);
        break;
    }

    LOG(info3) << "Lock acquired by external lock program ("
               << lockerPath << ").";

    // up and running
    dieIfLockerDies();
}

int runLocker2(int out[2], const fs::path &storage
               , const fs::path &lockerPath
               , int lockerFd)
{
    // move to own process group so terminal controls do not kill this process
    // before it can do something useful
    ::setpgid(0, 0);

    // we have to absolutize path to locking program do wo chdir
    utility::exec(absolute(lockerPath).string()
                  , lockerFd
                  , utility::Stdin(out[0])
                  , utility::ChangeCwd(storage)
                  );

    return EXIT_FAILURE;
}

class StorageLocker : public vtslibs::vts::StorageLocker {
public:
    StorageLocker()
        : lockerPid_(0), vtsSocket_(ios_), lockerSocket_(ios_)
    {
    }

    void run(const fs::path &lockerPath, const fs::path &storage) {
        // connect both sockets
        local::connect_pair(vtsSocket_, lockerSocket_);

        // output pipe to locker process
        int out[2];
        if (-1 == ::pipe(out)) {
            std::system_error e(errno, std::system_category());
            LOG(fatal) << "Cannot create a pipe: <"
                       << e.code() << ", " << e.what() << ">.";
            std::exit(EXIT_FAILURE);
        }

        try {
            int lockerFd(lockerSocket_.native_handle());
            lockerPid_ = utility::spawn([=]() mutable -> int
            {
                return runLocker2(out, storage, lockerPath, lockerFd);
            });
        } catch (const std::exception &e) {
            LOG(fatal) << "Locker execution failed: <" << e.what() << ">.";
            std::exit(EXIT_FAILURE);
        }

        // close other end of pipe
        ::close(out[0]);
    }

private:
    virtual void lock_impl(const std::string &sublock) {
        LOG(info4) << "Locking <" << sublock << ">.";
        (void) sublock;
    }

    virtual void unlock_impl(const std::string &sublock) {
        LOG(info4) << "Unlocking <" << sublock << ">.";
        (void) sublock;
    }

    ::pid_t lockerPid_;
    asio::io_service ios_;
    local::stream_protocol::socket vtsSocket_;
    local::stream_protocol::socket lockerSocket_;
};

vtslibs::vts::StorageLocker::pointer lock2(const fs::path &lockerPath
                                           , const fs::path &storage)
{
    auto locker(std::make_shared<StorageLocker>());
    locker->run(lockerPath, storage);

    LOG(info3) << "On-demand locking provided by external lock "
        "program (" << lockerPath << ").";

    dieIfLockerDies();
    return locker;
}

} // namespace

Lock::Lock(const fs::path &storage, const boost::optional<std::string> &lock)
{
    const auto lockerPath1(storage / "locker");
    const auto lockerPath2(storage / "locker2");

    if (!lock) {
        // try locker v2
        if (0 == ::access(lockerPath2.c_str(), X_OK)) {
            storageLocker_ = lock2(lockerPath2, storage);
            return;
        }
    }

    // try locker v1
    if (-1 == ::access(lockerPath1.c_str(), X_OK)) {
        // lockerPath1 cannot be executed
        if (lock) {
            LOG(fatal) << "Called with held lock however there is no program "
                "to pass it to (" << lockerPath1  << " is not an executable).";
            std::exit(EXIT_FAILURE);
        }

        return;
    }

    lock1(lockerPath1, storage, lock);
}
