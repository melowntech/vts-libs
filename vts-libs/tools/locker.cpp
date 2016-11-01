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

extern "C" {

static void vts_libs_tools_locker_sigchild_handler(int sig)
{
    // child died, terminate myself
    if (sig == SIGCHLD) {
        // this could be ok since we are using low-level IO in dbglog
        LOG(fatal)
            << "External lock program unexpectedly terminated. Terminaing.";
        ::raise(SIGTERM);
    }
}

} // extern "C"

namespace {

int runLocker(::pid_t ppid, int in[2], int out[2]
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

} // namespace

Lock::Lock(const fs::path &storage, const boost::optional<std::string> &lock)
{
    (void) storage;

    const auto lockerPath(storage / "locker");

    if (-1 == ::access(lockerPath.c_str(), X_OK)) {
        // lockerPath cannot be executed
        if (lock) {
            LOG(fatal) << "Called with held lock however there is no program "
                "to pass it to (" << lockerPath  << " is not an executable).";
            std::exit(EXIT_FAILURE);
        }

        return;
    }

    int in[2];
    if (-1 == ::pipe(in)) {
        std::system_error e(errno, std::system_category());
        LOG(err3) << "Cannot create a pipe: <"
                  << e.code() << ", " << e.what() << ">.";
        std::exit(EXIT_FAILURE);
    }

    int out[2];
    if (-1 == ::pipe(out)) {
        std::system_error e(errno, std::system_category());
        LOG(err3) << "Cannot create a pipe: <"
                  << e.code() << ", " << e.what() << ">.";
        std::exit(EXIT_FAILURE);
    }

    int pid(0);
    try {
        auto ppid(::getppid());
        pid = utility::spawn([=]() mutable -> int {
                return runLocker(ppid, in, out, storage, lockerPath, lock);
            });

        // close other ends of pipe
        ::close(out[0]);
        ::close(in[1]);
    } catch (const std::exception &e) {
        LOG(fatal) << "Locker execution failed: <" << e.what() << ">.";
        std::exit(EXIT_FAILURE);
    }

    // TODO: wait for info on pipe (with timeout)
    //       timed out -> kill child and terminate
    //       error -> terminate

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

    LOG(info3) << "Lock acquired by external lock program.";

    // locker runs, we want to terminate when child dies
    ::signal(SIGCHLD, &vts_libs_tools_locker_sigchild_handler);

    // up and running
}
