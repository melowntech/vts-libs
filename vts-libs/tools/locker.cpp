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
#include <signal.h>
#include <unistd.h>

#include <system_error>
#include <future>
#include <thread>
#include <queue>

#include <boost/asio.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/process.hpp"
#include "utility/steady-clock.hpp"
#include "utility/future.hpp"

#include "locker.hpp"

namespace asio = boost::asio;
namespace fs = boost::filesystem;
namespace bs = boost::system;
namespace ba = boost::algorithm;
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

void followLockerToGrave()
{
    // call handler when locker dies
    ::signal(SIGCHLD, &vts_libs_tools_locker_sigchild_handler);
}

void ignoreLockersDeath()
{
    // ignore locker's death
    ::signal(SIGCHLD, SIG_DFL);
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
    followLockerToGrave();
}

int runLocker2(const fs::path &storage
               , const fs::path &lockerPath
               , int lockerFd)
{
    // do not inherit log file
    dbglog::closeOnExec(true);

    // move to own process group so terminal controls do not kill this process
    // before it can do something useful
    ::setpgid(0, 0);

    // we need to clone lockerFd because redirects in exec close original
    auto lockerFdCopy(::dup(lockerFd));
    if (lockerFdCopy == -1) {
        std::system_error e(errno, std::system_category());
        LOG(err2)
            << "dup(" << lockerFd << ") failed: <" << e.code()
            << ", " << e.what() << ">";
        throw e;
    }

    utility::exec
        (
         // we have to absolutize path to locking program do wo chdir
         absolute(lockerPath).string()
         // bind stdin and stdout to locker fd
         , utility::Stdin(lockerFd)
         , utility::Stdout(lockerFdCopy)
         // change path to storage
         , utility::ChangeCwd(storage)
         );

    return EXIT_FAILURE;
}

struct LockTask {
    typedef std::shared_ptr<LockTask> pointer;
    typedef std::queue<pointer> queue;

    std::string command;
    std::promise<std::string> promise;
    bool componentLock;

    LockTask(const std::string &command, bool componentLock = false)
        : command(command), componentLock(componentLock)
    {}
};

class StorageLocker
    : public vtslibs::vts::StorageLocker
    , public std::enable_shared_from_this<StorageLocker>
{
public:
    StorageLocker()
        : lockerPid_(0), localSocket_(ios_), remoteSocket_(ios_)
        , work_(std::ref(ios_)), strand_(ios_), timer_(ios_)
        , inputBuffer_(1024)
    {
        // connect both sockets
        local::connect_pair(localSocket_, remoteSocket_);

        localSocket_.non_blocking(true);
        remoteSocket_.non_blocking(true);
    }

    ~StorageLocker() { stop(); }

    void start(const fs::path &lockerPath, const fs::path &storage);

private:
    virtual std::string lock_impl(const std::string &sublock);
    virtual void unlock_impl(const std::string &lock
                             , const std::string &sublock);

    void stop();

    void run();

    // next must be called under strand!
    void enqueue(const LockTask::pointer &task);
    // next must be called under strand!
    void next();

    void receive(const LockTask::pointer &task);
    void send(const LockTask::pointer &task);

    ::pid_t lockerPid_;
    asio::io_service ios_;
    local::stream_protocol::socket localSocket_;
    local::stream_protocol::socket remoteSocket_;

    boost::optional<asio::io_service::work> work_;
    std::thread worker_;
    asio::io_service::strand strand_;
    asio::deadline_timer timer_;
    asio::streambuf inputBuffer_;
    LockTask::queue tasks_;
};

void StorageLocker::start(const fs::path &lockerPath, const fs::path &storage)
{
    try {
        auto self(shared_from_this());
        ios_.notify_fork(asio::io_service::fork_prepare);
        lockerPid_ = utility::spawn
            ([this, self, storage, lockerPath]() mutable -> int
        {
            ios_.notify_fork(asio::io_service::fork_child);
            // close vts socket in subprocess (not needed there)
            localSocket_.close();
            // keep locker socket open and grab its fd
            int lockerFd(remoteSocket_.native_handle());
            return runLocker2(storage, lockerPath, lockerFd);
        });
        ios_.notify_fork(asio::io_service::fork_parent);
    } catch (const std::exception &e) {
        LOG(fatal) << "Locker execution failed: <" << e.what() << ">.";
        std::exit(EXIT_FAILURE);
    }

    // close remote socket (not needed here)
    remoteSocket_.close();

    std::thread worker(&StorageLocker::run, this);
    worker_.swap(worker);

    followLockerToGrave();
}

void StorageLocker::stop()
{
    ignoreLockersDeath();

    LOG(info2) << "Stopping.";
    work_ = boost::none;
    worker_.join();
    ios_.stop();
}

void StorageLocker::run()
{
    dbglog::thread_id("locker");
    LOG(info2) << "Spawned external locker worker.";

    for (;;) {
        try {
            ios_.run();
            LOG(info2) << "Terminated external locker worker.";
            return;
        } catch (const std::exception &e) {
            LOG(err3)
                << "Uncaught exception in external locker worker: <"
                << e.what() << ">. Going on.";
        }
    }
}

void StorageLocker::enqueue(const LockTask::pointer &task)
{
    // send immediately
    if (tasks_.empty()) {
        send(task);
        return;
    }
    tasks_.push(task);
}

void StorageLocker::next()
{
    if (tasks_.empty()) { return; }
    auto task(tasks_.front());
    tasks_.pop();
    send(task);
}

struct FatalLockerError : std::runtime_error{
    FatalLockerError(const std::string &msg) : std::runtime_error(msg) {}
};

void StorageLocker::receive(const LockTask::pointer &task)
{
    auto self(shared_from_this());
    auto processLine([self, this, task](const bs::error_code &ec, std::size_t)
    {
        try {
            if (ec) {
                LOGTHROW(err3, FatalLockerError)
                    << "Error communicating with locker2 process: >"
                    << ec << ">. Bailing out.";
            }

            // read line from input
            std::istream is(&inputBuffer_);
            is.unsetf(std::ios_base::skipws);
            std::string line;
            if (!std::getline(is, line, '\n')) {
                // fatal error
                LOGTHROW(err3, FatalLockerError)
                    << "Unable to get line from locker2 response"
                    ". Bailing out.";
            }

            // parse line
            LOG(info1) << "Received line from locker: <" << line << ">";

            if (line ==  "X") {
                if (task->componentLock) {
                    LOGTHROW(warn2, vtslibs::vts::StorageComponentLocked)
                        << "Component lock held by someone else: <"
                        << line << ">.";
                }
                LOGTHROW(warn2, vtslibs::vts::StorageLocked)
                    << "Lock held by someone else: <" << line << ">.";
            }

            if (line == "U") {
                task->promise.set_value(line);
                next();
                return;
            }

            if (ba::starts_with(line, "E:")) {
                LOGTHROW(err3, FatalLockerError)
                    << "Error communicating with locker2: <"
                    << line << ">.";
            }

            if (ba::starts_with(line, "L:")) {
                task->promise.set_value(line.substr(2));
                next();
                return;
            }

            LOGTHROW(err3, FatalLockerError)
                << "Error obtaining a lock from locker2: <"
                << line << ">.";
        } catch (...) {
            task->promise.set_exception(std::current_exception());
            next();
        }
    });

    asio::async_read_until(localSocket_, inputBuffer_, "\n"
                           , strand_.wrap(processLine));
}

void StorageLocker::send(const LockTask::pointer &task)
{
    auto self(shared_from_this());
    auto responseSent([self, this, task]
                      (const bs::error_code &ec, std::size_t)
    {
        try {
            if (ec) {
                LOGTHROW(err3, FatalLockerError)
                    << "Error communicating with locker2 process: >"
                    << ec << ">. Bailing out.";
            }

            // read response
            receive(task);
        } catch (...) {
            task->promise.set_exception(std::current_exception());
        }
    });

    asio::async_write(localSocket_
                      , asio::const_buffers_1(task->command.data()
                                              , task->command.size())
                      , strand_.wrap(responseSent));
}

std::string StorageLocker::lock_impl(const std::string &sublock)
{
    auto self(shared_from_this());
    LOG(info2) << "Locking <" << sublock << ">.";
    auto task(std::make_shared<LockTask>("L:" + sublock + "\n"
                                         , !sublock.empty()));
    ios_.post(strand_.wrap([this, self, task]() { enqueue(task); }));

    try {
        // wait for lock
        auto future(task->promise.get_future());
        if (utility::futureTimeout
            (future.wait_for(std::chrono::seconds(30))))
        {
            LOGTHROW(warn3, vtslibs::vts::LockTimedOut)
                << "Timed out while waiting to acquire lock for <"
                << sublock << ">.";
        }
        return future.get();
    } catch (const FatalLockerError&) {
        // bail out
        std::abort();
    }
    // never reached
    throw;
}

void StorageLocker::unlock_impl(const std::string &lock
                                , const std::string &sublock)
{
    LOG(info2) << "Unlocking <" << sublock << ">.";
    auto task(std::make_shared<LockTask>("U:" + sublock + ":" + lock + "\n"
                                         , !sublock.empty()));
    auto self(shared_from_this());
    ios_.post(strand_.wrap([this, self, task]() { enqueue(task); }));

    try {
        // wait for unlock
        auto future(task->promise.get_future());
        if (utility::futureTimeout
            (future.wait_for(std::chrono::seconds(30))))
        {
            LOGTHROW(warn3, vtslibs::vts::LockTimedOut)
                << "Timed out while waiting to acquire lock for <"
                << sublock << ">.";
        }
        future.get();
    } catch (const FatalLockerError&) {
        // bail out
        std::abort();
    }
    return;
}

vtslibs::vts::StorageLocker::pointer lock2(const fs::path &lockerPath
                                           , const fs::path &storage)
{
    auto locker(std::make_shared<StorageLocker>());
    locker->start(lockerPath, storage);

    LOG(info3) << "On-demand locking provided by external lock "
        "program (" << lockerPath << ").";

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
