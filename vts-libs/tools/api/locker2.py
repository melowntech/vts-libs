#!/usr/bin/python2

import sys
import socket
import fcntl
import os
import select
import time

import dbglog as log

_lock_timeout = 60

def now():
    return time.time()

class Locker2:
    def __init__(self, impl):
        self._impl = impl
        self._in = sys.stdin.fileno()
        self._out = sys.stdout.fileno()

        # grab connection file descriptor
        self._poll = select.poll()
        self._outBuff = ""
        self._inBuff = ""

        self._updatePoll()
        self._poll.register(self._in, select.EPOLLIN)

        # TODO: make configurable
        self._renewPeriod = _lock_timeout / 2.0
        self._nextRenew = now() + self._renewPeriod

    def __call__(self):
        while (not self._handle(1)):
            if (now() > self._nextRenew):
                self._impl.renew()
                self._nextRenew = now() + self._renewPeriod


    def _handle(self, timeout):
        events = self._poll.poll(timeout)
        if not len(events): return False

        for event in events:
            if (event[0]  not in (self._in, self._out)):
                log.warn2("Unexpected event on fd={}.", event[0])
                continue

            if self._handleClientEvent(event[1]):
                return True

        return False

    def _handleClientEvent(self, eventMask):
        if (eventMask & select.EPOLLHUP):
            return True
        if (eventMask & select.EPOLLIN):
            self._inBuff = self._inBuff + os.read(self._in, 1024)
            self._processInput()
        if (eventMask & select.EPOLLOUT):
            sent = os.write(self._out, self._outBuff)
            self._outBuff = self._outBuff[sent:]
            self._updatePoll();

        return False

    def _updatePoll(self):
        if (len(self._outBuff)):
            self._poll.register(self._out, select.EPOLLOUT)
        else:
            try:
                self._poll.unregister(self._out)
            except KeyError:
                pass

    def _processInput(self):
        if (not len(self._inBuff)): return

        commands = self._inBuff.split('\n')
        if (not len(commands[-1])):
            # last element is empty -> last command was complete
            self._inBuff = ""
        else:
            # last element non-empty -. last command was no complete
            # remember it in input buffer
            self._inBuff = commands[-1]

        # get rid of the last commands since it is either empty or incomplete
        del commands[-1]

        for command in commands:
            self._processCommand(command.split(":"))

    def _processCommand(self, command):
        if command[0] == "L":
            if (len(command) != 2):
                return self._invalidCommand()
            return self._lock(command[1])
        elif command[0] == "U":
            if (len(command) != 3):
                return self._invalidCommand()
            return self._unlock(command[1], command[2])
        else:
            return self._invalidCommand()

    def _lock(self, sublock):
        try:
            self._send("L:" + self._impl.lock(sublock))
        except ValueError:
            self._send("X")
        except Exception as e:
            self._error("Cannot acquire lock <%s>: <%s>." % (sublock, str(e)))

    def _unlock(self, sublock, value):
        try:
            self._impl.unlock(sublock, value)
            self._send("U")
        except Exception as e:
            self._error("Cannot release lock <%s>: <%s>." % (sublock, str(e)))

    def _invalidCommand(self):
        self._error("invalid command")

    def _error(self, what):
        log.err2("Sending error: <{}>.", what)
        self._send("E:%s" % (what, ))

    def ok(self, what):
        self._send("OK")

    def _send(self, what):
        self._outBuff = self._outBuff + what + '\n'
        self._updatePoll()

class _ExampleLocker:
    def __init__(self):
        self._locks = {}

    def lock(self, sublock):
        log.info3("Locking <{}>.", sublock)
        if (self._locks.get(sublock) is not None):
            raise ValueError, "Lock <%s> already locked." % (sublock, )

        value = "LOCK%s" % (sublock, )

        self._locks[sublock] = value
        return value

    def unlock(self, sublock, value):
        log.info3("Unlocking <{}> (value: <{}>).", sublock, value)
        heldValue = self._locks.get(sublock)
        if (heldValue is None):
            raise KeyError, "Lock <%s> not held." % (sublock, )
        if (heldValue != value):
            raise ValueError, "Lock <%s> value mismatch." % (sublock, )
        del self._locks[sublock]

    def renew(self):
        for (lock, value) in self._locks.items():
            log.info3("Renewing lock <{}>.", lock)

if (__name__ == "__main__"):
    # Create simple locker and run it
    log.thread_id("locker2")
    log.info3("Locker2 starting.")
    Locker2(impl = _ExampleLocker())()
    log.info3("Locker2 terminating.")
