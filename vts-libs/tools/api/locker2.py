#!/usr/bin/python2

import sys
import socket
import fcntl
import os
import select
import time
import MySQLdb
import stat
from ConfigParser import ConfigParser
import hashlib

import dbglog as log

_lock_timeout = 60

def _makeNonblocking(fd):
    flags = fcntl.fcntl(fd, _fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)

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
        self._renewPeriod = _lock_timeout * 1000 / 2
        self._nextRenew = time.time() + self._renewPeriod

    def __call__(self):
        while (not self._handle(1000)):
            if (time.time() > self._nextRenew):
                self._impl.renew()
                self._nextRenew = time.time() + self._renewPeriod


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

class _MarioDBLocker:
    def __init__(self):
        self._locks = {}

        # load dbconf
        conf = ConfigParser()
        conf.read('locker.conf')

        self._dbconf = {
            'host' : conf.get('db','host'),
            'user' : conf.get('db','user'),
            'password' : conf.get('db','password'),
            'database' : conf.get('db','database')
        }

    def _connect_to_db(self) :
        db = MySQLdb.connect(host=self._dbconf['host'],
                             user=self._dbconf['user'],
                             passwd=self._dbconf['password'],
                             db=self._dbconf['database'],
                             charset='utf8')
        return db

    # concat storage dir inode and glue name
    def _get_lock_name(self, gluename):
        store_path = os.path.dirname(os.path.abspath(__file__))
        store_inode = os.stat(os.path.realpath(store_path))[stat.ST_INO]

        return (hashlib.md5(str(store_inode) + gluename).hexdigest()
             + hashlib.sha1(str(store_inode) + gluename).hexdigest())

    def lock(self, sublock):
        lock_name = self._get_lock_name(sublock)

        log.info3("Locking <{}>.", lock_name)
        if (self._locks.get(lock_name) is not None):
            raise ValueError, "Lock <%s> already locked." % (lock_name, )

        db = self._connect_to_db()
        cur = db.cursor()

        lock_query = "CALL lock_acquire(%s, %s, %s, %s)"

        cur.execute(lock_query, [ lock_name, _lock_timeout, None
                                , socket.gethostname()])

        token = cur.fetchall()[0][0]
        cur.close()
        db.commit()
        db.close()

        self._locks[lock_name] = token
        return token

    def renew(self):
        db = self._connect_to_db()
        cur = db.cursor()

        lock_query = "CALL lock_acquire(%s, %s, %s, %s)"
        host = socket.gethostname()

        for (lock, value) in self._locks.items():
            log.info3("Renewing lock <{}>.", lock)
            cur.execute(lock_query, [ lock, _lock_timeout, value, host ])

        cur.close()
        db.commit()
        db.close()

    def unlock(self, sublock, value):
        lock_name = self._get_lock_name(sublock)

        log.info3("Unlocking <{}> (value: <{}>).", lock_name, value)
        heldValue = self._locks.get(lock_name)
        if (heldValue is None):
            raise KeyError, "Lock <%s> not held." % (lock_name, )
        if (heldValue != value):
            raise ValueError, "Lock <%s> value mismatch." % (lock_name, )

        db = self._connect_to_db()
        cur = db.cursor()

        lock_query = "CALL lock_release(%s, %s)"

        cur.execute(lock_query, [lock_name, heldValue])

        res = cur.fetchall()[0][0]

        cur.close()
        db.commit()
        db.close()

        if res != 'true':
            raise Exception, "Cannot release lock <%s> in DB." % (lock_name, )

        del self._locks[lock_name]


if (__name__ == "__main__"):
    # Create simple locker and run it
    log.thread_id("locker2")
    log.info3("Locker2 starting.")
    Locker2(impl = _MarioDBLocker())()
    log.info3("Locker2 terminating.")
