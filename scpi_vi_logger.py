#!/usr/bin/env python3
"""
Long-run voltage/current logger for Rohde & Schwarz instruments (PyVISA).

Built to survive unattended multi-week runs: it reconnects on its own, detects
the VISA desync that silently corrupts readings after a timeout, appends rather
than truncates so a restart never destroys data, and fsyncs every row so a power
cut on the PC costs you at most one sample.

Install:
    pip install pyvisa pyvisa-py

Self-test with no hardware (recommended before any long run):
    python scpi_vi_logger.py --simulate --interval 1 --hours 0.02 -c 1 2

Real use:
    python scpi_vi_logger.py --list
    python scpi_vi_logger.py -r TCPIP0::10.68.63.118::5025::SOCKET -c 1 2

Every --git-interval seconds (default 300 = 5 min) the CSV is committed and
pushed to the 'origin' remote of the git repo containing it, so a remote
viewer can tell the run is alive from the commit timestamps. Disable with
--no-git-push.

Resource strings:
    TCPIP0::192.168.1.50::inst0::INSTR      VXI-11   (default, most portable)
    TCPIP0::192.168.1.50::hislip0::INSTR    HiSLIP   (faster, newer firmware)
    TCPIP0::192.168.1.50::5025::SOCKET      raw socket
    USB0::0x0AAD::0x0197::1234.5678k02-123456-AB::INSTR

Run it detached so an SSH drop can't kill it:
    nohup python -u scpi_vi_logger.py -r ... > logger.out 2>&1 &
"""

import argparse
import csv
import datetime
import os
import random
import shutil
import signal
import subprocess
import sys
import threading
import time

import pyvisa

# ---------------------------------------------------------------- defaults
RESOURCE = "TCPIP0::10.68.63.118::5025::SOCKET"
INTERVAL = 30.0
CHANNELS = [1]
OUTFILE = "power_log.csv"
TIMEOUT_MS = 5000
GIT_INTERVAL = 300.0   # commit + push the CSV this often

# R&S returns 9.91E37 for "measurement not available". Parsing it as a real
# number would put a garbage spike in your data.
SCPI_NAN = 9.9e37

# Anything that means "the link misbehaved". ValueError covers a reply that
# won't parse as a float, which is itself a symptom of desync.
COMM_ERRORS = (pyvisa.VisaIOError, OSError, EOFError, ValueError)

RECONNECT_BACKOFF = [2, 5, 10, 30, 60]   # seconds, last value repeats
DISK_WARN_BYTES = 200 * 1024 * 1024


def log(msg):
    """Timestamped stderr-safe console line. Never blocks, never raises."""
    ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    try:
        print(f"{ts}  {msg}", flush=True)
    except (BrokenPipeError, OSError):
        pass   # stdout went away; the CSV is what matters


class Stopper:
    """Turns Ctrl-C, SIGTERM and SIGHUP into a clean shutdown."""

    def __init__(self):
        self.stop = False
        for sig in ("SIGINT", "SIGTERM", "SIGHUP", "SIGBREAK"):
            s = getattr(signal, sig, None)
            if s is not None:
                try:
                    signal.signal(s, self._handle)
                except (ValueError, OSError):
                    pass

    def _handle(self, signum, frame):
        self.stop = True
        log(f"signal {signum} received, finishing current sample and closing")

    def wait(self, seconds):
        """Sleep in slices so a signal doesn't wait out a full interval."""
        end = time.monotonic() + seconds
        while not self.stop:
            remaining = end - time.monotonic()
            if remaining <= 0:
                return
            time.sleep(min(remaining, 0.5))


# ------------------------------------------------------------------- link
class Link:
    """A VISA session that knows how to heal itself."""

    def __init__(self, resource, backend, timeout_ms, simulate=False):
        self.resource = resource
        self.backend = backend
        self.timeout_ms = timeout_ms
        self.simulate = simulate
        self.rm = None
        self.inst = None
        self.idn = ""

    @property
    def up(self):
        return self.inst is not None

    def open(self):
        self.close()
        if self.simulate:
            self.rm, self.inst = None, FakeInstrument()
        else:
            self.rm = pyvisa.ResourceManager(self.backend)
            inst = self.rm.open_resource(self.resource)
            inst.timeout = self.timeout_ms
            # SOCKET sessions don't negotiate line endings; without these the
            # first query hangs until timeout.
            if self.resource.upper().endswith("::SOCKET"):
                inst.read_termination = "\n"
                inst.write_termination = "\n"
            self.inst = inst
        self.inst.write("*CLS")
        self.idn = self.inst.query("*IDN?").strip()

    def close(self):
        for obj in (self.inst, self.rm):
            try:
                if obj is not None:
                    obj.close()
            except Exception:
                pass
        self.inst = None
        self.rm = None

    def query(self, cmd):
        return self.inst.query(cmd).strip()

    def write(self, cmd):
        self.inst.write(cmd)

    def resync(self):
        """Recover from a timed-out query.

        A timeout usually leaves the instrument's reply sitting in its output
        queue. Left alone, the next query returns that stale reply and every
        reading afterwards is shifted by one command - wrong numbers, no error.
        Device clear discards the queue; *OPC? then proves we're back in step.
        """
        try:
            self.inst.clear()          # not supported on some raw sockets
        except Exception:
            pass
        self.inst.write("*CLS")
        reply = self.inst.query("*OPC?").strip()
        if not reply.startswith("1"):
            raise IOError(f"still out of sync after clear (*OPC? -> {reply!r})")


# ---------------------------------------------------------------- reading
def parse_reading(text):
    v = float(text)
    if abs(v) >= SCPI_NAN:
        raise ValueError(f"instrument reported no valid measurement ({text})")
    return v


def read_channel(link, ch, multichannel):
    """Return (volts, amps) for one channel.

    MEAS: matters. A bare VOLT? gives the setpoint you programmed, not what
    the output is actually doing.
    """
    if multichannel:
        link.write(f"INST:NSEL {ch}")
        # Confirm the switch landed. Over hundreds of hours a dropped command
        # would otherwise file channel 2's readings under channel 1.
        got = link.query("INST:NSEL?")
        if int(float(got)) != ch:
            raise IOError(f"channel select failed: asked {ch}, got {got}")
    v = parse_reading(link.query("MEAS:VOLT?"))
    i = parse_reading(link.query("MEAS:CURR?"))
    return v, i


def sample_all(link, channels, multichannel):
    """One full pass. Raises on comms trouble so the caller can reconnect."""
    out = []
    for ch in channels:
        try:
            out.append(read_channel(link, ch, multichannel))
        except COMM_ERRORS:
            link.resync()          # raises if unrecoverable -> reconnect
            out.append(read_channel(link, ch, multichannel))   # one retry
    return out


def drain_errors(link, limit=20):
    """Empty the instrument's error queue so it can't overflow."""
    msgs = []
    for _ in range(limit):
        try:
            e = link.query("SYST:ERR?")
        except COMM_ERRORS:
            break
        if not e or e.startswith(("0,", "+0,")):
            break
        msgs.append(e)
    return msgs


# -------------------------------------------------------------- csv sink
class CsvSink:
    """Append-only CSV. Survives restarts, crashes and power loss."""

    def __init__(self, path, header):
        self.path = os.path.abspath(path)
        fresh = (not os.path.exists(self.path)) or os.path.getsize(self.path) == 0
        # Append, never truncate: a restart must not destroy earlier data.
        self.f = open(self.path, "a", newline="", encoding="utf-8")
        self.w = csv.writer(self.f)
        if fresh:
            self.w.writerow(header)
            self._commit()
        else:
            log(f"appending to existing {self.path}")

    def _commit(self):
        self.f.flush()
        os.fsync(self.f.fileno())   # flush() alone leaves it in OS cache

    def write(self, row):
        try:
            self.w.writerow(row)
            self._commit()
            return True
        except OSError as e:
            log(f"CSV WRITE FAILED: {e}")
            return False

    def free_bytes(self):
        try:
            return shutil.disk_usage(os.path.dirname(self.path) or ".").free
        except OSError:
            return None

    def close(self):
        try:
            self._commit()
            self.f.close()
        except OSError:
            pass


# -------------------------------------------------------------- git sync
class GitPusher:
    """Commits and pushes the CSV on a timer, on its own thread.

    Runs independently of the sampling loop so a slow/failed push (network
    down, no internet at the test bench) never delays or breaks logging.
    A remote-monitoring gap check relies on commit timestamps arriving, so
    failures are logged but never raised.
    """

    def __init__(self, repo_dir, file_path, interval, branch, enabled):
        self.repo_dir = repo_dir
        self.rel_path = os.path.relpath(file_path, repo_dir)
        self.interval = interval
        self.branch = branch
        self.enabled = enabled
        self._stop = threading.Event()
        self._thread = None

    def _git(self, *args):
        return subprocess.run(
            ["git", *args], cwd=self.repo_dir,
            capture_output=True, text=True,
        )

    def _push_once(self):
        try:
            add = self._git("add", self.rel_path)
            if add.returncode != 0:
                log(f"git add failed: {add.stderr.strip()}")
                return
            commit = self._git(
                "commit", "-m",
                f"data update {datetime.datetime.now().isoformat(timespec='seconds')}",
            )
            if commit.returncode != 0:
                if "nothing to commit" in (commit.stdout + commit.stderr).lower():
                    return
                log(f"git commit failed: {commit.stdout.strip()} {commit.stderr.strip()}")
                return
            push = self._git("push", "origin", self.branch)
            if push.returncode != 0:
                log(f"git push failed: {push.stderr.strip()}")
            else:
                log("git push ok")
        except Exception as e:
            log(f"git sync error: {e}")

    def _run(self):
        while not self._stop.wait(self.interval):
            self._push_once()
        self._push_once()   # final push on shutdown so the last rows land

    def start(self):
        if not self.enabled:
            return
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        if self._thread is None:
            return
        self._stop.set()
        self._thread.join(timeout=60)


# ------------------------------------------------------------- simulator
class FakeInstrument:
    """Stand-in that injects the failures a real run throws at you.

    Raises the same pyvisa.VisaIOError the driver does, so --simulate
    exercises the identical recovery paths as real hardware.
    """

    def __init__(self, fail_rate=0.15, drop_rate=0.04):
        self.fail_rate = fail_rate
        self.drop_rate = drop_rate
        self.ch = 1
        self.dead = False
        self.stale = None      # models the desync bug

    def _boom(self):
        raise pyvisa.VisaIOError(pyvisa.constants.StatusCode.error_timeout)

    def write(self, cmd):
        if self.dead:
            self._boom()
        if cmd.upper().startswith("INST:NSEL "):
            self.ch = int(cmd.split()[-1])
        if cmd.upper().startswith("*CLS"):
            self.stale = None

    def query(self, cmd):
        if self.dead:
            self._boom()
        if random.random() < self.drop_rate:
            self.dead = True
            self._boom()
        c = cmd.upper()
        if c.startswith("*IDN?"):
            return "Rohde&Schwarz,NGM202,SIM-0001,1.00\n"
        if c.startswith("*OPC?"):
            reply, self.stale = (self.stale or "1"), None
            return reply + "\n"
        if c.startswith("SYST:ERR?"):
            return "0,\"No error\"\n"
        if c.startswith("INST:NSEL?"):
            return f"{self.ch}\n"
        if random.random() < self.fail_rate:
            # Timeout AND leave the answer behind - exactly how real desync starts.
            self.stale = "12.345678"
            self._boom()
        if c.startswith("MEAS:VOLT?"):
            return f"{12.0 + self.ch + random.uniform(-0.01, 0.01):.6f}\n"
        if c.startswith("MEAS:CURR?"):
            return f"{0.5 + random.uniform(-0.005, 0.005):.6f}\n"
        return "0\n"

    def clear(self):
        if self.dead and random.random() < 0.5:
            self.dead = False      # link comes back
        self.stale = None

    def close(self):
        pass


# ------------------------------------------------------------------ main
def list_resources(backend):
    rm = pyvisa.ResourceManager(backend)
    found = rm.list_resources()
    if not found:
        log("No VISA resources found. LAN instruments often don't self-announce;")
        log("pass the address directly:  -r TCPIP0::<ip>::inst0::INSTR")
        return
    for r in found:
        try:
            with rm.open_resource(r) as dev:
                dev.timeout = 2000
                log(f"{r}\n    {dev.query('*IDN?').strip()}")
        except Exception:
            log(f"{r}\n    (no response)")


def main():
    p = argparse.ArgumentParser(description="Long-run V/I logger over PyVISA.")
    p.add_argument("-r", "--resource", default=RESOURCE)
    p.add_argument("-i", "--interval", type=float, default=INTERVAL)
    p.add_argument("-c", "--channels", type=int, nargs="+", default=CHANNELS)
    p.add_argument("-o", "--out", default=OUTFILE)
    p.add_argument("--backend", default="@py",
                   help="'@py' for pyvisa-py, '' for installed vendor VISA")
    p.add_argument("--timeout", type=int, default=TIMEOUT_MS, help="VISA ms")
    p.add_argument("--hours", type=float, default=None,
                   help="stop after this many hours (default: run forever)")
    p.add_argument("--list", action="store_true")
    p.add_argument("--simulate", action="store_true",
                   help="run against a fake instrument that injects faults")
    p.add_argument("--git-push", dest="git_push", action="store_true", default=True,
                   help="commit+push the CSV to git every --git-interval seconds (default: on)")
    p.add_argument("--no-git-push", dest="git_push", action="store_false",
                   help="disable automatic git commit/push")
    p.add_argument("--git-interval", type=float, default=GIT_INTERVAL,
                   help="seconds between git commit/push cycles (default 300 = 5 min)")
    p.add_argument("--git-repo-dir", default=None,
                   help="git repo root (default: directory containing --out)")
    p.add_argument("--git-branch", default="main")
    args = p.parse_args()

    if args.list:
        list_resources(args.backend)
        return 0

    if args.interval <= 0:
        sys.exit("--interval must be positive")

    multichannel = len(args.channels) > 1
    stopper = Stopper()

    header = ["timestamp", "elapsed_s", "status"]
    for ch in args.channels:
        header += [f"ch{ch}_voltage_V", f"ch{ch}_current_A", f"ch{ch}_power_W"]
    sink = CsvSink(args.out, header)

    repo_dir = args.git_repo_dir or os.path.dirname(sink.path) or "."
    pusher = GitPusher(repo_dir, sink.path, args.git_interval, args.git_branch, args.git_push)
    pusher.start()
    if args.git_push:
        log(f"git auto-push enabled: every {args.git_interval:g}s to origin/{args.git_branch} in {repo_dir}")

    link = Link(args.resource, args.backend, args.timeout, args.simulate)
    blanks = ["", "", ""] * len(args.channels)

    t0 = time.monotonic()
    deadline = t0 + args.hours * 3600 if args.hours else None
    n = 0
    written = gaps = reconnects = 0
    outage_since = None
    fail_streak = 0
    last_housekeeping = 0.0

    log(f"logging every {args.interval:g}s to {sink.path}")
    log("Ctrl-C to stop")

    try:
        while not stopper.stop:
            if deadline and time.monotonic() >= deadline:
                log("requested duration reached")
                break

            stamp = datetime.datetime.now().astimezone().isoformat(timespec="seconds")
            elapsed = round(time.monotonic() - t0, 1)
            status = "ok"

            # -- reconnect if we're down ----------------------------------
            if not link.up:
                try:
                    link.open()
                    status = "reconnected"
                    reconnects += 1
                    fail_streak = 0
                    down = f" after {time.monotonic() - outage_since:.0f}s" if outage_since else ""
                    outage_since = None
                    log(f"connected{down}: {link.idn}")
                except Exception as e:
                    link.close()
                    if outage_since is None:
                        outage_since = time.monotonic()
                        log(f"link down: {e}")
                    fail_streak += 1
                    if fail_streak in (5, 20, 100) or fail_streak % 500 == 0:
                        mins = (time.monotonic() - outage_since) / 60
                        log(f"still down after {fail_streak} tries ({mins:.0f} min)")

            # -- take the sample ------------------------------------------
            if link.up:
                try:
                    readings = sample_all(link, args.channels, multichannel)
                    cells = []
                    for v, i in readings:
                        cells += [f"{v:.6f}", f"{i:.6f}", f"{v * i:.6f}"]
                    fail_streak = 0
                except Exception as e:
                    log(f"sample failed, dropping link: {e}")
                    link.close()
                    outage_since = outage_since or time.monotonic()
                    cells, status = blanks, "comms_error"
            else:
                cells, status = blanks, "disconnected"

            if status in ("comms_error", "disconnected"):
                gaps += 1
            if sink.write([stamp, elapsed, status] + cells):
                written += 1
            log(f"{status:12s} " + "  ".join(str(c) for c in cells))

            # -- housekeeping, roughly hourly ------------------------------
            if time.monotonic() - last_housekeeping > 3600:
                last_housekeeping = time.monotonic()
                free = sink.free_bytes()
                if free is not None and free < DISK_WARN_BYTES:
                    log(f"LOW DISK: {free / 1e6:.0f} MB free")
                if link.up:
                    for e in drain_errors(link):
                        log(f"instrument error queue: {e}")
                log(f"uptime {(time.monotonic() - t0) / 3600:.1f} h  "
                    f"rows {written}  gaps {gaps}  reconnects {reconnects}")

            # -- schedule the next slot ------------------------------------
            # Anchored to t0 so it never drifts, and skips missed slots
            # instead of firing a catch-up burst after a slow patch.
            n += 1
            now = time.monotonic()
            target = t0 + n * args.interval
            if target <= now:
                n += int((now - target) // args.interval) + 1
                target = t0 + n * args.interval
            stopper.wait(target - now)

    except Exception as e:
        log(f"UNEXPECTED ERROR: {type(e).__name__}: {e}")
        return 1
    finally:
        sink.close()
        link.close()
        pusher.stop()
        hours = (time.monotonic() - t0) / 3600
        log(f"stopped after {hours:.2f} h - {written} rows "
            f"({gaps} gaps, {reconnects} reconnects) in {sink.path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
