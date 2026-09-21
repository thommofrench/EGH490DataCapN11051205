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
# Anchored to the script's own folder (the git repo), not the caller's cwd,
# so running it via a shortcut or from a different directory still writes
# into - and pushes from - the right place.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RESOURCE = "TCPIP0::10.68.63.118::5025::SOCKET"
INTERVAL = 30.0
CHANNELS = [1]
OUTFILE = os.path.join(SCRIPT_DIR, "dummy_test4_AC_test_power_log.csv")
TIMEOUT_MS = 5000
GIT_INTERVAL = 300.0   # commit + push the CSV this often
# The NGE103B on this bench. PSU control is ON by default; use --no-psu for a
# DMM-only run so a quick check never energises the PSU outputs.
PSU_RESOURCE = "USB0::0x0AAD::0x0197::5601.3800k03-113360::0::INSTR"

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

    Current is read first and voltage last on purpose: each MEAS: query
    switches the meter's measurement mode, and the mode left active between
    samples loads the circuit. Ending on voltage leaves the meter in its
    high-impedance mode instead of the low-impedance current mode, so the
    idle meter doesn't disturb the source under test.
    """
    if multichannel:
        link.write(f"INST:NSEL {ch}")
        # Confirm the switch landed. Over hundreds of hours a dropped command
        # would otherwise file channel 2's readings under channel 1.
        got = link.query("INST:NSEL?")
        if int(float(got)) != ch:
            raise IOError(f"channel select failed: asked {ch}, got {got}")
    i = parse_reading(link.query("MEAS:CURR?"))
    v = parse_reading(link.query("MEAS:VOLT?"))
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


# --------------------------------------------------------- PSU waveform
class PsuWaveform:
    """Drives an alternating current-setpoint square wave on an R&S NGE100
    PSU, on its own thread and its own VISA link.

    The NGE100's onboard Arbitrary/EasyArb engine only runs on channel 1
    (confirmed in the R&S NGE100 user manual, "Arbitrary Commands" section -
    every ARBitrary:* command is documented as acting on "channel 1"), so it
    can't drive two parallel-wired channels together. At a multi-minute
    period, host-timed SCPI writes are far more precise than needed, so this
    just sets SOUR:CURR on each channel directly instead of using ARB.

    Runs independently of the DMM link and the git pusher so a PSU comms
    hiccup never stops DMM logging, and vice versa.

    On every (re)connect the voltage is set first, then the current for the
    current phase, and only then are the channel outputs switched on, so the
    output never comes up at a stale setpoint. Outputs are switched off again
    on a clean shutdown (a hard kill can't do this).
    """

    def __init__(self, resource, backend, timeout_ms, channels,
                 peak_current, mod_depth, period_s, duty, voltage):
        self.link = Link(resource, backend, timeout_ms)
        self.channels = channels
        self.high = peak_current / len(channels)
        self.low = peak_current * (1 - mod_depth) / len(channels)
        self.high_time = period_s * duty
        self.low_time = period_s - self.high_time
        self.voltage = voltage
        self._stop = threading.Event()
        self._thread = None
        # Read from the main thread for CSV logging; plain attribute writes
        # are atomic under the GIL, so no lock is needed for this.
        self.phase = "pending"
        self.target_total_A = None
        # True after each (re)connect until the outputs have been switched on,
        # which happens in _run once the first current setpoint is applied.
        self._needs_enable = True

    def _apply(self, current):
        for ch in self.channels:
            self.link.write(f"INST:NSEL {ch}")
            self.link.write(f"SOUR:CURR {current:.4f}")

    def _set_outputs(self, on):
        """Switch each driven channel's output on/off (per channel, so other
        channels on the PSU are never touched), then surface any error the
        instrument queued for it."""
        state = "ON" if on else "OFF"
        for ch in self.channels:
            self.link.write(f"INST:NSEL {ch}")
            self.link.write(f"OUTP:STAT {state}")
        log(f"psu: outputs {state} on ch{self.channels}")
        for e in drain_errors(self.link):
            log(f"psu: instrument error after outputs {state}: {e}")

    def _ensure_connected(self):
        if self.link.up:
            return True
        try:
            self.link.open()
            self._needs_enable = True
            log(f"psu connected: {self.link.idn}")
            if self.voltage is not None:
                for ch in self.channels:
                    self.link.write(f"INST:NSEL {ch}")
                    self.link.write(f"SOUR:VOLT {self.voltage}")
                log(f"psu: voltage set to {self.voltage:g} V on ch{self.channels}")
            return True
        except Exception as e:
            log(f"psu link down: {e}")
            self.link.close()
            return False

    def _run(self):
        state_high = True
        while not self._stop.is_set():
            if not self._ensure_connected():
                if self._stop.wait(10):
                    break
                continue
            level = self.high if state_high else self.low
            try:
                self._apply(level)
                if self._needs_enable:
                    # Voltage went in on connect, current just now; safe to enable.
                    self._set_outputs(True)
                    self._needs_enable = False
                total = level * len(self.channels)
                self.phase = "high" if state_high else "low"
                self.target_total_A = total
                log(f"psu waveform: {level:.3f} A/ch "
                    f"({total:.3f} A total, {self.phase})")
            except COMM_ERRORS as e:
                log(f"psu: set failed, dropping link: {e}")
                self.link.close()
                continue   # retry the connection right away, not after a full half-period
            wait_s = self.high_time if state_high else self.low_time
            state_high = not state_high
            if self._stop.wait(wait_s):
                break

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=10)
        # Best effort: leave the PSU outputs off rather than stuck at the last
        # level. Only if the link is already up - no reconnect attempt, so
        # shutdown stays fast when the PSU is unreachable.
        if self.link.up:
            try:
                self._set_outputs(False)
            except Exception as e:
                log(f"psu: could not switch outputs off on exit: {e}")
        else:
            log("psu: link down at exit, outputs NOT switched off")
        self.link.close()


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
    p.add_argument("--git-repo-dir", default=SCRIPT_DIR,
                   help="git repo root (default: the folder this script lives in)")
    p.add_argument("--git-branch", default="main")
    p.add_argument("--psu-resource", default=PSU_RESOURCE,
                   help="VISA resource for the R&S NGE100 PSU, e.g. "
                        "USB0::0x0AAD::0x0197::<serial>::0::INSTR. "
                        "Defaults to the bench NGE103B; see --no-psu.")
    p.add_argument("--no-psu", action="store_true",
                   help="disable PSU waveform control entirely (DMM logging only); "
                        "the PSU outputs are never touched")
    p.add_argument("--psu-backend", default="",
                   help="VISA backend for the PSU ('' = installed vendor VISA, "
                        "needed for USB; '@py' for pyvisa-py)")
    p.add_argument("--psu-channels", type=int, nargs="+", default=[1, 2],
                   help="PSU channels wired in parallel and driven together")
    p.add_argument("--psu-voltage", type=float, default=1,
                   help="voltage set on each PSU channel at startup/reconnect "
                        "(the shared CV ceiling for parallel-wired channels), "
                        "default 1 V.")
    p.add_argument("--psu-peak-current", type=float, default=5.0,
                   help="TOTAL peak current across all --psu-channels combined, in amps")
    p.add_argument("--psu-mod-depth", type=float, default=0.8,
                   help="modulation depth: low level = peak * (1 - depth)")
    p.add_argument("--psu-period", type=float, default=240.0,
                   help="waveform period in seconds (default 240 = 4 min = 4.17 mHz)")
    p.add_argument("--psu-duty", type=float, default=0.5,
                   help="fraction of the period spent at the high (peak) level")
    args = p.parse_args()
    if args.no_psu:
        args.psu_resource = None

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
    if args.psu_resource:
        header += ["psu_target_A_total", "psu_phase"]
    sink = CsvSink(args.out, header)

    pusher = GitPusher(args.git_repo_dir, sink.path, args.git_interval, args.git_branch, args.git_push)
    pusher.start()
    if args.git_push:
        log(f"git auto-push enabled: every {args.git_interval:g}s to origin/{args.git_branch} in {args.git_repo_dir}")

    psu = None
    if args.psu_resource:
        psu = PsuWaveform(args.psu_resource, args.psu_backend, args.timeout,
                           args.psu_channels, args.psu_peak_current, args.psu_mod_depth,
                           args.psu_period, args.psu_duty, args.psu_voltage)
        psu.start()
        low = args.psu_peak_current * (1 - args.psu_mod_depth)
        log(f"psu waveform enabled on ch{args.psu_channels}: "
            f"{args.psu_peak_current:g} A / {low:g} A total (peak/low), "
            f"{args.psu_period:g}s period, {args.psu_duty:.0%} duty")
        if args.psu_voltage is None:
            log("psu: voltage left as already configured on the instrument "
                "(pass --psu-voltage to set it explicitly); outputs will be "
                "switched on once connected")
        else:
            log(f"psu: on connect will set {args.psu_voltage:g} V per channel, "
                f"then switch outputs on")

    link = Link(args.resource, args.backend, args.timeout, args.simulate)
    blanks = ["", "", ""] * len(args.channels)
    if args.psu_resource:
        blanks = blanks + ["", ""]

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
                    if psu is not None:
                        target = psu.target_total_A
                        cells += [f"{target:.4f}" if target is not None else "", psu.phase]
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
        if psu is not None:
            psu.stop()
        pusher.stop()
        hours = (time.monotonic() - t0) / 3600
        log(f"stopped after {hours:.2f} h - {written} rows "
            f"({gaps} gaps, {reconnects} reconnects) in {sink.path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
