#!/usr/bin/env python3
"""
Long-run voltage logger for Rohde & Schwarz instruments (PyVISA).

Two DMMs read DC voltage only - the cell voltage and the Hall voltage - into
one CSV, while an NGE103B PSU drives a square-wave current through the cell.
Each DMM has its own link, so either can drop out without stopping the other.

Built to survive unattended multi-week runs: it reconnects on its own, detects
the VISA desync that silently corrupts readings after a timeout, appends rather
than truncates so a restart never destroys data, and fsyncs every row so a power
cut on the PC costs you at most one sample.

Install:
    pip install pyvisa pyvisa-py

Self-test with no hardware (recommended before any long run):
    python scpi_vi_logger.py --simulate --no-psu --interval 1 --hours 0.02

Real use:
    python scpi_vi_logger.py --list
    python scpi_vi_logger.py --cell-resource USB0::... --hall-resource USB0::...

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
# The two UDS500 DMMs, both on USB and both reading DC voltage only.
CELL_RESOURCE = "USB0::0x0AAD::0x0135::000100196::INSTR"
HALL_RESOURCE = "USB0::0x0AAD::0x0135::000100194::INSTR"
INTERVAL = 30.0
OUTFILE = os.path.join(SCRIPT_DIR, "prelim_test3_PI_USB_cell_hall_voltage_log.csv")
TIMEOUT_MS = 5000
GIT_INTERVAL = 300.0   # commit + push the CSV this often
GIT_TIMEOUT = 90.0     # kill any single git command that runs longer than this
STALE_LOCK_S = 600.0   # a .git lock file older than this is from a dead git
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
        # Close only this instrument's session, never self.rm: pyvisa shares
        # one resource manager per VISA library, and closing it would also
        # kill the other DMM's (and the PSU's) session.
        try:
            if self.inst is not None:
                self.inst.close()
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


def read_voltage(link):
    """One fresh DC voltage reading.

    Voltage is the only mode the meters are ever put in, so they stay in
    high-impedance mode and never load the circuit under test.
    """
    return parse_reading(link.query("MEAS:VOLT?"))


def sample(link):
    """Raises on comms trouble so the caller can reconnect."""
    try:
        return read_voltage(link)
    except COMM_ERRORS:
        link.resync()          # raises if unrecoverable -> reconnect
        return read_voltage(link)   # one retry


class Meter:
    """One DMM and its own reconnect bookkeeping, so either meter can drop
    out and come back without affecting the other's readings."""

    def __init__(self, name, link):
        self.name = name
        self.link = link
        self.outage_since = None
        self.fail_streak = 0
        self.reconnects = 0

    def read(self):
        """Return (status, cell) for this slot; cell is the voltage as text,
        or blank when there's no reading."""
        status = "ok"
        if not self.link.up:
            try:
                self.link.open()
            except Exception as e:
                self.link.close()
                if self.outage_since is None:
                    self.outage_since = time.monotonic()
                    log(f"{self.name}: link down: {e}")
                self.fail_streak += 1
                if self.fail_streak in (5, 20, 100) or self.fail_streak % 500 == 0:
                    mins = (time.monotonic() - self.outage_since) / 60
                    log(f"{self.name}: still down after {self.fail_streak} tries ({mins:.0f} min)")
                return "disconnected", ""
            status = "reconnected"
            self.reconnects += 1
            self.fail_streak = 0
            down = f" after {time.monotonic() - self.outage_since:.0f}s" if self.outage_since else ""
            self.outage_since = None
            log(f"{self.name}: connected{down}: {self.link.idn}")
        try:
            v = sample(self.link)
        except Exception as e:
            log(f"{self.name}: sample failed, dropping link: {e}")
            self.link.close()
            self.outage_since = self.outage_since or time.monotonic()
            return "comms_error", ""
        self.fail_streak = 0
        return status, f"{v:.6f}"


def combine_status(meters, statuses):
    """'ok' when both meters agree on it, else e.g. 'cell:ok hall:comms_error'."""
    if len(set(statuses)) == 1:
        return statuses[0]
    return " ".join(f"{m.name}:{s}" for m, s in zip(meters, statuses))


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

    # Words in git's output that mean the local repo itself is damaged, as
    # opposed to the network or the remote being the problem.
    CORRUPT_SIGNS = ("is empty", "corrupt", "could not parse head", "bad object",
                     "unable to read", "invalid sha1 pointer", "loose object")
    REPAIR_GAP_S = 600   # at most one repair attempt per this many seconds
    FIRST_PUSH_S = 60.0  # first push this long after start, then every interval

    def __init__(self, repo_dir, file_path, interval, branch, enabled):
        self.repo_dir = repo_dir
        self.rel_path = os.path.relpath(file_path, repo_dir)
        self.interval = interval
        self.branch = branch
        self.enabled = enabled
        self._stop = threading.Event()
        self._thread = None
        self._last_repair = None
        # Read by the main thread's hourly status line.
        self.started = time.monotonic()
        self.last_ok = None

    def _git(self, *args, cwd=None):
        """Run one git command, killed if it runs past GIT_TIMEOUT.

        Without a timeout, one push that hangs on a dead WiFi link would block
        this thread, and so every later push, for the rest of the run.

        core.fsync makes git flush each object and ref to disk as it writes
        it. Without it, a power cut on the Pi left a pushed commit as an empty
        file in the local repo, and every commit after that failed.

        Every commit stores a whole new copy of the CSV, and by default git
        only packs them (as small deltas) once ~6700 loose objects pile up -
        gigabytes of SD card by week three. gc.auto=256 packs every few hours
        instead, single-threaded to keep the Pi's memory free.
        """
        p = subprocess.Popen(
            ["git", "-c", "core.fsync=all", "-c", "gc.auto=256", "-c", "pack.threads=1",
             *args], cwd=cwd or self.repo_dir,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
            env={**os.environ, "GIT_TERMINAL_PROMPT": "0"},
            # Own process group, so a timeout can kill git's ssh child too.
            # Killing git alone leaves ssh holding the output pipes open and
            # communicate() below would still hang.
            start_new_session=(os.name != "nt"),
        )
        try:
            out, err = p.communicate(timeout=GIT_TIMEOUT)
        except subprocess.TimeoutExpired:
            _kill_tree(p)
            out, err = p.communicate()
            err += f"\n(git {args[0]} killed after {GIT_TIMEOUT:g}s)"
        return subprocess.CompletedProcess(p.args, p.returncode, out, err)

    def _clear_stale_locks(self):
        """Delete git lock files left behind by a killed or crashed git.

        A git that dies mid-command (timeout kill, power cut) leaves its
        .lock file, and every later git command then refuses to run. A lock
        older than STALE_LOCK_S can't belong to a live git: each git run is
        killed after GIT_TIMEOUT, which is far shorter.
        """
        git_dir = os.path.join(self.repo_dir, ".git")
        for root, dirs, files in os.walk(git_dir):
            if root == git_dir:
                dirs[:] = ["refs"]   # locks live in .git itself and under .git/refs
            for name in files:
                if not name.endswith(".lock"):
                    continue
                path = os.path.join(root, name)
                try:
                    age = time.time() - os.path.getmtime(path)
                    if age > STALE_LOCK_S:
                        os.remove(path)
                        log(f"git: removed stale lock {os.path.relpath(path, self.repo_dir)} "
                            f"({age / 60:.0f} min old)")
                except OSError:
                    pass   # already gone, or the other logger just took it

    def _git_retry(self, *args, attempts=5):
        """Run git, retrying while another process holds the repo lock.

        The AC run and the DC baseline run are separate logger processes
        pushing from the same repo, so their add/commit/push cycles can
        overlap and trip over each other's .git/index.lock or ref locks. Their
        pushes also start in step, and the remote then refuses whichever ref
        update lands second ("cannot lock ref" / "failed to update ref").
        """
        for n in range(attempts):
            r = self._git(*args)
            out = r.stdout + r.stderr
            contended = (".lock" in out or "cannot lock ref" in out
                         or "failed to update ref" in out)
            if r.returncode == 0 or not contended:
                return r
            time.sleep(1 + n + random.random() * 2)
        return r

    def _push_once(self):
        try:
            self._clear_stale_locks()
            failed = self._sync()
            if failed is not None and self._looks_corrupt(failed) and self._repair():
                failed = self._sync()   # repaired: don't wait a whole interval
            if failed is None:
                self.last_ok = time.monotonic()
        except Exception as e:
            log(f"git sync error: {e}")

    def _sync(self):
        """add, commit, push. Returns None on success, else the failed git
        result (already logged)."""
        add = self._git_retry("add", self.rel_path)
        if add.returncode != 0:
            log(f"git add failed: {add.stderr.strip()}")
            return add
        commit = self._git_retry(
            "commit", "-m",
            f"data update {datetime.datetime.now().isoformat(timespec='seconds')}",
        )
        # "nothing to commit" still falls through to the push: the other
        # logger process may have committed our rows in its own commit, or
        # an earlier push may have failed, leaving local commits unpushed.
        out = (commit.stdout + commit.stderr).lower()
        if commit.returncode != 0 and \
                "nothing to commit" not in out and "nothing added to commit" not in out:
            log(f"git commit failed: {commit.stdout.strip()} {commit.stderr.strip()}")
            return commit
        push = self._git_retry("push", "origin", self.branch)
        if push.returncode != 0 and ("fetch first" in push.stderr
                                     or "non-fast-forward" in push.stderr):
            if self._merge_remote():
                push = self._git_retry("push", "origin", self.branch)
        if push.returncode != 0:
            log(f"git push failed: {push.stderr.strip()}")
            return push
        log("git push ok")
        return None

    def _looks_corrupt(self, result):
        out = (result.stdout + result.stderr).lower()
        return any(s in out for s in self.CORRUPT_SIGNS) and not self._repo_ok()

    def _repo_ok(self):
        """Can git read what every commit needs: HEAD's commit and tree, and
        the index?"""
        return all(self._git(*a).returncode == 0 for a in (
            ("cat-file", "-p", "HEAD"), ("ls-tree", "HEAD"), ("status", "--porcelain", "-uno")))

    def _repair(self):
        """Replace a damaged .git with a fresh copy of origin's history.

        Only .git is swapped; the working tree, and with it the CSVs the
        loggers hold open, is never touched. Rows that were committed locally
        but never pushed are still in those CSVs, so the next commit after
        the repair picks them up and nothing is lost. The damaged copy is
        kept as .git-broken until the next repair, for a look by hand.
        """
        now = time.monotonic()
        if self._last_repair is not None and now - self._last_repair < self.REPAIR_GAP_S:
            return False
        self._last_repair = now
        repo, git_dir = self.repo_dir, os.path.join(self.repo_dir, ".git")
        # Both loggers share the repo; only one may repair it at a time.
        lock = os.path.join(repo, ".git-repair.lock")
        try:
            os.mkdir(lock)
        except FileExistsError:
            if time.time() - os.path.getmtime(lock) < 1800:
                log("git: repo damaged; the other logger is already repairing it")
                return False
            os.utime(lock)   # left by a repair that died; take it over
        try:
            if self._repo_ok():
                return True   # the other logger repaired it just now
            url = self._git("config", "--get", "remote.origin.url").stdout.strip()
            log(f"git: local repo is damaged, re-downloading its history from {url}")
            tmp = os.path.join(repo, ".git-repair-tmp")
            shutil.rmtree(tmp, ignore_errors=True)
            clone = self._git("clone", "--bare", "--quiet", url, tmp,
                              cwd=os.path.dirname(os.path.abspath(repo)))
            if clone.returncode != 0:
                log(f"git repair: clone failed, will retry later: {clone.stderr.strip()}")
                shutil.rmtree(tmp, ignore_errors=True)
                return False
            # Keep this repo's own settings (remote, identity, branch tracking).
            try:
                shutil.copyfile(os.path.join(git_dir, "config"), os.path.join(tmp, "config"))
            except OSError:
                pass
            broken = os.path.join(repo, ".git-broken")
            shutil.rmtree(broken, ignore_errors=True)
            os.rename(git_dir, broken)
            os.rename(tmp, git_dir)
            self._git("config", "core.bare", "false")
            self._git("config", "remote.origin.url", url)
            reset = self._git("reset", "--quiet")   # rebuild the index; working tree untouched
            ok = reset.returncode == 0 and self._repo_ok()
            log("git repair: done, damaged copy kept in .git-broken" if ok else
                f"git repair: FAILED after swap: {reset.stderr.strip()}")
            return ok
        finally:
            shutil.rmtree(lock, ignore_errors=True)

    def _merge_remote(self):
        """Bring in commits pushed from elsewhere (e.g. code from the laptop)
        so our push isn't rejected for being behind. Returns True if merged.

        Merge, never rebase: a rebase rewrites the CSVs in the working tree,
        replacing the file a logger is still appending to, so its later rows
        would land in a deleted file. A merge leaves alone every file the
        incoming commits don't change, so it's only done when none of them
        touch a CSV. Merged code takes effect on the next service restart.
        """
        fetch = self._git_retry("fetch", "origin", self.branch)
        if fetch.returncode != 0:
            log(f"git fetch failed: {fetch.stderr.strip()}")
            return False
        upstream = f"origin/{self.branch}"
        changed = self._git("diff", "--name-only", f"HEAD...{upstream}").stdout.split()
        data = [f for f in changed if f.endswith(".csv")]
        if data:
            log(f"git: NOT merging {upstream}, it changes data files {data}; "
                f"pushes will keep failing until this is merged by hand")
            return False
        n = self._git("rev-list", "--count", f"HEAD..{upstream}").stdout.strip()
        merge = self._git_retry("merge", "--no-edit", upstream)
        if merge.returncode != 0:
            self._git("merge", "--abort")
            log(f"git merge of {upstream} failed: {merge.stdout.strip()} {merge.stderr.strip()}")
            return False
        log(f"git: merged {n} new commit(s) from {upstream} "
            f"({', '.join(changed) or 'no file changes'}); new code runs after a restart")
        return True

    def _run(self):
        # First push soon after starting rather than a whole interval in:
        # after a power cut this uploads the rows logged since the last push
        # before the cut, and shows on GitHub that the run is back.
        wait = min(self.FIRST_PUSH_S, self.interval)
        while not self._stop.wait(wait):
            self._push_once()
            wait = self.interval
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
        # Long enough for a final add/commit/push that each run to the git
        # timeout. The systemd units' TimeoutStopSec must stay above this.
        self._thread.join(timeout=3 * GIT_TIMEOUT + 30)


def _kill_tree(p):
    """Kill a subprocess and everything it started."""
    try:
        if os.name == "nt":
            subprocess.run(["taskkill", "/F", "/T", "/PID", str(p.pid)],
                           capture_output=True)
        else:
            os.killpg(p.pid, signal.SIGKILL)   # p leads its own group
    except (OSError, ProcessLookupError):
        pass
    try:
        p.kill()
    except OSError:
        pass


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

    Between phase changes the PSU is pinged every CHECK_S. A PSU that was
    power-cycled comes back with its outputs off and its old USB session
    dead, so the ping fails, the link is rebuilt and the outputs re-enabled
    within seconds rather than at the next phase change. A front-panel
    Output-off is left alone: the USB link survives that, so it's never
    mistaken for a restart. While the link is down the CSV's psu_phase reads
    "psu_down" and the target is blank, so the data never claims a current
    the PSU isn't being told to deliver.
    """

    CHECK_S = 5.0       # ping interval between phase changes
    RECONNECT_S = 10.0  # wait between failed connection attempts

    def __init__(self, resource, backend, timeout_ms, channels,
                 peak_current, mod_depth, period_s, duty, voltage):
        self.link = Link(resource, backend, timeout_ms)
        self.channels = channels
        self.high = peak_current / len(channels)
        self.low = peak_current * (1 - mod_depth) / len(channels)
        self.high_time = period_s * duty
        self.low_time = period_s - self.high_time
        self.voltage = voltage
        # --psu-mod-depth 0 gives a flat DC setpoint (the baseline run). The
        # loop still re-applies it every half period, which harmlessly
        # re-asserts the setpoint, but the phase is logged as "dc" rather than
        # a meaningless high/low alternation. "psu_down" while disconnected.
        self.dc = mod_depth == 0
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
        instrument queued for it.

        Per-channel OUTP:STAT only arms a channel; nothing actually comes out
        until the NGE100's master output switch (OUTP:GEN, the front-panel
        "Output" button) is also on. That switch is instrument-wide, so it's
        only ever turned ON here, never OFF - flipping it off at shutdown
        would kill any other channel someone else has running.
        """
        state = "ON" if on else "OFF"
        for ch in self.channels:
            self.link.write(f"INST:NSEL {ch}")
            self.link.write(f"OUTP:STAT {state}")
        if on:
            self.link.write("OUTP:GEN ON")
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

    def _mark_down(self):
        self.phase = "psu_down"
        self.target_total_A = None

    def _run(self):
        state_high = True
        switch_at = None   # monotonic time of the next phase change; None = set it now
        while not self._stop.is_set():
            # Nothing may escape this loop: a dead thread would leave the PSU
            # uncontrolled for the rest of the run while DMM logging carried on.
            try:
                if not self._ensure_connected():
                    self._mark_down()
                    self._stop.wait(self.RECONNECT_S)
                    continue
                now = time.monotonic()
                if switch_at is not None and now >= switch_at:
                    state_high = not state_high
                    switch_at = None
                if switch_at is None or self._needs_enable:
                    level = self.high if state_high else self.low
                    self._apply(level)
                    if self._needs_enable:
                        # Voltage went in on connect, current just now; safe to enable.
                        self._set_outputs(True)
                        self._needs_enable = False
                    total = level * len(self.channels)
                    self.phase = "dc" if self.dc else ("high" if state_high else "low")
                    self.target_total_A = total
                    log(f"psu waveform: {level:.3f} A/ch "
                        f"({total:.3f} A total, {self.phase})")
                    if switch_at is None:
                        switch_at = now + (self.high_time if state_high else self.low_time)
                else:
                    self.link.query("*OPC?")   # still there? fails if the PSU restarted
            except COMM_ERRORS as e:
                log(f"psu: comms failed, dropping link: {e}")
                self.link.close()
                self._mark_down()
                # Reconnect almost at once (outputs are re-enabled on
                # reconnect); the pause only stops a PSU that fails straight
                # after every connect from spinning this loop.
                self._stop.wait(1)
                continue
            except Exception as e:
                log(f"psu: unexpected {type(e).__name__}: {e}; dropping link")
                self.link.close()
                self._mark_down()
                self._stop.wait(self.RECONNECT_S)
                continue
            self._stop.wait(max(0.0, min(self.CHECK_S, switch_at - time.monotonic())))

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
    p = argparse.ArgumentParser(description="Long-run cell/Hall voltage logger over PyVISA.")
    p.add_argument("--cell-resource", default=CELL_RESOURCE,
                   help="VISA resource of the DMM reading the cell voltage")
    p.add_argument("--hall-resource", default=HALL_RESOURCE,
                   help="VISA resource of the DMM reading the Hall voltage")
    p.add_argument("-i", "--interval", type=float, default=INTERVAL)
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
    p.add_argument("--psu-voltage", type=float, default=2.1,
                   help="voltage set on each PSU channel at startup/reconnect "
                        "(the shared CV ceiling for parallel-wired channels), "
                        "default 2.1 V.")
    p.add_argument("--psu-peak-current", type=float, default=5.0,
                   help="TOTAL peak current across all --psu-channels combined, in amps")
    p.add_argument("--psu-mod-depth", type=float, default=0.8,
                   help="modulation depth: low level = peak * (1 - depth); "
                        "0 = constant DC at --psu-peak-current (baseline run)")
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

    stopper = Stopper()

    header = ["timestamp", "elapsed_s", "status", "Cell Voltage (V)", "Hall Voltage (V)"]
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
        if psu.dc:
            log(f"psu DC enabled on ch{args.psu_channels}: "
                f"{args.psu_peak_current:g} A total, constant")
        else:
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

    meters = [Meter("cell", Link(args.cell_resource, args.backend, args.timeout, args.simulate)),
              Meter("hall", Link(args.hall_resource, args.backend, args.timeout, args.simulate))]

    t0 = time.monotonic()
    deadline = t0 + args.hours * 3600 if args.hours else None
    n = 0
    written = gaps = 0
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

            # -- take the sample (each meter reconnects itself if down) ---
            statuses, cells = [], []
            for m in meters:
                s, v = m.read()
                statuses.append(s)
                cells.append(v)
            status = combine_status(meters, statuses)
            # The PSU runs on its own thread, so its state is logged even
            # when a meter is down.
            if psu is not None:
                target = psu.target_total_A
                cells += [f"{target:.4f}" if target is not None else "", psu.phase]

            if any(s in ("comms_error", "disconnected") for s in statuses):
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
                for m in meters:
                    if m.link.up:
                        for e in drain_errors(m.link):
                            log(f"{m.name}: instrument error queue: {e}")
                log(f"uptime {(time.monotonic() - t0) / 3600:.1f} h  "
                    f"rows {written}  gaps {gaps}  reconnects "
                    + "  ".join(f"{m.name} {m.reconnects}" for m in meters))
                if pusher.enabled:
                    mins = (time.monotonic() - (pusher.last_ok or pusher.started)) / 60
                    if mins > 3 * args.git_interval / 60:
                        log(f"WARNING: no successful git push for {mins:.0f} min; "
                            f"data is safe in the CSV but not reaching GitHub")
                    elif pusher.last_ok is not None:
                        log(f"git: last push ok {mins:.0f} min ago")

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
        for m in meters:
            m.link.close()
        if psu is not None:
            psu.stop()
        pusher.stop()
        hours = (time.monotonic() - t0) / 3600
        log(f"stopped after {hours:.2f} h - {written} rows ({gaps} gaps, reconnects "
            + ", ".join(f"{m.name} {m.reconnects}" for m in meters) + f") in {sink.path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
