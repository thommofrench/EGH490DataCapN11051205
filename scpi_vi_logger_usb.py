#!/usr/bin/env python3
"""
scpi_vi_logger.py, but with the two DMMs on their USB-B ports.

Everything else is unchanged and comes straight from scpi_vi_logger.py: the CSV
logging, the GitHub auto-push, the NGE103B waveform, reconnect handling and
clean shutdown. This file only changes how the DMMs are reached, so a fix made
in scpi_vi_logger.py applies to both.

Same command line as scpi_vi_logger.py:
    python scpi_vi_logger_usb.py                 # both DMMs + PSU on USB
    python scpi_vi_logger_usb.py --no-psu        # DMMs only
    python scpi_vi_logger_usb.py --list          # show connected USB instruments
    python scpi_vi_logger_usb.py --cell-resource USB0::... --hall-resource USB0::...

The DMM addresses default to CELL_RESOURCE and HALL_RESOURCE in
scpi_vi_logger.py; run --list to find them.

What differs from scpi_vi_logger.py
-----------------------------------
1. The DMMs go over USB-TMC, so --backend defaults to '' (the installed vendor
   VISA) here instead of '@py'. Pass --backend to override.

2. After the Pi reboots, the DMMs (which stay powered) can be left with a
   stuck USB link: every query times out until the DMM is power-cycled. So
   after RESET_AFTER failed connects in a row, SharedRmLink port-resets the
   instrument's USB device, which re-initialises its USB side the same way a
   replug would, then carries on retrying (resetting again every RESET_EVERY
   failures). Linux only; needs write access to /dev/bus/usb, which
   deploy/99-rs-instruments.rules grants.
"""

import glob
import os
import sys

import scpi_vi_logger as base

RESET_AFTER = 2     # failed connects in a row before the first USB reset
RESET_EVERY = 10    # after that, reset again every this many failures

SYSFS_USB = "/sys/bus/usb/devices"
USBDEVFS_RESET = 0x5514   # _IO('U', 20) from linux/usbdevice_fs.h


def _read(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return None


def find_usb_device(resource, sysfs=SYSFS_USB):
    """Map a USB0::vid::pid::serial::...::INSTR resource to its /dev/bus/usb
    node, by matching the serial the kernel already read at enumeration.

    Reading it from sysfs rather than asking the device matters here: the
    device this is used on is the one that has stopped answering.
    """
    parts = resource.split("::")
    if len(parts) < 4 or not parts[0].upper().startswith("USB"):
        raise ValueError(f"not a USB resource: {resource}")
    vid, pid, serial = int(parts[1], 16), int(parts[2], 16), parts[3]
    for dev in glob.glob(os.path.join(sysfs, "*")):
        v, p = _read(os.path.join(dev, "idVendor")), _read(os.path.join(dev, "idProduct"))
        if v is None or p is None or int(v, 16) != vid or int(p, 16) != pid:
            continue
        if _read(os.path.join(dev, "serial")) != serial:
            continue
        bus, num = _read(os.path.join(dev, "busnum")), _read(os.path.join(dev, "devnum"))
        return f"/dev/bus/usb/{int(bus):03d}/{int(num):03d}"
    raise LookupError(f"no USB device {parts[1]}:{parts[2]} serial {serial} (unplugged or off?)")


def usb_reset(resource):
    """Port-reset an instrument's USB device, like unplugging and replugging
    it. Returns the device node that was reset."""
    import fcntl   # Linux only, so imported here to keep Windows working
    node = find_usb_device(resource)
    fd = os.open(node, os.O_WRONLY)
    try:
        fcntl.ioctl(fd, USBDEVFS_RESET, 0)
    finally:
        os.close(fd)
    return node


class SharedRmLink(base.Link):
    """Link that USB-resets an instrument that keeps failing to connect."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.open_failures = 0

    def open(self):
        try:
            super().open()
        except Exception:
            self.open_failures += 1
            n = self.open_failures
            if n >= RESET_AFTER and (n - RESET_AFTER) % RESET_EVERY == 0:
                self._usb_reset(n)
            raise
        self.open_failures = 0

    def _usb_reset(self, n):
        if self.simulate or not self.resource.upper().startswith("USB"):
            return
        self.close()   # let go of the device before resetting it under us
        try:
            node = usb_reset(self.resource)
            base.log(f"usb reset {node} ({self.resource}) after {n} failed connects")
        except Exception as e:
            base.log(f"usb reset of {self.resource} failed: {type(e).__name__}: {e}")


def _argv_has_backend(argv):
    return any(a == "--backend" or a.startswith("--backend=") for a in argv)


def main():
    base.Link = SharedRmLink      # PsuWaveform builds its Link from this name too
    if not _argv_has_backend(sys.argv[1:]):
        sys.argv[1:1] = ["--backend", ""]

    return base.main()


if __name__ == "__main__":
    sys.exit(main())
