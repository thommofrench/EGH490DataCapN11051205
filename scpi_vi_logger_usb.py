#!/usr/bin/env python3
"""
scpi_vi_logger.py, but with the DMM on its USB-B port instead of Ethernet.

Everything else is unchanged and comes straight from scpi_vi_logger.py: the CSV
logging, the GitHub auto-push, the NGE103B waveform, reconnect handling and
clean shutdown. This file only swaps how the DMM is reached, so a fix made in
scpi_vi_logger.py applies to both.

Same command line as scpi_vi_logger.py:
    python scpi_vi_logger_usb.py                 # DMM on USB, PSU on USB
    python scpi_vi_logger_usb.py --no-psu        # DMM only
    python scpi_vi_logger_usb.py --list          # show connected USB instruments
    python scpi_vi_logger_usb.py -r USB0::...    # override the DMM address

First-time setup: plug the DMM in, run --list, and paste its USB0::... string
into DMM_USB_RESOURCE below.

What differs from the Ethernet script
-------------------------------------
1. The DMM goes over USB-TMC, which needs the installed vendor VISA (R&S VISA),
   so --backend defaults to '' here instead of '@py'. Pass --backend to override.

2. Both instruments are now on the same VISA library, and pyvisa shares ONE
   resource manager per library; closing it closes every session opened through
   it. The stock Link.close() closes the manager, which was harmless when the
   DMM used pyvisa-py, but here a DMM reconnect (or the DMM closing at
   shutdown) would kill the PSU's session, and at exit the PSU outputs could
   not be switched off. SharedRmLink closes only its own instrument session.
"""

import argparse
import sys

import scpi_vi_logger as base

# The DMM's USB address, e.g. "USB0::0x0AAD::0x0xxx::<serial>::0::INSTR".
# Fill in from `--list` once the DMM is plugged in and switched on.
DMM_USB_RESOURCE = ""


class SharedRmLink(base.Link):
    """Link that closes its own VISA session but leaves the shared resource
    manager (and so every other instrument's session) alone."""

    def close(self):
        try:
            if self.inst is not None:
                self.inst.close()
        except Exception:
            pass
        self.inst = None
        self.rm = None


def _argv_has_backend(argv):
    return any(a == "--backend" or a.startswith("--backend=") for a in argv)


def main():
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("-r", "--resource")
    pre.add_argument("--list", action="store_true")
    pre.add_argument("--simulate", action="store_true")
    pre.add_argument("-h", "--help", action="store_true")
    known, _ = pre.parse_known_args()

    if not known.resource and not DMM_USB_RESOURCE and not (
            known.list or known.simulate or known.help):
        base.log("DMM_USB_RESOURCE is not set in scpi_vi_logger_usb.py.")
        base.log("Plug the DMM in via USB-B, switch it on, and paste its USB0::... "
                 "address into that constant. Instruments found right now:")
        try:
            base.list_resources("")
        except Exception as e:
            base.log(f"could not query VISA: {e}")
        return 2

    base.RESOURCE = DMM_USB_RESOURCE
    base.Link = SharedRmLink      # PsuWaveform builds its Link from this name too
    if not _argv_has_backend(sys.argv[1:]):
        sys.argv[1:1] = ["--backend", ""]

    return base.main()


if __name__ == "__main__":
    sys.exit(main())
