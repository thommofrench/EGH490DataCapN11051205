# EGH490 DataCap

Long-run voltage logging for the EGH490 cell tests. A Raspberry Pi reads two
Rohde & Schwarz DMMs (cell voltage and Hall voltage) into one CSV, drives an
NGE103B PSU, and pushes the CSV to this repo every 5 minutes.

## Scripts

### `scpi_vi_logger.py`
The main logger (PyVISA).
- Reads DC voltage from two DMMs, the cell and the Hall sensor, into one CSV.
  Each DMM has its own link, so if one drops out the other keeps logging.
- Drives the NGE103B PSU with a square-wave current (AC test, the default) or a
  constant current (`--dc`).
- Built for unattended multi-week runs: it reconnects by itself, appends to the
  CSV and fsyncs every row, and commits and pushes the CSV to GitHub every
  `--git-interval` seconds.
- `--simulate --no-psu` runs it without any hardware for a quick self-test.

### `scpi_vi_logger_usb.py`
A thin wrapper around `scpi_vi_logger.py` for DMMs connected over USB-B. This is
the script the Pi runs. It uses the same command line, and the only changes are:
- it uses the vendor VISA backend by default
- it port-resets a DMM's USB link when the DMM gets stuck after a Pi reboot

## Usage

```bash
pip install pyvisa pyvisa-py

python scpi_vi_logger_usb.py --list     # show connected instruments
python scpi_vi_logger_usb.py            # AC test: both DMMs + square-wave PSU
python scpi_vi_logger_usb.py --dc       # DC test: both DMMs + constant-current PSU
python scpi_vi_logger_usb.py --no-psu   # DMMs only
```

Instrument addresses and the test name (which sets the CSV filename) are
constants at the top of `scpi_vi_logger.py`. Run with `--help` for all options.

## Other files

- `deploy/`: systemd services for the AC and DC tests, a udev rule for USB
  access to the instruments, and a script that sets the Pi's clock from GitHub
  at startup
- `*_log.csv`: logged data from the dummy and prelim test runs
