# CAN-scheduling-simulation

Simulation of scheduling different messages on a CAN bus, plus a Python tool
that summarises delivery results from the simulation log.

## Requirements on Linux

- GCC with C23 support (GCC 14 or newer)
- Meson and Ninja
- Python 3 with `venv`

On Debian/Ubuntu:

```sh
sudo apt install build-essential meson ninja-build python3-venv
```

## Build and run the simulation

From the repository root:

```sh
meson setup builddir
meson compile -C builddir
./builddir/simulation > result.txt
```

A full run takes about 30 seconds and writes one line per transmitted frame
to `result.txt`.

## Analyse the results

```sh
cd analysis
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python analyze_can_log.py
```

This prints on-time, late and lost frame statistics and writes charts to
`analysis/figures/`. By default it reads `result.txt` (or `results.txt`) from
the current directory or the repository root.

The analyser can also build and run the simulator itself:

```sh
.venv/bin/python analyze_can_log.py --run
```

Add `--sim-ms 1000` for a shorter run.
