# CAN log analyser

Parses the simulator's stdout log (`result.txt`) and reports delivery quality:

- on-time, late and **lost** frames (total and per CAN ID),
- on-time percentage and overall delivery success rate,
- average/maximum time past deadline,
- charts: outcomes by CAN ID (counts + percentages), outcomes over time, and
  late times per ID (only when late frames exist).

## How "lost" is computed

The simulator prints a line only when a frame finishes transmission:

```
Delivered id:0x010,ts:10.162000
Late id:0x010,ts:...,timeLate:0.420000
```

It does not print dropped frames, so lost frames are inferred from the
generation periods in `simulation.c`:

| CAN ID | Message | Period |
|---|---|---|
| `0x010` | engine temperature | 10 ms |
| `0x020` | engine pressure | 20 ms |
| `0x100` | power voltage | 100 ms |
| `0x200` | navigation GPS | 200 ms |
| `0x300` | diagnosis status | 500 ms |

Generation starts at t=0, so the expected count per ID is
`floor(last_timestamp_for_that_ID / period) + 1`, and
`lost = expected - observed`. If the log is truncated or timestamps are
unreliable, pass the real simulation end time with `--duration-ms`.

## Setup (venv + requirements.txt)

From the repository root:

```sh
cd analysis
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

## Run

The script looks for `result.txt` (or `results.txt`) in the current directory
and in the repository root. It writes charts to `analysis/figures/`.

```sh
# analyse an existing log
.venv/bin/python analyze_can_log.py

# explicit log file, 5-second bins for the time chart, custom output directory
.venv/bin/python analyze_can_log.py ../result.txt --bin-ms 5000 -o ./figures

# build + run the simulator yourself, then analyse the result
.venv/bin/python analyze_can_log.py --run

# short run: stop at 1000 ms of simulated time, save log to a named file
.venv/bin/python analyze_can_log.py --run --sim-ms 1000 --log-out ./short_run.log
```

`--run` builds with `meson setup` / `meson compile` in `builddir`, runs the
simulator, and saves stdout to `analysis/sim_run.log` unless `--log-out` is
given. A full run takes roughly 30 seconds and produces ~51 MB of log.
With `--sim-ms`, the process is stopped as soon as that simulated time is
reached. Frames generated just before the cut may still be in flight, so the
analyser infers each ID's expected count from that ID's last observed frame
rather than from `--sim-ms` (otherwise they would show up as false "lost").
