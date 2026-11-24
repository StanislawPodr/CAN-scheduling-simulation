#!/usr/bin/env python3
"""Analyse the CAN scheduling simulator log.

Expected input protocol (stdout of ``./builddir/can-simulation``), one frame per line::

    Delivered id:0x010,ts:10.162000
    Late id:0x010,ts:...,timeLate:0.420000

With ``--run`` the script first builds the C simulator with Meson and runs it,
optionally for a limited simulated time (``--sim-ms``), saving stdout to a log.
Without ``--run`` it analyses an existing log; the default is ``result.txt``
(or ``results.txt``) in the current directory or the repository root.

A frame is "on time" if it was printed as ``Delivered`` and "late" if it was
printed as ``Late`` (``timeLate`` is how far past its deadline it arrived).
Because the simulator does not print dropped frames, "lost" is derived from
the generation periods in ``simulation.c``: for every CAN ID, generation
starts at t = 0 and repeats every PERIOD ms, so the number of frames that
should have been generated is ``floor(last_frame_ts / PERIOD) + 1``.
Lost = expected - observed.  Use ``--duration-ms`` to supply the simulation
end time explicitly instead of inferring it per ID from the log.
"""

from __future__ import annotations

import argparse
import math
import re
import shutil
import subprocess
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # headless: write PNG files instead of opening a window
import matplotlib.pyplot as plt  # noqa: E402

# ---------------------------------------------------------------------------
# Protocol / simulation constants (mirror CAN_info.h and simulation.c)
# ---------------------------------------------------------------------------

# CAN ID -> (name, generation period in ms)
CAN_IDS: dict[int, tuple[str, float]] = {
    0x010: ("ENGINE_TEMPERATURE", 10.0),
    0x020: ("ENGINE_PRESSURE", 20.0),
    0x100: ("POWER_ACC_VOLTAGE", 100.0),
    0x200: ("NAV_GPS_POS", 200.0),
    0x300: ("DIAGNOSIS_SYSTEM_STAT", 500.0),
}

SHORT_NAMES: dict[int, str] = {
    0x010: "ENGINE\nTEMP",
    0x020: "ENGINE\nPRESS",
    0x100: "POWER\nVOLTAGE",
    0x200: "NAV\nGPS",
    0x300: "DIAG\nSTATUS",
}

STATUSES = ("on_time", "late", "lost")
COLORS = {"on_time": "#2ca02c", "late": "#ff7f0e", "lost": "#d62728"}
LABELS = {"on_time": "On time", "late": "Late", "lost": "Lost (not delivered)"}

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_BUILDDIR = REPO_ROOT / "builddir"
DEFAULT_RUN_LOG = Path(__file__).resolve().parent / "sim_run.log"
DEFAULT_LOG_NAMES = ("result.txt", "results.txt")

LINE_RE = re.compile(
    r"^(?P<status>Delivered|Late)\s+"
    r"id:(?P<id>0x[0-9a-fA-F]+),"
    r"ts:(?P<ts>\d+(?:\.\d+)?)"
    r"(?:,timeLate:(?P<late>\d+(?:\.\d+)?))?"
    r"\s*$"
)

# ---------------------------------------------------------------------------
# Data model
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Frame:
    can_id: int
    ts_ms: float
    late_ms: float
    is_late: bool


@dataclass(frozen=True)
class Event:
    """One expected generation slot, tagged with how it ended up."""

    ts_ms: float
    status: str


@dataclass
class IdStats:
    name: str
    period_ms: float
    expected: int = 0
    on_time: int = 0
    late: int = 0
    lost: int = 0
    late_total_ms: float = 0.0
    late_max_ms: float = 0.0

    @property
    def observed(self) -> int:
        return self.on_time + self.late

    @property
    def avg_late_ms(self) -> float:
        return self.late_total_ms / self.late if self.late else 0.0


@dataclass
class Report:
    stats: dict[int, IdStats] = field(default_factory=dict)
    events: list[Event] = field(default_factory=list)
    unknown_lines: int = 0
    first_ts_ms: float = 0.0
    last_ts_ms: float = 0.0

    @property
    def expected(self) -> int:
        return sum(s.expected for s in self.stats.values())

    @property
    def on_time(self) -> int:
        return sum(s.on_time for s in self.stats.values())

    @property
    def late(self) -> int:
        return sum(s.late for s in self.stats.values())

    @property
    def lost(self) -> int:
        return sum(s.lost for s in self.stats.values())

    @property
    def observed(self) -> int:
        return self.on_time + self.late

    @property
    def late_total_ms(self) -> float:
        return sum(s.late_total_ms for s in self.stats.values())

    @property
    def late_max_ms(self) -> float:
        return max((s.late_max_ms for s in self.stats.values()), default=0.0)


# ---------------------------------------------------------------------------
# Parsing and aggregation
# ---------------------------------------------------------------------------


def read_log(path: Path) -> tuple[list[Frame], int]:
    frames: list[Frame] = []
    unknown = 0
    with path.open("r", encoding="utf-8", errors="replace") as log:
        for line in log:
            line = line.strip()
            if not line:
                continue
            match = LINE_RE.match(line)
            if not match:
                unknown += 1
                continue
            is_late = match.group("status") == "Late"
            frames.append(
                Frame(
                    can_id=int(match.group("id"), 16),
                    ts_ms=float(match.group("ts")),
                    late_ms=float(match.group("late") or 0.0),
                    is_late=is_late,
                )
            )
    return frames, unknown


def ensure_simulator(builddir: Path) -> Path:
    """Build the C simulator with Meson if needed and return the binary path."""
    if shutil.which("meson") is None:
        raise RuntimeError("meson is required to build the simulator (--run)")

    if not (builddir / "build.ninja").is_file():
        subprocess.run(["meson", "setup", str(builddir)], cwd=REPO_ROOT, check=True)
    subprocess.run(["meson", "compile", "-C", str(builddir)], cwd=REPO_ROOT, check=True)

    binary = builddir / "can-simulation"
    if not binary.is_file():
        raise RuntimeError(f"simulator binary not found: {binary}")
    return binary


def run_simulator(binary: Path, log_path: Path, sim_ms: float | None) -> Path:
    """Run the simulator, tee its stdout to ``log_path``, optionally stop at sim_ms."""
    log_path.parent.mkdir(parents=True, exist_ok=True)
    stopped_early = False

    with log_path.open("w", encoding="utf-8") as out:
        proc = subprocess.Popen(
            [str(binary)],
            cwd=REPO_ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        assert proc.stdout is not None
        for line in proc.stdout:
            out.write(line)
            if sim_ms is not None:
                match = LINE_RE.match(line.strip())
                if match and float(match.group("ts")) >= sim_ms:
                    stopped_early = True
                    break
        out.flush()

        if stopped_early:
            proc.terminate()
        try:
            _, stderr = proc.communicate(timeout=15)
        except subprocess.TimeoutExpired:
            proc.kill()
            _, stderr = proc.communicate()
        if stderr.strip():
            print("Simulator stderr:", file=sys.stderr)
            print(stderr.rstrip(), file=sys.stderr)
        if not stopped_early and proc.returncode != 0:
            raise RuntimeError(f"simulator exited with status {proc.returncode}")

    return log_path


def build_report(frames: list[Frame], unknown: int, duration_ms: float | None) -> Report:
    by_id: dict[int, list[Frame]] = defaultdict(list)
    for frame in frames:
        by_id[frame.can_id].append(frame)

    report = Report(unknown_lines=unknown)
    all_ts = [f.ts_ms for f in frames]
    if all_ts:
        report.first_ts_ms = min(all_ts)
        report.last_ts_ms = max(all_ts)

    # Known IDs first, then any unexpected IDs that still appear in the log.
    ids = list(CAN_IDS) + sorted(set(by_id) - set(CAN_IDS))

    for can_id in ids:
        name, period = CAN_IDS.get(can_id, (f"UNKNOWN_0x{can_id:03X}", 0.0))
        id_frames = sorted(by_id.get(can_id, []), key=lambda f: f.ts_ms)
        seen = len(id_frames)

        if period > 0:
            end_ms = duration_ms if duration_ms is not None else (id_frames[-1].ts_ms if seen else 0.0)
            expected = max(int(math.floor(end_ms / period + 1e-9)) + 1, seen) if seen or duration_ms else 0
        else:
            # No known generation period: only count frames that actually appeared.
            expected = seen

        stats = IdStats(name=name, period_ms=period, expected=expected)

        for frame in id_frames:
            if frame.is_late:
                stats.late += 1
                stats.late_total_ms += frame.late_ms
                stats.late_max_ms = max(stats.late_max_ms, frame.late_ms)
                report.events.append(Event(frame.ts_ms, "late"))
            else:
                stats.on_time += 1
                report.events.append(Event(frame.ts_ms, "on_time"))

        # The first `seen` generation slots are assumed to be the observed
        # frames (the simulator keeps per-ID FIFO order). Any remaining slots
        # were generated but never delivered.
        stats.lost = expected - seen
        for slot in range(seen, expected):
            report.events.append(Event(slot * period, "lost"))

        report.stats[can_id] = stats

    report.events.sort(key=lambda e: e.ts_ms)
    return report


# ---------------------------------------------------------------------------
# Console report
# ---------------------------------------------------------------------------


def pct(part: int, whole: int) -> float:
    return 100.0 * part / whole if whole else 0.0


def fmt_int(value: int) -> str:
    return f"{value:,}"


def print_report(report: Report, log_path: Path) -> None:
    duration_s = (report.last_ts_ms - report.first_ts_ms) / 1000.0
    print("=" * 100)
    print(f"CAN message delivery report   log: {log_path}")
    print(f"Observed window: {report.first_ts_ms:.3f} .. {report.last_ts_ms:.3f} ms ({duration_s:,.2f} s)")
    print("=" * 100)

    print("\nOVERALL")
    print(f"  Generated / expected frames : {fmt_int(report.expected)}")
    print(f"  All observed frames         : {fmt_int(report.observed)}")
    print(f"  Delivered on time           : {fmt_int(report.on_time)}  ({pct(report.on_time, report.expected):6.2f}% of expected)")
    print(f"  Delivered late              : {fmt_int(report.late)}  ({pct(report.late, report.expected):6.2f}% of expected)")
    print(f"  Lost (never delivered)      : {fmt_int(report.lost)}  ({pct(report.lost, report.expected):6.2f}% of expected)")
    print(f"  Delivery success (on+late)  : {pct(report.observed, report.expected):6.2f}%")
    if report.late:
        print(f"  Average late time           : {report.late_total_ms / report.late:8.3f} ms  (max {report.late_max_ms:.3f} ms)")
    else:
        print("  Average late time           : 0.000 ms  (no late frames)")

    print("\nPER CAN ID")
    header = (
        f"  {'ID':<7} {'Name':<21} {'Period':>8} {'Expected':>11} {'On time':>11} "
        f"{'Late':>8} {'Lost':>8} {'On-time %':>10} {'Avg late':>10}"
    )
    print(header)
    print("  " + "-" * (len(header) - 2))
    for can_id, stats in sorted(report.stats.items()):
        avg = f"{stats.avg_late_ms:.3f} ms" if stats.late else "-"
        period = f"{stats.period_ms:g} ms" if stats.period_ms else "unknown"
        print(
            f"  0x{can_id:03X}  {stats.name:<21} {period:>8} "
            f"{fmt_int(stats.expected):>11} {fmt_int(stats.on_time):>11} "
            f"{fmt_int(stats.late):>8} {fmt_int(stats.lost):>8} "
            f"{pct(stats.on_time, stats.expected):9.2f}% {avg:>10}"
        )

    if report.unknown_lines:
        print(f"\n  Warning: skipped {fmt_int(report.unknown_lines)} unrecognised log line(s).")


# ---------------------------------------------------------------------------
# Charts
# ---------------------------------------------------------------------------


def compact(value: int) -> str:
    if value == 0:
        return "0"
    if abs(value) >= 1_000_000:
        return f"{value / 1_000_000:.1f}M"
    if abs(value) >= 1_000:
        return f"{value / 1_000:.0f}k"
    return str(value)


def plot_by_id(report: Report, outdir: Path) -> Path:
    ids = sorted(report.stats)
    labels = [f"0x{i:03X}\n{SHORT_NAMES.get(i, 'UNKNOWN')}" for i in ids]
    values = {s: [getattr(report.stats[i], s) for i in ids] for s in STATUSES}

    fig, (ax_counts, ax_pct) = plt.subplots(1, 2, figsize=(14, 5.8))

    width = 0.26
    for idx, status in enumerate(STATUSES):
        offset = (idx - 1) * width
        bars = ax_counts.bar([x + offset for x in range(len(ids))], values[status], width,
                             color=COLORS[status], label=LABELS[status])
        for bar, value in zip(bars, values[status]):
            ax_counts.text(bar.get_x() + bar.get_width() / 2, bar.get_height(),
                           compact(value), ha="center", va="bottom", fontsize=8)
    ax_counts.set_title("Frame outcomes by CAN ID (counts)")
    ax_counts.set_ylabel("Number of frames")
    ax_counts.set_xticks(range(len(ids)), labels)
    ax_counts.legend()
    ax_counts.grid(axis="y", alpha=0.3)
    ax_counts.set_axisbelow(True)

    totals = [max(report.stats[i].expected, 1) for i in ids]
    bottoms = [0.0] * len(ids)
    for status in STATUSES:
        shares = [100.0 * values[status][i] / totals[i] for i in range(len(ids))]
        bars = ax_pct.bar(range(len(ids)), shares, 0.6, bottom=bottoms,
                          color=COLORS[status], label=LABELS[status])
        for bar, share in zip(bars, shares):
            if share >= 4:
                ax_pct.text(bar.get_x() + bar.get_width() / 2, bar.get_y() + bar.get_height() / 2,
                            f"{share:.1f}%", ha="center", va="center", fontsize=8, color="white")
        bottoms = [b + s for b, s in zip(bottoms, shares)]
    ax_pct.set_title("Frame outcomes by CAN ID (share of generated frames)")
    ax_pct.set_ylabel("Share of generated frames (%)")
    ax_pct.set_ylim(0, 100)
    ax_pct.set_xticks(range(len(ids)), labels)
    ax_pct.legend()
    ax_pct.grid(axis="y", alpha=0.3)
    ax_pct.set_axisbelow(True)

    fig.suptitle("Delivery results per message type", fontsize=14, fontweight="bold")
    fig.tight_layout()
    path = outdir / "by_can_id.png"
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return path


def plot_over_time(report: Report, bin_ms: int, outdir: Path) -> Path | None:
    if not report.events:
        return None

    end_ms = max(e.ts_ms for e in report.events)
    n_bins = int(end_ms // bin_ms) + 1
    counts = {status: [0] * n_bins for status in STATUSES}
    for event in report.events:
        idx = min(int(event.ts_ms // bin_ms), n_bins - 1)
        counts[event.status][idx] += 1

    centers_s = [(i + 0.5) * bin_ms / 1000.0 for i in range(n_bins)]
    fig, ax = plt.subplots(figsize=(13, 5.5))
    ax.stackplot(centers_s, *(counts[s] for s in STATUSES),
                 labels=[LABELS[s] for s in STATUSES],
                 colors=[COLORS[s] for s in STATUSES], alpha=0.8)
    ax.set_title(f"Delivered, late and lost frames over time ({bin_ms} ms bins)")
    ax.set_xlabel("Simulation time (s)")
    ax.set_ylabel(f"Frames per {bin_ms} ms bin")
    ax.set_xlim(0, end_ms / 1000.0)
    ax.legend(loc="upper right")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    path = outdir / "over_time.png"
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return path


def plot_late_times(report: Report, outdir: Path) -> Path | None:
    late_ids = [i for i, s in sorted(report.stats.items()) if s.late]
    if not late_ids:
        return None

    labels = [f"0x{i:03X}\n{SHORT_NAMES.get(i, 'UNKNOWN')}" for i in late_ids]
    avgs = [report.stats[i].avg_late_ms for i in late_ids]
    maxes = [report.stats[i].late_max_ms for i in late_ids]

    fig, ax = plt.subplots(figsize=(9, 5))
    bars = ax.bar(range(len(late_ids)), avgs, color=COLORS["late"], label="Average late time")
    ax.scatter(range(len(late_ids)), maxes, color=COLORS["lost"], marker="_", s=250,
               linewidths=2, zorder=3, label="Maximum late time")
    for bar, avg in zip(bars, avgs):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height(), f"{avg:.3f} ms",
                ha="center", va="bottom", fontsize=9)
    ax.set_title("How late were the late frames?")
    ax.set_ylabel("Time past deadline (ms)")
    ax.set_xticks(range(len(late_ids)), labels)
    ax.legend()
    ax.grid(axis="y", alpha=0.3)
    ax.set_axisbelow(True)
    fig.tight_layout()
    path = outdir / "late_times.png"
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return path


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("logfile", nargs="?", type=Path,
                        help="existing simulator log (default: result.txt or results.txt)")
    parser.add_argument("--run", "--simulate", action="store_true",
                        help="build and run the C simulator, then analyse its output")
    parser.add_argument("--sim-ms", type=float, default=None,
                        help="with --run: stop once this simulated time (ms) is reached (default: full run)")
    parser.add_argument("--log-out", type=Path, default=None,
                        help="with --run: where to save simulator stdout (default: analysis/sim_run.log)")
    parser.add_argument("--builddir", type=Path, default=DEFAULT_BUILDDIR,
                        help="Meson build directory used by --run")
    parser.add_argument("-o", "--outdir", type=Path, default=Path(__file__).resolve().parent / "figures",
                        help="directory for generated PNG charts")
    parser.add_argument("--bin-ms", type=int, default=1000, help="bin width for the over-time chart (default: 1000)")
    parser.add_argument("--duration-ms", type=float, default=None,
                        help="simulation end time in ms; overrides the per-ID inference from last frame timestamps")
    args = parser.parse_args()

    if args.sim_ms is not None and not args.run:
        parser.error("--sim-ms only makes sense with --run")
    if args.log_out is not None and not args.run:
        parser.error("--log-out only makes sense with --run")
    if args.bin_ms <= 0:
        parser.error("--bin-ms must be positive")
    if args.duration_ms is not None and args.duration_ms < 0:
        parser.error("--duration-ms cannot be negative")
    if args.sim_ms is not None and args.sim_ms <= 0:
        parser.error("--sim-ms must be positive")

    if args.run:
        log_path = args.log_out or DEFAULT_RUN_LOG
        print(f"Building simulator in {args.builddir} ...", flush=True)
        try:
            binary = ensure_simulator(args.builddir)
            print(f"Running {binary}" + (f" until {args.sim_ms:g} ms" if args.sim_ms else " (full run)") + " ...",
                  flush=True)
            run_simulator(binary, log_path, args.sim_ms)
        except (RuntimeError, subprocess.CalledProcessError, OSError) as exc:
            parser.error(f"could not run simulator: {exc}")
        print(f"Simulator output saved to {log_path}\n", flush=True)
    else:
        log_path = args.logfile
        if log_path is None:
            candidates = [Path(name) for name in DEFAULT_LOG_NAMES]
            candidates += [REPO_ROOT / name for name in DEFAULT_LOG_NAMES]
            log_path = next((p for p in candidates if p.is_file()), candidates[0])

        if not log_path.is_file():
            parser.error(f"log file not found: {log_path} (use --run to generate one)")

    frames, unknown = read_log(log_path)
    if not frames:
        print(f"No Delivered/Late lines found in {log_path}", file=sys.stderr)
        return 2

    report = build_report(frames, unknown, args.duration_ms)
    print_report(report, log_path)

    args.outdir.mkdir(parents=True, exist_ok=True)
    outputs = [plot_by_id(report, args.outdir)]
    over_time = plot_over_time(report, args.bin_ms, args.outdir)
    if over_time:
        outputs.append(over_time)
    late_plot = plot_late_times(report, args.outdir)
    if late_plot:
        outputs.append(late_plot)

    print("\nCHARTS")
    for path in outputs:
        print(f"  {path}")
    if not late_plot:
        print("  (no late-time chart: no late frames in this run)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
