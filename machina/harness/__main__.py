"""Command line: one flight, or a campaign of identical-seed flights with its determinism check.

    python -m harness fly rth --seed 0
    python -m harness campaign ls-rth --runs 20 --jobs 4 --scenario rth --hold 5
    python -m harness campaign xp-hover --runs 4 --xplane --scenario hover --hold 5

Campaign records go to $MACHINA_CACHE/inav-fork/runs/campaign-<prefix>.{csv,json}: one row per
requested flight, missing or not, and the aggregate: how many completed, how many distinct
output and trajectory digests (1 means bit-identical), the spread, and the speed.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import platform
import subprocess
import time
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

from . import sitl, xplane


def _fly(a, tag: str) -> sitl.Flight:
    kw = dict(seed=a.seed, hold=a.hold, tlimit_s=a.tlimit, tag=tag)
    if a.xplane:
        return xplane.fly(a.scenario, **kw)
    return sitl.fly(a.scenario, rate_hz=a.rate, lockstep=not a.wall_clock, **kw)


def _git(*args: str) -> str:
    here = Path(__file__).resolve().parents[2]
    return subprocess.run(["git", "-C", str(here), *args], capture_output=True, text=True).stdout.strip()


def campaign(a) -> dict:
    tags = [f"{a.prefix}-{i}" for i in range(1, a.runs + 1)]
    started, w0 = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), time.perf_counter()

    def one(tag: str):
        try:
            return _fly(a, tag)
        except Exception as e:  # every requested flight leaves a row
            return f"harness-error: {e}"

    with ThreadPoolExecutor(a.jobs) as ex:
        flights = list(ex.map(one, tags))
    wall = time.perf_counter() - w0
    rows = []
    for tag, f in zip(tags, flights, strict=True):
        if isinstance(f, str):
            rows.append({"tag": tag, "result": f, "completed": False})
            continue
        s = f.summary()
        rows.append(
            {
                "tag": tag,
                "result": f.result,
                "completed": sitl.completed(f),
                "steps": f.steps,
                "sim_s": round(f.sim_s, 3),
                "wall_s": round(f.wall_s, 2),
                "speedup": s["speedup"],
                "armed_s": f.events.get("climb"),
                "hold_drift_m": s.get("hold_drift_m"),
                "land_dist_m": s.get("land_dist_m"),
                "rth_s": s.get("rth_s"),
                "hdg_err_deg": (s.get("att_err_deg") or {}).get("hdg_mean"),
                "clock_start_ns": f.clock_checks.get("clock_start_ns"),
                "steps_checked": f.clock_checks.get("steps_checked"),
                "outputs_sha256": f.outputs_sha256,
                "trajectory_sha256": f.trajectory_sha256,
            }
        )
    ok = [f for f in flights if not isinstance(f, str) and sitl.completed(f)]
    binary = Path(sitl.inav_bin()).resolve()
    agg = {
        "prefix": a.prefix,
        "backend": "xplane" if a.xplane else "machina",
        "clock": "wall_clock" if a.wall_clock else "lockstep",
        "scenario": a.scenario,
        "hold_s": a.hold,
        "seed": a.seed,
        "rate_hz": a.rate,
        "requested": a.runs,
        "jobs": a.jobs,
        "results": dict(Counter(r["result"] for r in rows)),
        "completed": len(ok),
        "steps_checked": int(sum(r.get("steps_checked") or 0 for r in rows)),
        "clock_start_ns": sorted({r.get("clock_start_ns") for r in rows if r.get("clock_start_ns") is not None}),
        "distinct_output_digests": len({f.outputs_sha256 for f in ok}),
        "distinct_trajectory_digests": len({f.trajectory_sha256 for f in ok}),
        "spread": sitl.spread(ok) if len(ok) >= 2 else None,
        "speedup_p5_p50_p95": np.percentile([r["speedup"] for r in rows if r.get("speedup")], [5, 50, 95])
        .round(3)
        .tolist()
        if any(r.get("speedup") for r in rows)
        else None,
        "sim_s_total": round(sum(r.get("sim_s") or 0 for r in rows), 1),
        "wall_s": round(wall, 1),
        "started_utc": started,
        "host": platform.node(),
        "cpu": _cpu(),
        "nproc": os.cpu_count(),
        "load1_end": round(os.getloadavg()[0], 2),
        "inav_bin": binary.name,
        "inav_bin_sha256_16": hashlib.sha256(binary.read_bytes()).hexdigest()[:16],
        "fork_commit": _git("rev-parse", "--short=9", "HEAD")
        + ("+dirty" if _git("status", "--porcelain", "--", "src", "machina/harness") else ""),
    }
    agg["host_x_realtime"] = round(agg["sim_s_total"] / wall, 2) if wall else None
    for key in ("hold_drift_m", "land_dist_m", "rth_s", "hdg_err_deg", "armed_s"):
        vals = [r[key] for r in rows if r.get("completed") and r.get(key) is not None]
        agg[f"{key}_min_p50_max"] = (
            [round(min(vals), 4), round(float(np.median(vals)), 4), round(max(vals), 4)] if vals else None
        )
    out = sitl.RUNS / f"campaign-{a.prefix}"
    with out.with_suffix(".csv").open("w", newline="") as fh:
        w = csv.DictWriter(fh, sorted({k for r in rows for k in r}), lineterminator="\n")
        w.writeheader()
        w.writerows(rows)
    out.with_suffix(".json").write_text(json.dumps(agg, indent=1) + "\n")
    return agg


def _cpu() -> str:
    try:
        with open("/proc/cpuinfo") as fh:
            return next((ln.split(":", 1)[1].strip() for ln in fh if ln.startswith("model name")), "")
    except OSError:
        return ""


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(
        prog="python -m harness", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("cmd", choices=["fly", "campaign"])
    ap.add_argument("prefix", nargs="?", default="", help="fly: the scenario; campaign: the run prefix")
    ap.add_argument("--scenario", default="hover", choices=["hover", "rth", "rcloss"])
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--hold", type=float, default=10.0)
    ap.add_argument("--rate", type=int, default=1000, help="sensor packets per second")
    ap.add_argument("--tlimit", type=float, default=200.0)
    ap.add_argument("--wall-clock", action="store_true", help="paced on the host's clock, not lockstep")
    ap.add_argument("--xplane", action="store_true", help="the X-Plane path in lockstep")
    ap.add_argument("--runs", type=int, default=10)
    ap.add_argument("--jobs", type=int, default=4)
    a = ap.parse_args(argv)
    if a.cmd == "fly":
        a.scenario = a.prefix or a.scenario
        print(json.dumps(_fly(a, "").summary(), indent=1, default=float))
    else:
        print(json.dumps(campaign(a), indent=1))


if __name__ == "__main__":
    main()
