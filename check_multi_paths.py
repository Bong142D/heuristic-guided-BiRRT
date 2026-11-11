#!/usr/bin/env python3
"""Check the multi-robot results with geometry of its own. It reads only the saved paths and the
original map, and shares no code with the planner.

For every successful run:
  - each path starts at its start and ends at its goal, and its saved length is right;
  - each path stays at least D/2 from the obstacles of the original map;
  - a pair of paths closer than D has a conflict zone, and a pair with a zone is closer than D;
  - reserve and separated: no path comes closer than D to the start or goal of another robot;
  - separated: no two paths come closer than D.

Exit code 0 = no violation. Example:
  python3 check_multi_paths.py ~/rrt_results/multi/quick
"""
import argparse
import glob
import os
import sys

import numpy as np

import multi_common as mc

TOL = 2e-3     # coordinates are saved with 4 decimals
ZONE_TOL = 0.01  # zones compare points every 0.1 m, which can overestimate a distance by a few mm


def point_segments(p, a, b):
    """Distance from points p (k, 2) to segments a-b (m, 2), as an array (k, m)."""
    ab = b - a
    denom = np.maximum((ab * ab).sum(axis=1), 1e-18)
    ap = p[:, None, :] - a[None, :, :]
    t = np.clip((ap * ab[None, :, :]).sum(axis=2) / denom, 0.0, 1.0)
    d = ap - t[:, :, None] * ab[None, :, :]
    return np.sqrt((d * d).sum(axis=2))


def cross(u, v):
    return u[..., 0] * v[..., 1] - u[..., 1] * v[..., 0]


def paths_min_distance(p, q):
    """Exact smallest distance between two polylines."""
    a0, a1 = p[:-1], p[1:]
    b0, b1 = q[:-1], q[1:]
    d = min(point_segments(p, b0, b1).min(), point_segments(q, a0, a1).min())
    # segments that properly cross have distance 0, which the end-point distances do not show
    da = a1 - a0
    db = b1 - b0
    ab = b0[None, :, :] - a0[:, None, :]
    denom = cross(da[:, None, :], db[None, :, :])
    with np.errstate(divide="ignore", invalid="ignore"):
        t = cross(ab, db[None, :, :]) / denom
        u = cross(ab, da[:, None, :]) / denom
        crossing = np.any((denom != 0) & (t >= 0) & (t <= 1) & (u >= 0) & (u <= 1))
    if crossing:
        return 0.0
    return float(d)


def segment_squares(a, b, lo, hi):
    """Distance from one segment to axis-aligned squares (k, 2)."""
    d = b - a
    t0 = np.zeros(len(lo))
    t1 = np.ones(len(lo))
    hit = np.ones(len(lo), dtype=bool)
    for i in range(2):
        if abs(d[i]) < 1e-12:
            hit &= (a[i] >= lo[:, i]) & (a[i] <= hi[:, i])
        else:
            ta = (lo[:, i] - a[i]) / d[i]
            tb = (hi[:, i] - a[i]) / d[i]
            t0 = np.maximum(t0, np.minimum(ta, tb))
            t1 = np.minimum(t1, np.maximum(ta, tb))
    hit &= t0 <= t1

    def to_square(p):
        g = np.maximum(np.maximum(lo - p, 0.0), p - hi)
        return np.hypot(g[:, 0], g[:, 1])

    corners = np.stack([lo, hi, np.stack([lo[:, 0], hi[:, 1]], 1), np.stack([hi[:, 0], lo[:, 1]], 1)], 1)
    seg_a, seg_b = a[None, :], b[None, :]
    corner_d = point_segments(corners.reshape(-1, 2), seg_a, seg_b).reshape(len(lo), 4).min(axis=1)
    out = np.minimum(np.minimum(to_square(a), to_square(b)), corner_d)
    out[hit] = 0.0
    return out


def clearance(path, grid, reach):
    """Smallest distance from a path to occupied cells; reach is returned when none is that close."""
    best = reach
    for a, b in zip(path[:-1], path[1:]):
        lo = np.floor((np.minimum(a, b) - reach - mc.ORIGIN) / mc.RES).astype(int)
        hi = np.floor((np.maximum(a, b) + reach - mc.ORIGIN) / mc.RES).astype(int)
        lo = np.clip(lo, 0, mc.GRID - 1)
        hi = np.clip(hi, 0, mc.GRID - 1)
        idx = np.argwhere(grid[lo[0]:hi[0] + 1, lo[1]:hi[1] + 1])
        if len(idx) == 0:
            continue
        corner = mc.ORIGIN + (idx + lo) * mc.RES
        best = min(best, float(segment_squares(a, b, corner, corner + mc.RES).min()))
    return best


def check_run(run, sc, grid, D):
    problems = []
    robots = run["robots"]
    paths = [np.array(r["path"], dtype=float) if r.get("path") else None for r in robots]
    if any(p is None for p in paths):
        return ["paths not saved"], None
    starts = np.array(sc["starts"], dtype=float)
    goals = np.array(sc["goals"], dtype=float)
    min_clear = np.inf
    for i, p in enumerate(paths):
        if np.hypot(*(p[0] - starts[i])) > TOL:
            problems.append("robot {} does not begin at its start".format(i))
        if np.hypot(*(p[-1] - goals[i])) > TOL:
            problems.append("robot {} does not end at its goal".format(i))
        length = np.hypot(*(p[1:] - p[:-1]).T).sum()
        if abs(length - robots[i]["length"]) > 0.01 + 1e-4 * len(p):
            problems.append("robot {}: saved length {:.3f}, measured {:.3f}".format(i, robots[i]["length"], length))
        c = clearance(p, grid, 0.5 * D + 0.5)
        min_clear = min(min_clear, c)
        if c < 0.5 * D - TOL:
            problems.append("robot {}: {:.3f} m from an obstacle, needs {:.3f}".format(i, c, 0.5 * D))

    n = len(paths)
    has_zone = {(z["a"], z["b"]) for z in run.get("zones", [])}
    min_sep = np.inf
    mode = run["method"].split("+")[1]
    for i in range(n):
        for j in range(i + 1, n):
            d = paths_min_distance(paths[i], paths[j])
            min_sep = min(min_sep, d)
            if d < D - ZONE_TOL and (i, j) not in has_zone:
                problems.append("robots {} {}: {:.3f} m apart but no zone".format(i, j, d))
            if d >= D + TOL and (i, j) in has_zone:
                problems.append("robots {} {}: {:.3f} m apart but a zone".format(i, j, d))
            if mode == "separated" and d < D - TOL:
                problems.append("robots {} {}: {:.3f} m apart in separated mode".format(i, j, d))
    if mode in ("reserve", "separated") and run.get("reserve_dropped", 0) == 0:
        for i in range(n):
            others = [k for k in range(n) if k != i]
            ends = np.concatenate([starts[others], goals[others]])
            d = point_segments(ends, paths[i][:-1], paths[i][1:]).min()
            if d < D - TOL:
                problems.append("robot {}: {:.3f} m from the endpoint of another robot".format(i, d))
    return problems, {"clearance": min_clear, "min_sep": min_sep}


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("results", help="folder written by run_multi_experiments.py")
    p.add_argument("--scenarios", default=None, help="default: the sibling folder 'scenarios' of the results")
    p.add_argument("--dataset-root", default="/home/huy/rrt/dataset_maps_200x200")
    p.add_argument("--every", type=int, default=1, help="check only every k-th successful run")
    args = p.parse_args()

    scen_root = args.scenarios or os.path.join(os.path.dirname(os.path.abspath(args.results.rstrip("/"))), "scenarios")
    checked = bad = skipped = 0
    worst_clear, examples = np.inf, []
    for result in sorted(glob.glob(os.path.join(args.results, "*", "map_*_result.json"))):
        env = os.path.basename(os.path.dirname(result))
        name = os.path.basename(result).replace("_result.json", "")
        data = mc.load_json(result)
        scen = {s["id"]: s for s in mc.load_json(os.path.join(scen_root, env, name + ".json"))["scenarios"]}
        grid = mc.load_occupancy(os.path.join(args.dataset_root, env, name + ".pcd"))
        count = 0
        for run in data["runs"]:
            if not run["success"]:
                continue
            count += 1
            if (count - 1) % args.every:
                continue
            problems, info = check_run(run, scen[run["scenario"]], grid, data["D"])
            if info is None:
                skipped += 1
                continue
            checked += 1
            worst_clear = min(worst_clear, info["clearance"])
            if problems:
                bad += 1
                if len(examples) < 20:
                    examples.append("{}/{} scenario {} trial {} {}: {}".format(
                        env, name, run["scenario"], run["trial"], run["method"], "; ".join(problems[:3])))
    for e in examples:
        print("  " + e)
    print("checked {} runs, {} with violations, {} skipped because their paths were not saved; closest approach to an obstacle {:.3f} m".format(
        checked, bad, skipped, worst_clear))
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
