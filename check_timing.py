#!/usr/bin/env python3
"""Find runs whose search time was disturbed (machine sleep, WSL/host stalls, wall-clock steps).

A run is flagged when its time per iteration is over FACTOR times the median time per
iteration of the runs of the same planner on the same map that have a similar number of
iterations (within a factor of 2), and the extra time is large enough to shift that planner's
mean on the map: over MIN_EXCESS and over MIN_SHARE of its total time there. Comparing with
runs of similar length matters: time per iteration grows with the tree, so a 1.5 s stall inside
a 50k-iteration run is only a few times the map-wide median and would go unnoticed.
Its time per node must also be over FACTOR times the median of the runs with a similar number of
nodes. Some runs grow a tree 2-3 times larger than usual for their length (RRT-FConnect without
exploration does it often) and are slow per iteration for real; a stall makes a run slow per node too.

With --delete the result files of the flagged maps are removed, so re-running the same
run_dataset_experiments.py / sweep_fconnect.py command re-runs exactly those maps (finished
maps are skipped).

Usage:
  python3 check_timing.py ~/rrt_results/full_run_2 [more folders] [--delete]
"""
import argparse
import bisect
import glob
import json
import os
import statistics

FACTOR = 3.0
MIN_EXCESS = 0.02  # s
MIN_SHARE = 0.02   # of the planner's total time on the map; smaller excess is ordinary OS noise
MIN_NEIGHBOURS = 10


def similar(sizes, x):
    """Index range of the sorted sizes within a factor of 2 of x."""
    lo, hi = bisect.bisect_left(sizes, x / 2.0), bisect.bisect_right(sizes, x * 2.0)
    if hi - lo < MIN_NEIGHBOURS:  # sparse sizes: the closest runs by rank instead
        c = bisect.bisect_left(sizes, x)
        lo = max(0, min(c - MIN_NEIGHBOURS // 2, len(sizes) - MIN_NEIGHBOURS))
        hi = min(len(sizes), lo + MIN_NEIGHBOURS)
    return lo, hi


def flagged_runs(result_file):
    with open(result_file) as f:
        data = json.load(f)
    per_algo = {}
    for r in data["results"]:
        for algo, v in r["algorithms"].items():
            if v["success"] and v["num_iterations"] > 0 and v["node_count"] > 0:
                per_algo.setdefault(algo, []).append((v["num_iterations"], v["node_count"], v["search_time"], r))
    bad = []
    for algo, runs in per_algo.items():
        min_excess = max(MIN_EXCESS, MIN_SHARE * sum(x[2] for x in runs))
        by_nodes = sorted((x[1], x[2] / x[1]) for x in runs)
        node_counts = [x[0] for x in by_nodes]
        runs.sort(key=lambda x: x[0])
        iters = [x[0] for x in runs]
        for it, nodes, t, r in runs:
            lo, hi = similar(iters, it)
            ref = statistics.median(runs[k][2] / runs[k][0] for k in range(lo, hi))
            excess = t - it * ref
            if t / it <= FACTOR * ref or excess <= min_excess:
                continue
            lo, hi = similar(node_counts, nodes)
            per_node = t / nodes / statistics.median(by_nodes[k][1] for k in range(lo, hi))
            if per_node > FACTOR:
                bad.append((algo, r["scenario"], r["trial"], t, it, t / it / ref, per_node, excess))
    return bad


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("folders", nargs="+")
    p.add_argument("--delete", action="store_true", help="delete result files of flagged maps")
    args = p.parse_args()

    n_bad_maps = 0
    for folder in args.folders:
        folder = os.path.expanduser(folder)
        for f in sorted(glob.glob(os.path.join(folder, "**", "map_*_result.json"), recursive=True)):
            bad = flagged_runs(f)
            if not bad:
                continue
            n_bad_maps += 1
            for algo, scen, trial, t, it, ratio, per_node, excess in bad:
                print("{}  {}  scenario {} trial {}  {:.1f} ms for {} iterations  ({:.1f}x similar runs, {:.1f}x per node, +{:.0f} ms)".format(
                    os.path.relpath(f, folder), algo, scen, trial, t * 1000, it, ratio, per_node, excess * 1000))
            if args.delete:
                os.remove(f)
                print("  deleted", f)
    print("{} map(s) with disturbed timing".format(n_bad_maps))


if __name__ == "__main__":
    main()
