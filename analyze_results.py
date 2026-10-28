#!/usr/bin/env python3
"""Summarize batch results as a Table 2-style comparison with the RRT-FConnect paper.

Usage:
  python3 analyze_results.py ~/rrt_results/<run_folder> [--extra <other_run_folder> ...]

--extra: planners missing from <run_folder> are taken from these folders (same maps and
scenarios, run separately, e.g. EBi-RRT in its own pass). Such rows are marked "(extra)".
The machine can run at a different speed in that pass, so their "vs RRT-Connect" goes through
RRT-FConnect, which every pass runs: (time / RRT-FConnect of the extra pass) x
(RRT-FConnect / RRT-Connect of <run_folder>). Their time columns are as measured.

Reads <run_folder>/<env>.json written by run_dataset_experiments.py. Path length, nodes,
iterations and search time are averaged over successful runs; success rate is over all runs.
Every run counts: maps with disturbed timing are re-run (check_timing.py), not filtered here,
because a filter on long times would also drop genuinely long runs.
"""
import json
import os
import statistics
import sys

# Code class name -> name used in the paper
ALGO_NAMES = [
    ("BRRT", "RRT-Connect"),
    ("GoalBiasBRRT", "GB-RRT-Connect"),
    ("BRRT_Case1", "EBi-RRT (?)"),
    ("SOF_BRRT_Case3", "RRT-FConnect"),
]

ENV_NAMES = {
    "blobs_10": "Simple cluttered",
    "blobs_20": "General cluttered",
    "blobs_40": "Highly cluttered",
    "narrow_10": "Narrow passage",
    "narrow_20": "Narrow passage",
    "narrow_40": "Narrow passage",
}

# Table 2 of the paper: path length, nodes, iterations, search time (ms), success rate (%)
PAPER = {
    "Simple cluttered": {
        "RRT-Connect": (195.4, 606.4, 554.7, 1.31, 100), "GB-RRT-Connect": (194.6, 606.1, 797.0, 1.56, 100),
        "EBi-RRT (?)": (189.9, 543.2, 410.6, 3.82, 100), "RRT-FConnect": (185.1, 483.7, 264.7, 1.47, 100)},
    "General cluttered": {
        "RRT-Connect": (229.7, 1209.4, 3303.1, 7.88, 100), "GB-RRT-Connect": (228.7, 1218.6, 4786.8, 9.53, 100),
        "EBi-RRT (?)": (215.4, 1084.4, 2162.3, 23.62, 100), "RRT-FConnect": (204.2, 836.7, 1426.4, 7.51, 100)},
    "Highly cluttered": {
        "RRT-Connect": (280.9, 4085.2, 35138.7, 118.08, 93), "GB-RRT-Connect": (275.9, 3636.7, 44601.2, 116.55, 86),
        "EBi-RRT (?)": (267.2, 5113.4, 26463.3, 442.51, 88), "RRT-FConnect": (246.8, 2624.9, 13236.8, 75.10, 95)},
    "Narrow passage": {
        "RRT-Connect": (302.9, 1545.2, 9430.7, 17.30, 100), "GB-RRT-Connect": (302.9, 1468.7, 12621.4, 18.52, 99.5),
        "EBi-RRT (?)": (312.1, 7634.8, 18414.5, 1066.72, 95.8), "RRT-FConnect": (280.0, 1178.7, 4577.7, 16.16, 100)},
}


def mean(xs):
    return statistics.mean(xs) if xs else float("nan")


def summarize(path):
    with open(path) as f:
        data = json.load(f)
    rows = {}
    for code, _ in ALGO_NAMES:
        runs = [r["algorithms"][code] for r in data["results"] if code in r["algorithms"]]
        if not runs:
            continue
        ok = [r for r in runs if r["success"]]
        times = [r["search_time"] * 1000.0 for r in ok]
        rows[code] = (
            mean([r["path_length"] for r in ok]),
            mean([r["node_count"] for r in ok]),
            mean([r["num_iterations"] for r in ok]),
            mean(times),
            100.0 * len(ok) / len(runs),
            len(runs),
            statistics.median(times) if times else float("nan"),
        )
    return data.get("parameters", {}), rows


def main():
    args = sys.argv[1:]
    if not args or args[0].startswith("-") or (len(args) > 1 and (args[1] != "--extra" or len(args) < 3)):
        print(__doc__)
        sys.exit(1)
    folder, extras = args[0], args[2:]
    envs = sorted(f[:-5] for f in os.listdir(folder) if f.endswith(".json"))
    params = {}
    for env in envs:
        params, rows = summarize(os.path.join(folder, env + ".json"))
        scale = {}  # extra planner -> machine-speed factor of its pass, from the shared RRT-FConnect
        for extra in extras:
            path = os.path.join(extra, env + ".json")
            if os.path.exists(path):
                extra_rows = summarize(path)[1]
                f = "SOF_BRRT_Case3"
                factor = rows[f][3] / extra_rows[f][3] if f in rows and f in extra_rows else 1.0
                for code, row in extra_rows.items():
                    if code not in rows:
                        rows[code] = row
                        scale[code] = factor
        paper_env = ENV_NAMES.get(env)
        rrtc_time = rows["BRRT"][3] if "BRRT" in rows else None
        print("\n### {} ({})".format(env, paper_env or "not in paper"))
        print("| Planner | Runs | Path length | Nodes | Iterations | Time (ms) | Median (ms) | vs RRT-Connect | Success | Paper: length / nodes / iters / ms / success |")
        print("|---|---|---|---|---|---|---|---|---|---|")
        for code, name in ALGO_NAMES:
            if code not in rows:
                continue
            L, N, I, T, S, n, M = rows[code]
            ref = PAPER.get(paper_env, {}).get(name)
            ref_s = " / ".join(str(v) for v in ref[:4]) + " / {}%".format(ref[4]) if ref else "-"
            ratio = "{:.2f}".format(T * scale.get(code, 1.0) / rrtc_time) if rrtc_time else "-"
            label = name + (" (extra)" if code in scale else "")
            print("| {} | {} | {:.1f} | {:.1f} | {:.1f} | {:.2f} | {:.2f} | {} | {:.1f}% | {} |".format(label, n, L, N, I, T, M, ratio, S, ref_s))
    print("\nParameters:", params)


if __name__ == "__main__":
    main()
