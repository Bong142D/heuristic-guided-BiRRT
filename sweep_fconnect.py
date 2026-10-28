#!/usr/bin/env python3
"""Run RRT-FConnect (SOF_BRRT_Case3) variants to find why the reproduction differs from the paper.

Each variant is paper_table1.json plus the experiment switches read by testcase.h:
  r_max      max ray length; <= 0 keeps the code default 20 * epsilon = 10 (paper: 5)
  exploration  random-exploration mode after 20 non-improving guide pairs (not in the paper)
  knn_k      new node is paired with its k nearest nodes of the other tree
  knn_exact  use kd_nearest_n_exact instead of kd_nearest_n (prunes with heap[0])
  pool_size  0 = unbounded heap cache (code), > 0 = fixed pool of best pairs (paper: 100)
  pop_guide  remove the guide pair after using it (code) or keep it until a better one appears
  reject_invalid_sample  end the iteration when the sample is inside an obstacle (code);
             false = steer towards it like RRT-Connect / Algorithm 1
  apf_always apply APF steering even when the sample is within one step (paper)
  explore_trigger / explore_len  exploration mode thresholds (code: 20 / 50)

RRT-Connect is run next to every variant (interleaved per query), so the time ratio
T/T_RRTC is not affected by machine speed drifting between variants.

Example:
  python3 sweep_fconnect.py --variants orig rmax5 noexplore --trials 3 --out ~/rrt_results/sweep1
  python3 sweep_fconnect.py --summary-only --out ~/rrt_results/sweep1 --ref ~/rrt_results/full_run_1
"""
import argparse
import json
import os
import statistics
import subprocess
import sys

REPO = os.path.dirname(os.path.abspath(__file__))
BASE_CONFIG = os.path.join(REPO, "src/path_finder/config/paper_table1.json")

PAPER_SWITCHES = dict(r_max=5.0, exploration=False, knn_exact=True, pool_size=100)
FIXED = dict(PAPER_SWITCHES, exploration=True)  # = paper_explore = paper_table1_fixed.json

VARIANTS = {
    # one change at a time from the original code
    "orig": {},
    "rmax5": dict(r_max=5.0),
    "noexplore": dict(exploration=False),
    "knn_exact": dict(knn_exact=True),
    "pool100": dict(pool_size=100),
    "keep": dict(pop_guide=False),
    "pool100_keep": dict(pool_size=100, pop_guide=False),
    # everything the paper describes
    "paper": dict(PAPER_SWITCHES),
    "paper_keep": dict(PAPER_SWITCHES, pop_guide=False),
    "paper_explore": dict(PAPER_SWITCHES, exploration=True),
    "paper_k10": dict(PAPER_SWITCHES, knn_k=10),
    "paper_k60": dict(PAPER_SWITCHES, knn_k=60),
    "paper_rmax10": dict(PAPER_SWITCHES, r_max=10.0),
    # round 2: exploration kept on (round 1 showed it is required), pool variations
    "pool100_exact": dict(pool_size=100, knn_exact=True),
    "pool100_rmax5": dict(pool_size=100, r_max=5.0),
    "pe_pool30": dict(PAPER_SWITCHES, exploration=True, pool_size=30),
    "pe_pool300": dict(PAPER_SWITCHES, exploration=True, pool_size=300),
    "pe_pool1000": dict(PAPER_SWITCHES, exploration=True, pool_size=1000),
    "pe_k10": dict(PAPER_SWITCHES, exploration=True, knn_k=10),
    # round 4: remaining iteration gap, starting from the fixed config (paper_table1_fixed.json)
    "fx": dict(FIXED),
    "fx_noreject": dict(FIXED, reject_invalid_sample=False),
    "fx_apf": dict(FIXED, apf_always=True),
    "fx_noreject_apf": dict(FIXED, reject_invalid_sample=False, apf_always=True),
    "fx_expl10": dict(FIXED, explore_trigger=10),
    "fx_expl50": dict(FIXED, explore_trigger=50),
    "fx_len20": dict(FIXED, explore_len=20),
    "fx_len100": dict(FIXED, explore_len=100),
    "fx_noreject_noexpl": dict(FIXED, reject_invalid_sample=False, exploration=False),
    "fx_noreject_expl50": dict(FIXED, reject_invalid_sample=False, explore_trigger=50),
    "fx_noreject_len20": dict(FIXED, reject_invalid_sample=False, explore_len=20),
}

ENVS = ["blobs_10", "blobs_20", "blobs_40", "narrow_10"]


def write_config(name, switches, out_dir):
    with open(BASE_CONFIG) as f:
        cfg = json.load(f)
    cfg.update(switches)
    cfg["environment"] = "sweep_" + name
    path = os.path.join(out_dir, "config.json")
    with open(path, "w") as f:
        json.dump(cfg, f, indent=4)
    return path


def stats(path, code):
    with open(path) as f:
        data = json.load(f)
    runs = [r["algorithms"][code] for r in data["results"] if code in r["algorithms"]]
    if not runs:
        return None
    ok = [r for r in runs if r["success"]]
    times = [r["search_time"] * 1000.0 for r in ok]

    def mean(xs):
        return statistics.mean(xs) if xs else float("nan")

    return dict(
        runs=len(runs),
        length=mean([r["path_length"] for r in ok]),
        nodes=mean([r["node_count"] for r in ok]),
        iters=mean([r["num_iterations"] for r in ok]),
        time=mean(times),
        median=statistics.median(times) if times else float("nan"),
        success=100.0 * len(ok) / len(runs),
        us_per_iter=1000.0 * sum(times) / max(1, sum(r["num_iterations"] for r in ok)),
    )


def summarize(out, names, envs, ref):
    print("\n# RRT-FConnect variants ({})".format(out))
    for env in envs:
        print("\n### {}".format(env))
        print("| Variant | Runs | Path length | Nodes | Iterations | Time TB (ms) | Time TV (ms) | µs/iter | Success | T/T_RRTC |")
        print("|---|---|---|---|---|---|---|---|---|---|")
        rows = []
        if ref:
            for code, label in [("BRRT", "ref: RRT-Connect"), ("SOF_BRRT_Case3", "ref: RRT-FConnect")]:
                path = os.path.join(ref, env + ".json")
                if os.path.exists(path):
                    rows.append((label, stats(path, code)))
        for name in names:
            path = os.path.join(out, name, env + ".json")
            if os.path.exists(path):
                s = stats(path, "SOF_BRRT_Case3")
                rrtc = stats(path, "BRRT")
                if s and rrtc:
                    s["ratio"] = s["time"] / rrtc["time"]
                rows.append((name, s))
        for label, s in rows:
            if s is None:
                continue
            ratio = "{:.2f}".format(s["ratio"]) if "ratio" in s else "-"
            print("| {} | {} | {:.1f} | {:.0f} | {:.0f} | {:.2f} | {:.2f} | {:.2f} | {:.1f} | {} |".format(
                label, s["runs"], s["length"], s["nodes"], s["iters"], s["time"], s["median"], s["us_per_iter"], s["success"], ratio))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--variants", nargs="+", default=list(VARIANTS), help="names from VARIANTS")
    p.add_argument("--envs", nargs="+", default=ENVS)
    p.add_argument("--trials", type=int, default=3)
    p.add_argument("--maps", type=int, default=5)
    p.add_argument("--out", required=True)
    p.add_argument("--ref", default="", help="full run folder whose baselines are shown for comparison")
    p.add_argument("--summary-only", action="store_true")
    args = p.parse_args()

    unknown = [v for v in args.variants if v not in VARIANTS]
    if unknown:
        sys.exit("Unknown variants: {}".format(unknown))
    args.out = os.path.abspath(os.path.expanduser(args.out))
    if not args.summary_only:
        for name in args.variants:
            vdir = os.path.join(args.out, name)
            os.makedirs(vdir, exist_ok=True)
            cfg = write_config(name, VARIANTS[name], vdir)
            print("\n=== {} {} ===".format(name, VARIANTS[name]), flush=True)
            subprocess.call([sys.executable, os.path.join(REPO, "run_dataset_experiments.py"),
                             "--baselines", "brrt", "--config", cfg, "--out", vdir,
                             "--trials", str(args.trials), "--maps", str(args.maps), "--envs"] + args.envs)
    summarize(args.out, args.variants, args.envs, os.path.expanduser(args.ref) if args.ref else "")


if __name__ == "__main__":
    main()
