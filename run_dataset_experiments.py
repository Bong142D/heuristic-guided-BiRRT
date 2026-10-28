#!/usr/bin/env python3
"""Run the batch experiment (Table 2 of the RRT-FConnect paper) over a generated dataset.

For every environment and map, launches test_planners.launch in experiment mode, which
runs each start/goal scenario `--trials` times with every enabled planner and writes one
result JSON per map. Per-environment results are then merged into <out>/<env>.json.

Example:
  python3 run_dataset_experiments.py --trials 10
  python3 run_dataset_experiments.py --envs blobs_10 --maps 1 --max-scenarios 2 --trials 2   # quick test
"""
import argparse
import json
import os
import subprocess
import time

REPO = os.path.dirname(os.path.abspath(__file__))


def merge_results(env_dir, out_path):
    merged, meta = [], None
    for name in sorted(os.listdir(env_dir)):
        if not (name.startswith("map_") and name.endswith("_result.json")):
            continue
        with open(os.path.join(env_dir, name)) as f:
            data = json.load(f)
        if meta is None:
            meta = {k: v for k, v in data.items() if k != "results"}
        map_name = name[: -len("_result.json")]
        for r in data.get("results", []):
            r["map"] = map_name
            merged.append(r)
    if meta is None:
        print("  No results to merge in", env_dir)
        return
    meta["results"] = merged
    with open(out_path, "w") as f:
        json.dump(meta, f, indent=2)
    print("  Merged {} runs -> {}".format(len(merged), out_path))


def run_map(map_pcd, scenarios, args, out_file):
    cmd = [
        "roslaunch", "path_finder", "test_planners.launch",
        "experiment:=true",
        "map_file:=" + map_pcd,
        "scenarios_file:=" + scenarios,
        "input_param:=" + os.path.abspath(args.config),
        "output_result:=" + out_file,
        "trials_per_query:={}".format(args.trials),
        "run_brrt:={}".format(str("brrt" in args.baselines).lower()),
        "run_goal_bias_brrt:={}".format(str("gb" in args.baselines).lower()),
        "run_brrt_case1:={}".format(str("ebi" in args.baselines).lower()),
        "run_brrt_optimize:=true",
    ]
    log_path = out_file.replace("_result.json", ".log")
    with open(log_path, "w") as log:
        subprocess.call(cmd, stdout=log, stderr=subprocess.STDOUT)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--dataset-root", default="/home/huy/rrt/dataset_maps_200x200",
                   help="folder created by generate_dataset.py")
    p.add_argument("--envs", nargs="+", default=["blobs_10", "blobs_20", "blobs_40", "narrow_10"],
                   help="environment sub-folders to run")
    p.add_argument("--config", default=os.path.join(REPO, "src/path_finder/config/paper_table1.json"),
                   help="planner parameter JSON")
    p.add_argument("--trials", type=int, default=10, help="runs per start/goal scenario")
    p.add_argument("--maps", type=int, default=5, help="number of maps per environment")
    p.add_argument("--max-scenarios", type=int, default=0, help="use only the first N scenarios of each map (0 = all)")
    p.add_argument("--baselines", nargs="*", default=["brrt", "gb", "ebi"], choices=["brrt", "gb", "ebi"],
                   help="baselines run next to RRT-FConnect: brrt = RRT-Connect, gb = GB-RRT-Connect, ebi = EBi-RRT")
    p.add_argument("--out", default=os.path.expanduser("~/rrt_results/" + time.strftime("%Y%m%d_%H%M%S")),
                   help="output folder")
    args = p.parse_args()

    os.makedirs(args.out, exist_ok=True)
    print("Results ->", args.out)
    for env in args.envs:
        env_src = os.path.join(args.dataset_root, env)
        env_out = os.path.join(args.out, env)
        os.makedirs(env_out, exist_ok=True)
        print("\n[{}]".format(env))
        for i in range(args.maps):
            map_pcd = os.path.join(env_src, "map_{}.pcd".format(i))
            scenarios = os.path.join(env_src, "map_{}_scenarios.json".format(i))
            if not (os.path.exists(map_pcd) and os.path.exists(scenarios)):
                print("  map_{} not found, skipping".format(i))
                continue
            if args.max_scenarios > 0:
                with open(scenarios) as f:
                    trimmed = json.load(f)[: args.max_scenarios]
                scenarios = os.path.join(env_out, "map_{}_scenarios.json".format(i))
                with open(scenarios, "w") as f:
                    json.dump(trimmed, f)
            out_file = os.path.join(env_out, "map_{}_result.json".format(i))
            if os.path.exists(out_file):
                print("  map_{} already done, skipping".format(i))
                continue
            t0 = time.time()
            run_map(map_pcd, scenarios, args, out_file)
            status = "ok" if os.path.exists(out_file) else "FAILED (see {})".format(out_file.replace("_result.json", ".log"))
            print("  map_{}: {} in {:.0f}s".format(i, status, time.time() - t0))
        merge_results(env_out, os.path.join(args.out, env + ".json"))

    print("\nDone. Summarize with:\n  python3 analyze_results.py", args.out)


if __name__ == "__main__":
    main()
