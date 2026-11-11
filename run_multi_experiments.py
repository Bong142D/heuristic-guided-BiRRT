#!/usr/bin/env python3
"""Run the multi-robot path planning experiments: one roslaunch per map, skipping finished maps.

Example:
  python3 generate_multi_scenarios.py --envs blobs_10 --maps 1 --N 2 4 --count 2 --out ~/rrt_results/multi/scenarios
  python3 run_multi_experiments.py --scenarios ~/rrt_results/multi/scenarios --trials 2 --out ~/rrt_results/multi/quick
  python3 check_multi_paths.py ~/rrt_results/multi/quick
  python3 analyze_multi.py ~/rrt_results/multi/quick
"""
import argparse
import os
import subprocess
import time

REPO = os.path.dirname(os.path.abspath(__file__))
DEFAULT_METHODS = "connect+independent,fconnect+independent,connect+reserve,fconnect+reserve,fconnect+separated"


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--dataset-root", default="/home/huy/rrt/dataset_maps_200x200")
    p.add_argument("--scenarios", required=True, help="folder made by generate_multi_scenarios.py")
    p.add_argument("--envs", nargs="*", default=None, help="default: every environment in --scenarios")
    p.add_argument("--maps", type=int, default=5)
    p.add_argument("--config", default=os.path.join(REPO, "src/path_finder/config/paper_table1_final.json"))
    p.add_argument("--methods", default=DEFAULT_METHODS)
    p.add_argument("--trials", type=int, default=5)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--time-limit-call", type=float, default=5.0)
    p.add_argument("--time-limit-total", type=float, default=60.0)
    p.add_argument("--path-trials", type=int, default=1,
                   help="save the paths of the first k trials of each scenario (-1 = all); only these can be checked and drawn")
    p.add_argument("--separated-max-n", type=int, default=1000, help="skip the separated methods above this many robots")
    p.add_argument("--out", required=True)
    args = p.parse_args()

    envs = args.envs or sorted(d for d in os.listdir(args.scenarios) if os.path.isdir(os.path.join(args.scenarios, d)))
    for env in envs:
        env_out = os.path.join(args.out, env)
        os.makedirs(env_out, exist_ok=True)
        print("\n[{}]".format(env))
        for m in range(args.maps):
            scenarios = os.path.join(args.scenarios, env, "map_{}.json".format(m))
            pcd = os.path.join(args.dataset_root, env, "map_{}.pcd".format(m))
            if not (os.path.exists(scenarios) and os.path.exists(pcd)):
                print("  map_{} not found, skipping".format(m))
                continue
            result = os.path.join(env_out, "map_{}_result.json".format(m))
            if os.path.exists(result):
                print("  map_{} already done, skipping".format(m))
                continue
            cmd = [
                "roslaunch", "path_finder", "multi_robot.launch",
                "map_file:=" + pcd,
                "scenarios_file:=" + os.path.abspath(scenarios),
                "output_result:=" + os.path.abspath(result),
                "input_param:=" + os.path.abspath(args.config),
                "methods:=" + args.methods,
                "trials:={}".format(args.trials),
                "seed:={}".format(args.seed),
                "time_limit_call:={}".format(args.time_limit_call),
                "time_limit_total:={}".format(args.time_limit_total),
                "path_trials:={}".format(args.path_trials),
                "separated_max_n:={}".format(args.separated_max_n),
            ]
            t0 = time.time()
            with open(result.replace("_result.json", ".log"), "w") as log:
                subprocess.call(cmd, stdout=log, stderr=subprocess.STDOUT)
            status = "ok" if os.path.exists(result) else "FAILED (see the .log file)"
            print("  map_{}: {} in {:.0f}s".format(m, status, time.time() - t0))
    print("\nDone.")


if __name__ == "__main__":
    main()
