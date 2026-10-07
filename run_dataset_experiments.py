#!/usr/bin/env python3
import os
import json
import time
import subprocess
import argparse


def merge_config_results(cfg_out_dir, cfg_name, output_dir):
    """Merge all map_N_result_I.json files for a config into one JSON."""
    merged_results = []
    meta = None

    result_files = sorted([
        f for f in os.listdir(cfg_out_dir) if f.endswith(".json")
    ])

    if not result_files:
        print("  No result files to merge for {}".format(cfg_name))
        return

    for rf in result_files:
        path = os.path.join(cfg_out_dir, rf)
        with open(path, 'r') as f:
            data = json.load(f)
        if meta is None:
            meta = {k: v for k, v in data.items() if k != "results"}
        merged_results.extend(data.get("results", []))

    merged = dict(meta)
    merged["results"] = merged_results

    out_path = os.path.join(output_dir, "{}.json".format(cfg_name))
    with open(out_path, 'w') as f:
        json.dump(merged, f, indent=4)
    print("  Merged {} results -> {}".format(len(merged_results), out_path))


def run_experiments(dataset_dir, input_dir, output_dir):
    if not os.path.isdir(dataset_dir):
        print("Dataset dir {} not found".format(dataset_dir))
        return
    if not os.path.isdir(input_dir):
        print("Input dir {} not found".format(input_dir))
        return

    os.makedirs(output_dir, exist_ok=True)

    # Collect all input param files
    input_files = sorted([
        f for f in os.listdir(input_dir) if f.endswith(".json")
    ])
    print("Found {} input configs".format(len(input_files)))

    for cfg_idx, cfg_file in enumerate(input_files):
        params_file = os.path.join(input_dir, cfg_file)
        cfg_name = os.path.splitext(cfg_file)[0]  # strip .json

        # Per-config output subdirectory
        cfg_out_dir = os.path.join(output_dir, cfg_name)
        os.makedirs(cfg_out_dir, exist_ok=True)

        print("\n[{}/{}] Config: {}".format(cfg_idx + 1, len(input_files), cfg_name))

        # Iterate over maps
        for map_idx in range(5):
            map_pcd  = os.path.join(dataset_dir, "map_{}.pcd".format(map_idx))
            map_json = os.path.join(dataset_dir, "map_{}_scenarios.json".format(map_idx))

            if not os.path.exists(map_pcd) or not os.path.exists(map_json):
                print("  Map {} not found, skipping".format(map_idx))
                continue

            print("  Map {}: {}".format(map_idx, map_pcd))

            out_file = os.path.join(cfg_out_dir, "map_{}_result.json".format(map_idx))

            # Skip if already done
            if os.path.exists(out_file):
                print("    Map {} results already exist, skipping".format(map_idx))
                continue

            print("    Running Map {} batch scenarios".format(map_idx))

            planner_cmd = [
                "roslaunch", "path_finder", "test_planners.launch",
                "map_file:={}".format(map_pcd),
                "scenarios_file:={}".format(map_json),
                "input_param:={}".format(params_file),
                "output_result:={}".format(out_file),
            ]

            planner_proc = subprocess.Popen(planner_cmd)
            planner_proc.wait()

        print("  Config {} done. Merging...".format(cfg_name))
        merge_config_results(cfg_out_dir, cfg_name, output_dir)

    print("\nAll experiments complete. Results in: {}".format(output_dir))


if __name__ == '__main__':
    # parser = argparse.ArgumentParser(
    #     description="Run batch dataset experiments across multiple configs, maps, and scenarios."
    # )
    # parser.add_argument(
    #     "--dataset", type=str, required=True,
    #     help="Path to dataset dir containing map_N.pcd and map_N_scenarios.json files"
    # )
    # parser.add_argument(
    #     "--input", type=str, required=True,
    #     help="Path to directory containing input param JSON files (one per config)"
    # )
    # parser.add_argument(
    #     "--output", type=str, required=True,
    #     help="Path to output directory; per-config results go into <output>/<config_name>/"
    # )
    # args = parser.parse_args()
    map_types = ["blobs","narrow","flappy","rooms"]
    map_fills = ["10","20","40"]
    for map_type in map_types:
        for map_fill in map_fills:
            if map_type != "blobs" and map_fill in ["20","40"]:
                continue
            map_string = f"{map_type}_{map_fill}"
            dataset = f"/home/huy/rrt/dataset_maps_200x200/{map_string}"
            input_dir = f"/home/huy/rrt/experiments/2026_05_21/input"
            output_dir = f"/home/huy/rrt/experiments/2026_05_21_new/output_200_{map_string}"
            print(f"Running experiment with {map_string}")
            run_experiments(dataset, input_dir, output_dir)

    # dataset = "/home/huy/rrt/dataset_maps/blobs_20"
    # input_dir = "/home/huy/rrt/experiments/2026_04_16/input"
    # output_dir = "/home/huy/rrt/experiments/2026_04_16/output_blobs_20"
    # run_experiments(dataset, input_dir, output_dir)
