#!/usr/bin/env python3
"""Generate the 200x200 benchmark maps and their start/goal scenarios.

Needs a running roscore. Maps that already exist are skipped, so the script can be
re-run to add new environments, e.g.:
  python3 generate_dataset.py --types flappy rooms --fills 0.1 0.2
"""
import argparse
import os
import time
import glob
import shutil
import subprocess

def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--types", nargs="+", default=["blobs", "narrow", "flappy", "rooms"])
    p.add_argument("--fills", nargs="+", type=float, default=[0.1, 0.2, 0.4])
    p.add_argument("--maps", type=int, default=5, help="maps per environment")
    p.add_argument("--out", default="/home/huy/rrt/dataset_maps_200x200")
    args = p.parse_args()
    map_types, fill_levels, num_maps, base_out_dir = args.types, args.fills, args.maps, args.out

    if not os.path.exists(base_out_dir):
        os.makedirs(base_out_dir)

    for m_type in map_types:
        for fill in fill_levels:
            dir_name = "{}_{}".format(m_type, int(fill * 100))
            full_path = os.path.join(base_out_dir, dir_name)
            if not os.path.exists(full_path):
                os.makedirs(full_path)
            
            for i in range(num_maps):
                print("Generating {} fill {} map {}/{}".format(m_type, fill, i+1, num_maps))
                map_prefix = os.path.join(full_path, "map_{}".format(i))
                if os.path.exists(map_prefix + ".pcd") and os.path.exists(map_prefix + "_scenarios.json"):
                    print("  already exists, skipping")
                    continue
                
                # 1. Start Map Generator
                map_cmd = [
                    "rosrun", "map_generator", "dataset_map_generator.py",
                    "_map_type:={}".format(m_type),
                    "_fill_level:={}".format(fill),
                    "_map/x_size:=200.0",
                    "_map/y_size:=200.0",
                    "_fail_limit:=1000000"
                ]
                map_proc = subprocess.Popen(map_cmd)
                
                # Wait for map to be generated and published
                time.sleep(5.0) 

                # 2. Collect PCD
                pcd_temp_dir = os.path.join(full_path, "temp_pcd_{}".format(i))
                if not os.path.exists(pcd_temp_dir):
                    os.makedirs(pcd_temp_dir)

                pcd_cmd = [
                    "rosrun", "pcl_ros", "pointcloud_to_pcd",
                    "input:=/random_forest/all_map",
                    "_prefix:={}/tmp_".format(pcd_temp_dir),
                    "_binary:=true",
                    "_compressed:=true"
                ]
                # Capturing stdout to monitor for "Data saved to"
                pcd_proc = subprocess.Popen(pcd_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                
                # Poll stdout for the "Data saved to" message
                saved = False
                start_time = time.time()
                while time.time() - start_time < 30.0: # 30s timeout
                    line = pcd_proc.stdout.readline()
                    if not line:
                        break
                    if "Data saved to" in line:
                        saved = True
                        break
                
                time.sleep(1.0) # Small extra buffer
                pcd_proc.terminate()
                pcd_proc.wait()
                os.sync() # Force disk commit

                # rename the dumped pcd file (sort to get the first/oldest one which is likely complete)
                pcd_files = sorted(glob.glob(os.path.join(pcd_temp_dir, "*.pcd")))
                if len(pcd_files) > 0:
                    shutil.move(pcd_files[0], map_prefix + ".pcd")
                shutil.rmtree(pcd_temp_dir, ignore_errors=True)

                # 3. Call planner to sample start/goal
                planner_cmd = [
                    "rosrun", "path_finder", "path_finder",
                    "/global_cloud:=/random_forest/all_map", 
                    "_generate_mode:=true",
                    "_experiment:=true",
                    "_dataset_path:={}".format(map_prefix),
                    "_occ_map/origin_x:=-100.0",
                    "_occ_map/origin_y:=-100.0",
                    "_occ_map/origin_z:=-1.0",
                    "_occ_map/map_size_x:=200.0",
                    "_occ_map/map_size_y:=200.0",
                    "_occ_map/map_size_z:=1.5",
                    "_occ_map/resolution:=0.5",
                    "_gen_min_dist:=150.0"
                ]
                planner_proc = subprocess.Popen(planner_cmd)
                planner_proc.wait()

                map_proc.terminate()
                map_proc.wait()
                time.sleep(1.0)
                
if __name__ == '__main__':
    main()
