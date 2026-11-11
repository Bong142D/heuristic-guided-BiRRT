#!/usr/bin/env python3
"""Draw one multi-robot run to a PNG: map, inflated map, paths and conflict zones.

Example:
  python3 plot_multi_paths.py ~/rrt_results/multi/quick/blobs_10/map_0_result.json --scenario 0 --method fconnect+reserve --out run.png
"""
import argparse
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import multi_common as mc


def piece(path, s0, s1):
    """The part of a polyline between arc lengths s0 and s1."""
    seg = np.hypot(*(path[1:] - path[:-1]).T)
    arc = np.concatenate(([0.0], np.cumsum(seg)))
    keep = (arc >= s0) & (arc <= s1)
    pts = [np.array([np.interp(s0, arc, path[:, 0]), np.interp(s0, arc, path[:, 1])])]
    pts += list(path[keep])
    pts.append(np.array([np.interp(s1, arc, path[:, 0]), np.interp(s1, arc, path[:, 1])]))
    return np.array(pts)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("result")
    p.add_argument("--scenario", type=int, default=0)
    p.add_argument("--trial", type=int, default=0)
    p.add_argument("--method", default="fconnect+reserve")
    p.add_argument("--dataset-root", default="/home/huy/rrt/dataset_maps_200x200")
    p.add_argument("--out", required=True)
    args = p.parse_args()

    data = mc.load_json(args.result)
    run = next(r for r in data["runs"] if r["scenario"] == args.scenario and r["trial"] == args.trial and r["method"] == args.method)
    env = os.path.basename(os.path.dirname(os.path.abspath(args.result)))
    name = os.path.basename(args.result).replace("_result.json", "")
    grid = mc.load_occupancy(os.path.join(args.dataset_root, env, name + ".pcd"))
    inflated = mc.inflate(grid, 0.5 * data["D"])

    fig, ax = plt.subplots(figsize=(10, 10))
    extent = [-100, 100, -100, 100]
    shade = np.ones(grid.shape)
    shade[inflated] = 0.8
    shade[grid] = 0.35
    ax.imshow(shade.T, origin="lower", extent=extent, cmap="gray", vmin=0, vmax=1, interpolation="nearest")

    colors = plt.cm.tab20(np.linspace(0, 1, 20))
    paths = {}
    for i, r in enumerate(run["robots"]):
        if not r.get("path"):
            continue
        path = np.array(r["path"])
        paths[i] = path
        c = colors[i % 20]
        ax.plot(path[:, 0], path[:, 1], color=c, lw=1.0, zorder=2)
        ax.plot(*path[0], "o", color=c, ms=6, mec="k", zorder=4)
        ax.plot(*path[-1], "*", color=c, ms=11, mec="k", zorder=4)
    for z in run.get("zones", []):
        for robot, s0, s1 in ((z["a"], z["a_in"], z["a_out"]), (z["b"], z["b_in"], z["b_out"])):
            seg = piece(paths[robot], s0, s1)
            ax.plot(seg[:, 0], seg[:, 1], color="red", lw=3.0, alpha=0.65, zorder=3)
    ax.set_title("{}/{} scenario {} ({}), N={}, {}\n{}, total length {:.0f} m, {} zones in {} pairs, {:.1%} of path length in a zone".format(
        env, name, run["scenario"], run["kind"], run["N"], run["method"],
        "success" if run["success"] else "FAILED: " + run["fail_class"], run.get("total_length", 0.0),
        run.get("n_zones", 0), run.get("conflict_pairs", 0), run.get("overlap_fraction", 0.0)))
    ax.set_xlim(-100, 100)
    ax.set_ylim(-100, 100)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    fig.tight_layout()
    fig.savefig(args.out, dpi=110)
    print("saved", args.out)


if __name__ == "__main__":
    main()
