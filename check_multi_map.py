#!/usr/bin/env python3
"""Compare the C++ map helpers (OccMap::inflateXY, blockDisc, blockPath, restoreBase, connectedXY)
cell by cell with the separate Python geometry of multi_common.py.

Run the node in self test mode first:
  roslaunch path_finder multi_robot.launch map_file:=MAP.pcd self_test_output:=/tmp/self_test.json
  python3 check_multi_map.py MAP.pcd /tmp/self_test.json
"""
import sys

import numpy as np

import check_multi_paths as cp
import multi_common as mc


def as_grid(text):
    return np.frombuffer(text.encode(), dtype=np.uint8).reshape(mc.GRID, mc.GRID) == ord("1")


def compare(name, got, want, errors):
    diff = int((got != want).sum())
    print("{:14s} {} blocked cells, {} differ".format(name, int(want.sum()), diff))
    if diff:
        errors.append(name)


def block_path(grid, path, radius):
    out = grid.copy()
    lo = mc.ORIGIN + np.array([[i, j] for i in range(mc.GRID) for j in range(mc.GRID)]) * mc.RES
    hit = np.zeros(len(lo), dtype=bool)
    for a, b in zip(path[:-1], path[1:]):
        near = np.flatnonzero((lo[:, 0] + mc.RES > min(a[0], b[0]) - radius - mc.RES) & (lo[:, 0] < max(a[0], b[0]) + radius + mc.RES) &
                              (lo[:, 1] + mc.RES > min(a[1], b[1]) - radius - mc.RES) & (lo[:, 1] < max(a[1], b[1]) + radius + mc.RES))
        hit[near[cp.segment_squares(a, b, lo[near], lo[near] + mc.RES) < radius]] = True
    out[hit.reshape(mc.GRID, mc.GRID)] = True
    return out


def compare_pairs(name, pairs, grid, errors):
    labels = mc.label_free(grid)
    wrong = 0
    connected = 0
    for item in pairs:
        a, b = mc.cell_of(item["a"]), mc.cell_of(item["b"])
        want = labels[a] >= 0 and labels[a] == labels[b]
        connected += bool(want)
        wrong += bool(want) != item["connected"]
    print("{:14s} {} pairs, {} connected, {} differ".format(name, len(pairs), connected, wrong))
    if wrong:
        errors.append(name)


def main():
    data = mc.load_json(sys.argv[2])
    D = data["D"]
    errors = []
    base = mc.inflate(mc.load_occupancy(sys.argv[1]), 0.5 * D)
    compare("inflated", as_grid(data["inflated"]), base, errors)

    disc = base.copy()
    mc.block_point(disc, np.array(data["disc_center"]), D)
    compare("disc", as_grid(data["disc"]), disc, errors)
    compare("restored", as_grid(data["restored"]), base, errors)

    blocked = block_path(base, np.array(data["path"]), D)
    compare("blocked path", as_grid(data["blocked_path"]), blocked, errors)

    compare_pairs("pairs, wall", data["blocked_pairs"], blocked, errors)
    compare_pairs("pairs, plain", data["plain_pairs"], base, errors)

    print("MISMATCH: " + ", ".join(errors) if errors else "all map helpers agree")
    sys.exit(1 if errors else 0)


if __name__ == "__main__":
    main()
