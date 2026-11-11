#!/usr/bin/env python3
"""Generate N-robot scenarios on the dataset maps, one JSON file per map.

Kinds:
  random_<d>  random starts and goals, every robot travels at least <d> metres in a straight line
  parallel    starts in the bottom band, goals in the top band, same left-right order
  crossing    like parallel, but the goals are in reversed order, so the paths have to cross
  swap        like parallel, but every other robot travels the opposite way

Every scenario is checked when it is made: endpoints lie in free cells of the inflated map, the
endpoints of different robots are far enough apart, and each robot can reach its goal while
keeping D away from the endpoints of all the others.

Example:
  python3 generate_multi_scenarios.py --envs blobs_10 narrow_10 --N 2 4 8 16 --out ~/rrt_results/multi/scenarios
"""
import argparse
import json
import os

import numpy as np

import multi_common as mc

BAND_NEAR, BAND_FAR = 75.0, 95.0  # the bands reach from |y| = 75 to 95 m; the narrow walls end at 70 m


def disc_cells(p, radius):
    grid = np.zeros((mc.GRID, mc.GRID), dtype=bool)
    mc.block_point(grid, p, radius)
    return {tuple(c) for c in np.argwhere(grid)}


def reservation_connected(base, starts, goals, D):
    """Can each robot reach its goal with the discs around all other endpoints blocked?

    All discs are blocked once and labelled once. A robot may also use the cells that only its own
    discs block, so it is connected when those cells link its start and goal to a common region.
    """
    n = len(starts)
    s_discs = [disc_cells(p, D) for p in starts]
    g_discs = [disc_cells(p, D) for p in goals]
    cover = {}
    for i in range(n):
        for c in s_discs[i] | g_discs[i]:
            cover.setdefault(c, set()).add(i)
    blocked = base.copy()
    for c in cover:
        blocked[c] = True
    labels = mc.label_free(blocked)

    def regions_from(cell, own):
        seen, stack, regions = {cell}, [cell], set()
        while stack:
            x, y = stack.pop()
            for q in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                if not (0 <= q[0] < mc.GRID and 0 <= q[1] < mc.GRID) or q in seen:
                    continue
                if labels[q] >= 0:
                    regions.add(int(labels[q]))
                elif q in own:
                    seen.add(q)
                    stack.append(q)
        return regions, seen

    for i in range(n):
        own = {c for c in s_discs[i] | g_discs[i] if cover[c] == {i} and not base[c]}
        s_cell, g_cell = mc.cell_of(starts[i]), mc.cell_of(goals[i])
        if base[s_cell] or base[g_cell]:
            return False
        s_regions, s_seen = regions_from(s_cell, own)
        if g_cell in s_seen:
            continue
        g_regions, _ = regions_from(g_cell, own)
        if not s_regions & g_regions:
            return False
    return True


class MapSampler:
    def __init__(self, pcd, D, rng):
        self.D = D
        self.rng = rng
        self.sep = D + np.sqrt(2.0) * mc.RES
        self.base = mc.inflate(mc.load_occupancy(pcd), 0.5 * D)
        labels = mc.label_free(self.base)
        values, counts = np.unique(labels[labels >= 0], return_counts=True)
        self.free_cells = int(counts.sum()) if len(counts) else 0
        self.main_fraction = float(counts.max()) / self.free_cells if len(counts) else 0.0
        main = values[np.argmax(counts)] if len(counts) else -1
        self.cells = np.argwhere(labels == main)
        ys = mc.ORIGIN[1] + (self.cells[:, 1] + 0.5) * mc.RES
        self.bottom = self.cells[(ys >= -BAND_FAR) & (ys <= -BAND_NEAR)]
        self.top = self.cells[(ys >= BAND_NEAR) & (ys <= BAND_FAR)]

    def point(self, cells):
        c = cells[self.rng.randint(len(cells))]
        # stay off the cell border, so that rounding the coordinates cannot change the cell
        return mc.ORIGIN + (c + self.rng.uniform(0.05, 0.95, 2)) * mc.RES

    def far_enough(self, p, taken):
        return all(np.hypot(*(p - q)) >= self.sep for q in taken)

    def pick(self, cells, taken, tries=2000):
        if len(cells) == 0:
            return None
        for _ in range(tries):
            p = self.point(cells)
            if self.far_enough(p, taken):
                return p
        return None

    def random(self, n, d_min):
        starts, goals, taken = [], [], []
        for _ in range(n):
            for _ in range(2000):
                s, g = self.point(self.cells), self.point(self.cells)
                if np.hypot(*(s - g)) >= d_min and self.far_enough(s, taken) and self.far_enough(g, taken + [s]):
                    break
            else:
                return None
            starts.append(s)
            goals.append(g)
            taken += [s, g]
        return starts, goals

    def bands(self, n, kind):
        lower, upper, taken = [], [], []
        for _ in range(n):
            a = self.pick(self.bottom, taken)
            if a is None:
                return None
            taken.append(a)
            b = self.pick(self.top, taken)
            if b is None:
                return None
            taken.append(b)
            lower.append(a)
            upper.append(b)
        lower.sort(key=lambda p: p[0])
        upper.sort(key=lambda p: p[0], reverse=kind == "crossing")
        starts, goals = [], []
        for i in range(n):
            if kind == "swap" and i % 2:
                starts.append(upper[i])
                goals.append(lower[i])
            else:
                starts.append(lower[i])
                goals.append(upper[i])
        return starts, goals

    def make(self, kind, n):
        if kind.startswith("random_"):
            return self.random(n, float(kind.split("_")[1]))
        return self.bands(n, kind)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--dataset-root", default="/home/huy/rrt/dataset_maps_200x200")
    p.add_argument("--envs", nargs="+", default=["blobs_10", "blobs_20", "blobs_40", "narrow_10", "narrow_20",
                                                 "flappy_10", "flappy_20", "rooms_10", "rooms_20"])
    p.add_argument("--maps", type=int, default=5)
    p.add_argument("--N", nargs="+", type=int, default=[2, 4, 8, 16])
    p.add_argument("--kinds", nargs="+", default=["random_50", "random_150", "parallel", "crossing", "swap"])
    p.add_argument("--count", type=int, default=10, help="scenarios per (map, kind, N)")
    p.add_argument("--radius", type=float, default=0.25)
    p.add_argument("--margin", type=float, default=0.1)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--tries", type=int, default=30, help="attempts per scenario before giving up")
    p.add_argument("--out", required=True)
    args = p.parse_args()

    D = 2.0 * args.radius + args.margin
    missing = 0
    for e, env in enumerate(args.envs):
        env_out = os.path.join(args.out, env)
        os.makedirs(env_out, exist_ok=True)
        for m in range(args.maps):
            pcd = os.path.join(args.dataset_root, env, "map_{}.pcd".format(m))
            if not os.path.exists(pcd):
                print("  {} map_{} not found, skipping".format(env, m))
                continue
            # the same seed always gives the same scenarios for this map, whatever else is generated
            rng = np.random.RandomState(args.seed + 1000 * zlib_env(env) + m)
            sampler = MapSampler(pcd, D, rng)
            scenarios = []
            short = {}
            for kind in args.kinds:
                for n in args.N:
                    made = 0
                    for _ in range(args.count):
                        for _ in range(args.tries):
                            result = sampler.make(kind, n)
                            if result is not None and reservation_connected(sampler.base, result[0], result[1], D):
                                break
                            result = None
                        if result is None:
                            break
                        scenarios.append({
                            "id": len(scenarios),
                            "kind": kind,
                            "starts": [[round(float(v), 3) for v in q] for q in result[0]],
                            "goals": [[round(float(v), 3) for v in q] for q in result[1]],
                        })
                        made += 1
                    if made < args.count:
                        short["{} N={}".format(kind, n)] = made
                        missing += args.count - made
            with open(os.path.join(env_out, "map_{}.json".format(m)), "w") as f:
                json.dump({"map": "{}/map_{}".format(env, m), "robot_radius": args.radius, "margin": args.margin,
                           "seed": args.seed, "free_fraction": float(sampler.free_cells) / mc.GRID ** 2,
                           "main_region_fraction": sampler.main_fraction, "scenarios": scenarios}, f)
            note = "" if not short else "  short: " + ", ".join("{}={}".format(k, v) for k, v in short.items())
            print("{} map_{}: {} scenarios, largest region {:.1%} of free cells{}".format(
                env, m, len(scenarios), sampler.main_fraction, note))
    print("Done. {} requested scenarios could not be made.".format(missing))


def zlib_env(env):
    # a stable number for an environment name (hash() changes between Python runs)
    import zlib
    return zlib.crc32(env.encode()) % 100000


if __name__ == "__main__":
    main()
