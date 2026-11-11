#!/usr/bin/env python3
"""Summarize multi-robot path planning results: one table row per (environment, kind, N, method).

Costs are medians over successful runs. The "vs" columns compare with --baseline on the runs where
both methods succeeded on the same scenario and trial, as a ratio of sums, with a bootstrap 95%
interval over scenarios. Success has a bootstrap 95% interval over scenarios too.

Example:
  python3 analyze_multi.py ~/rrt_results/multi/quick
  python3 analyze_multi.py ~/rrt_results/multi/quick --merge-envs
"""
import argparse
import glob
import os
from collections import defaultdict

import numpy as np

import multi_common as mc


def load_runs(folder):
    runs = []
    for path in sorted(glob.glob(os.path.join(folder, "*", "map_*_result.json"))):
        env = os.path.basename(os.path.dirname(path))
        name = os.path.basename(path).replace("_result.json", "")
        for r in mc.load_json(path)["runs"]:
            r["env"] = env
            r["scenario_key"] = (env, name, r["scenario"])
            runs.append(r)
    return runs


def bootstrap(groups, stat, rng, n=1000):
    """95% interval of stat(list of items) when whole groups are resampled."""
    if len(groups) < 2:
        return float("nan"), float("nan")
    values = []
    for _ in range(n):
        picked = [groups[i] for i in rng.randint(len(groups), size=len(groups))]
        v = stat([x for g in picked for x in g])
        if v == v:
            values.append(v)
    return (float(np.percentile(values, 2.5)), float(np.percentile(values, 97.5))) if values else (float("nan"), float("nan"))


def by_scenario(items, key=lambda x: x["scenario_key"]):
    groups = defaultdict(list)
    for r in items:
        groups[key(r)].append(r)
    return list(groups.values())


def median(items, field):
    values = [r[field] for r in items if field in r]
    return float(np.median(values)) if values else float("nan")


def ratio(pairs, field):
    a = sum(p[0][field] for p in pairs)
    b = sum(p[1][field] for p in pairs)
    return a / b if b > 0 else float("nan")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("results")
    p.add_argument("--baseline", default="connect+independent")
    p.add_argument("--merge-envs", action="store_true", help="one row per (kind, N, method) over all environments")
    args = p.parse_args()

    rng = np.random.RandomState(1)
    runs = load_runs(args.results)
    print("{} runs".format(len(runs)))
    table = defaultdict(list)
    lookup = {}
    for r in runs:
        env = "all" if args.merge_envs else r["env"]
        table[(env, r["kind"], r["N"], r["method"])].append(r)
        lookup[(r["scenario_key"], r["trial"], r["method"])] = r

    print("| env | kind | N | method | runs | success (95% CI) | total length | zones | pairs | overlap | endpoint z. | head-on z. | search ms | time vs base (95% CI) | length vs base |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for key in sorted(table):
        items = table[key]
        ok = [r for r in items if r["success"]]
        rate = len(ok) / len(items)
        lo, hi = bootstrap(by_scenario(items), lambda xs: np.mean([x["success"] for x in xs]), rng)
        pairs = []
        for r in ok:
            base = lookup.get((r["scenario_key"], r["trial"], args.baseline))
            if base is not None and base["success"]:
                pairs.append((r, base))
        if pairs and key[3] != args.baseline:
            time_ratio = ratio(pairs, "search_time")
            t_lo, t_hi = bootstrap(by_scenario(pairs, key=lambda x: x[0]["scenario_key"]), lambda xs: ratio(xs, "search_time"), rng)
            time_text = "{:.2f} ({:.2f}–{:.2f})".format(time_ratio, t_lo, t_hi)
            length_text = "{:.3f}".format(ratio(pairs, "total_length"))
        else:
            time_text = length_text = "–"
        print("| {} | {} | {} | {} | {} | {:.1%} ({:.0%}–{:.0%}) | {:.0f} | {:.1f} | {:.1f} | {:.1%} | {:.2f} | {:.2f} | {:.0f} | {} | {} |".format(
            key[0], key[1], key[2], key[3], len(items), rate, lo, hi,
            median(ok, "total_length"), median(ok, "n_zones"), median(ok, "conflict_pairs"), median(ok, "overlap_fraction"),
            np.mean([r["endpoint_zones"] for r in ok]) if ok else float("nan"),
            np.mean([r["head_on_zones"] for r in ok]) if ok else float("nan"),
            median(items, "search_time") * 1000.0, time_text, length_text))

    failures = defaultdict(int)
    for r in runs:
        if not r["success"]:
            failures[(r["method"], r["fail_class"])] += 1
    if failures:
        print("\nFailures:")
        for k in sorted(failures):
            print("  {} {}: {}".format(k[0], k[1], failures[k]))
    dropped = sum(r.get("reserve_dropped", 0) for r in runs)
    print("\nReservations dropped because they cut a robot off: {}".format(dropped))


if __name__ == "__main__":
    main()
