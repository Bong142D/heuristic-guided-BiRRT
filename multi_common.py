#!/usr/bin/env python3
"""Shared helpers for the multi-robot scripts: read a .pcd map, build the 2D occupancy grid,
inflate it and label its connected regions.

This is written separately from the C++ OccMap on purpose: the scenario generator and the
path checker use it as an independent second implementation of the same geometry.
"""
import json
import struct

import numpy as np

ORIGIN = np.array([-100.0, -100.0])
MAP_SIZE = 200.0
RES = 0.5
GRID = int(np.ceil(MAP_SIZE / RES))
Z_ORIGIN = -1.0
Z_LAYERS = 3  # layer 0 is the floor that OccMap::init marks as occupied


def lzf_decompress(data, out_len):
    out = bytearray(out_len)
    i = o = 0
    n = len(data)
    while i < n:
        ctrl = data[i]
        i += 1
        if ctrl < 32:  # literal run
            run = ctrl + 1
            out[o:o + run] = data[i:i + run]
            i += run
            o += run
        else:  # back reference
            length = ctrl >> 5
            if length == 7:
                length += data[i]
                i += 1
            ref = o - ((ctrl & 0x1f) << 8) - data[i] - 1
            i += 1
            length += 2
            for _ in range(length):
                out[o] = out[ref]
                o += 1
                ref += 1
    return bytes(out[:o])


def load_pcd(path):
    """Return an (n, 3) float array. Supports the DATA kinds PCL writes for x y z float fields."""
    with open(path, "rb") as f:
        raw = f.read()
    pos = 0
    header = {}
    while True:
        end = raw.index(b"\n", pos)
        line = raw[pos:end].decode("ascii").strip()
        pos = end + 1
        if line.startswith("#") or not line:
            continue
        key, _, value = line.partition(" ")
        header[key] = value
        if key == "DATA":
            break
    n = int(header["POINTS"])
    kind = header["DATA"].strip()
    if kind == "ascii":
        return np.loadtxt(raw[pos:].decode("ascii").splitlines()).reshape(-1, 3)
    if kind == "binary":
        return np.frombuffer(raw, dtype="<f4", count=n * 3, offset=pos).reshape(-1, 3).astype(float)
    if kind == "binary_compressed":
        comp, full = struct.unpack("<II", raw[pos:pos + 8])
        data = lzf_decompress(raw[pos + 8:pos + 8 + comp], full)
        # fields are stored column by column in this format
        return np.frombuffer(data, dtype="<f4", count=n * 3).reshape(3, n).T.astype(float)
    raise ValueError("unsupported PCD data kind: " + kind)


def occupancy_from_points(points):
    """2D grid[ix, iy] = True when OccMap would mark that column occupied above the floor."""
    grid = np.zeros((GRID, GRID), dtype=bool)
    ij = np.floor((points[:, :2] - ORIGIN) / RES).astype(int)
    k = np.floor((points[:, 2] - Z_ORIGIN) / RES).astype(int)
    keep = (ij >= 0).all(axis=1) & (ij < GRID).all(axis=1) & (k >= 1) & (k < Z_LAYERS)
    grid[ij[keep, 0], ij[keep, 1]] = True
    # OccMap::init marks the outer ring of cells as occupied
    grid[0, :] = grid[-1, :] = grid[:, 0] = grid[:, -1] = True
    return grid


def load_occupancy(path):
    return occupancy_from_points(load_pcd(path))


def disc_stencil(radius):
    """Cell offsets whose square lies closer than `radius` to the square of cell (0, 0)."""
    reach = int(np.ceil(radius / RES)) + 1
    offsets = []
    for dx in range(-reach, reach + 1):
        for dy in range(-reach, reach + 1):
            gx = max(0, abs(dx) - 1) * RES
            gy = max(0, abs(dy) - 1) * RES
            if np.hypot(gx, gy) < radius - 1e-9:
                offsets.append((dx, dy))
    return offsets


def inflate(grid, radius):
    out = grid.copy()
    for dx, dy in disc_stencil(radius):
        xs, xd = (slice(0, GRID - dx), slice(dx, GRID)) if dx >= 0 else (slice(-dx, GRID), slice(0, GRID + dx))
        ys, yd = (slice(0, GRID - dy), slice(dy, GRID)) if dy >= 0 else (slice(-dy, GRID), slice(0, GRID + dy))
        out[xd, yd] |= grid[xs, ys]
    return out


def block_point(grid, p, radius):
    """Mark every cell whose square lies closer than `radius` to point p."""
    lo = np.floor((np.asarray(p) - radius - ORIGIN) / RES).astype(int)
    hi = np.floor((np.asarray(p) + radius - ORIGIN) / RES).astype(int)
    lo = np.clip(lo, 0, GRID - 1)
    hi = np.clip(hi, 0, GRID - 1)
    for ix in range(lo[0], hi[0] + 1):
        x0 = ORIGIN[0] + ix * RES
        gx = max(x0 - p[0], 0.0, p[0] - (x0 + RES))
        for iy in range(lo[1], hi[1] + 1):
            y0 = ORIGIN[1] + iy * RES
            gy = max(y0 - p[1], 0.0, p[1] - (y0 + RES))
            if gx * gx + gy * gy < radius * radius:
                grid[ix, iy] = True


def label_free(grid):
    """4-connected labels of the free cells, using union-find over vertical runs. -1 = occupied.

    Cells that only touch at a corner leave a passage of no width, so they do not connect.
    """
    parent = []

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    labels = -np.ones(grid.shape, dtype=np.int32)
    prev_runs = []
    for ix in range(grid.shape[0]):
        col = ~grid[ix]
        edges = np.flatnonzero(np.diff(np.concatenate(([0], col.astype(np.int8), [0]))))
        runs = []
        j = 0
        for s, e in zip(edges[::2], edges[1::2]):  # free cells s .. e-1
            rid = len(parent)
            parent.append(rid)
            runs.append((s, e, rid))
            # a run of the previous column connects when it shares at least one row with this one
            while j < len(prev_runs) and prev_runs[j][1] <= s:
                j += 1
            m = j
            while m < len(prev_runs) and prev_runs[m][0] < e:
                ra, rb = find(prev_runs[m][2]), find(rid)
                if ra != rb:
                    parent[rb] = ra
                m += 1
        for s, e, rid in runs:
            labels[ix, s:e] = rid
        prev_runs = runs
    if parent:
        roots = np.array([find(i) for i in range(len(parent))], dtype=np.int32)
        mask = labels >= 0
        labels[mask] = roots[labels[mask]]
    return labels


def cell_of(p):
    return tuple(np.floor((np.asarray(p[:2]) - ORIGIN) / RES).astype(int))


def load_json(path):
    with open(path) as f:
        return json.load(f)
