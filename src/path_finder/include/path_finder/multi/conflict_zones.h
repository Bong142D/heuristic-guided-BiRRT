// Conflict zones between robot paths: the stretches where two paths run closer than D = 2r + m.
// These are what the coordination stage has to order. Nothing here depends on time or speed.
#ifndef MULTI_CONFLICT_ZONES_H
#define MULTI_CONFLICT_ZONES_H

#include "path_finder/multi/path_utils.h"

#include <cstdint>
#include <limits>
#include <unordered_map>

namespace multi
{
  struct ConflictZone
  {
    int a = 0, b = 0;                // robots, a < b
    double a_in = 0.0, a_out = 0.0;  // arc-length interval on a's path
    double b_in = 0.0, b_out = 0.0;  // arc-length interval on b's path
    double min_dist = 0.0;           // closest approach inside the zone
    int direction = 0;               // +1 same direction, -1 head-on, 0 they only cross
    bool start_a = false, goal_a = false, start_b = false, goal_b = false; // endpoint inside the zone

    bool hasEndpoint() const { return start_a || goal_a || start_b || goal_b; }
    double length() const { return 0.5 * ((a_out - a_in) + (b_out - b_in)); }
  };

  struct ZoneStats
  {
    int n_zones = 0;
    int conflict_pairs = 0;
    int endpoint_zones = 0;
    int head_on_zones = 0;
    int same_dir_zones = 0;
    double zone_length = 0.0;      // sum of the zone lengths, averaged over both paths of each zone
    double overlap_fraction = 0.0; // share of the total path length that lies in at least one zone
    double min_dist = std::numeric_limits<double>::infinity(); // closest approach over all zones
  };

  namespace detail
  {
    struct Sampled
    {
      Path2 points;
      std::vector<double> arcs;
      std::unordered_map<std::uint64_t, std::vector<int>> cells; // spatial hash, cell size D
      Eigen::Vector2d lo, hi;
    };

    inline std::uint64_t cellKey(int x, int y)
    {
      return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32) | static_cast<std::uint32_t>(y);
    }

    inline Sampled sample(const Path2 &path, double D, double step)
    {
      Sampled s;
      resample(path, step, s.points, s.arcs);
      s.lo = Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity());
      s.hi = -s.lo;
      for (size_t i = 0; i < s.points.size(); ++i)
      {
        const Eigen::Vector2d &p = s.points[i];
        s.lo = s.lo.cwiseMin(p);
        s.hi = s.hi.cwiseMax(p);
        s.cells[cellKey(static_cast<int>(std::floor(p.x() / D)), static_cast<int>(std::floor(p.y() / D)))].push_back(static_cast<int>(i));
      }
      return s;
    }

    struct Run
    {
      int i0, i1, j0, j1;
      double min_dist;
    };

    inline void zonesOfPair(const Sampled &A, const Sampled &B, int ia, int ib, double D, std::vector<ConflictZone> &out)
    {
      if (A.points.empty() || B.points.empty())
        return;
      if (A.lo.x() - D > B.hi.x() || B.lo.x() - D > A.hi.x() || A.lo.y() - D > B.hi.y() || B.lo.y() - D > A.hi.y())
        return;

      const int n = static_cast<int>(A.points.size());
      std::vector<int> nearest(n, -1), jlo(n, 0), jhi(n, 0);
      std::vector<double> dist(n, 0.0);
      for (int i = 0; i < n; ++i)
      {
        const Eigen::Vector2d &p = A.points[i];
        const int cx = static_cast<int>(std::floor(p.x() / D)), cy = static_cast<int>(std::floor(p.y() / D));
        double best = D;
        for (int dx = -1; dx <= 1; ++dx)
          for (int dy = -1; dy <= 1; ++dy)
          {
            auto it = B.cells.find(cellKey(cx + dx, cy + dy));
            if (it == B.cells.end())
              continue;
            for (int j : it->second)
            {
              double d = (B.points[j] - p).norm();
              if (d >= D)
                continue;
              if (nearest[i] < 0)
              {
                jlo[i] = jhi[i] = j;
                nearest[i] = j;
                best = d;
              }
              else
              {
                jlo[i] = std::min(jlo[i], j);
                jhi[i] = std::max(jhi[i], j);
                if (d < best)
                {
                  best = d;
                  nearest[i] = j;
                }
              }
            }
          }
        dist[i] = best;
      }

      // contiguous stretches on A, merged when they also touch on B: a robot weaving along the
      // edge of the other path is one zone, not many
      std::vector<Run> runs;
      for (int i = 0; i < n; ++i)
      {
        if (nearest[i] < 0)
          continue;
        const bool extend = !runs.empty() && runs.back().i1 == i - 1;
        if (!runs.empty() && !extend)
        {
          const Run &r = runs.back();
          double gap_a = A.arcs[i] - A.arcs[r.i1];
          double gap_b = 0.0;
          if (jlo[i] > r.j1)
            gap_b = B.arcs[jlo[i]] - B.arcs[r.j1];
          else if (jhi[i] < r.j0)
            gap_b = B.arcs[r.j0] - B.arcs[jhi[i]];
          if (gap_a >= D || gap_b >= D)
            runs.push_back({i, i, jlo[i], jhi[i], dist[i]});
          else
            runs.back().i1 = i;
        }
        else if (runs.empty())
        {
          runs.push_back({i, i, jlo[i], jhi[i], dist[i]});
        }
        else
        {
          runs.back().i1 = i;
        }
        Run &r = runs.back();
        r.j0 = std::min(r.j0, jlo[i]);
        r.j1 = std::max(r.j1, jhi[i]);
        r.min_dist = std::min(r.min_dist, dist[i]);
      }

      for (const Run &r : runs)
      {
        ConflictZone z;
        z.a = ia;
        z.b = ib;
        z.a_in = A.arcs[r.i0];
        z.a_out = A.arcs[r.i1];
        z.b_in = B.arcs[r.j0];
        z.b_out = B.arcs[r.j1];
        z.min_dist = r.min_dist;
        z.start_a = r.i0 == 0;
        z.goal_a = r.i1 == n - 1;
        z.start_b = r.j0 == 0;
        z.goal_b = r.j1 == static_cast<int>(B.points.size()) - 1;

        // Two straight paths crossing at an angle t stay within D of each other for 2 D / sin(t).
        // Only a zone longer than 5 D, so an angle under about 25 degrees, means they really run
        // alongside each other. There the correlation of the positions along both paths gives the direction.
        if (z.a_out - z.a_in >= 5.0 * D && z.b_out - z.b_in >= 5.0 * D)
        {
          double si = 0, sj = 0, sii = 0, sjj = 0, sij = 0;
          int m = 0;
          for (int i = r.i0; i <= r.i1; ++i)
          {
            if (nearest[i] < 0)
              continue;
            double x = A.arcs[i], y = B.arcs[nearest[i]];
            si += x;
            sj += y;
            sii += x * x;
            sjj += y * y;
            sij += x * y;
            ++m;
          }
          double cov = sij - si * sj / m;
          double var = std::sqrt(std::max(0.0, sii - si * si / m) * std::max(0.0, sjj - sj * sj / m));
          double corr = var > 1e-12 ? cov / var : 0.0;
          z.direction = corr > 0.5 ? 1 : (corr < -0.5 ? -1 : 0);
        }
        out.push_back(z);
      }
    }
  } // namespace detail

  // step: spacing of the points compared along each path. Using points instead of the exact
  // polylines overestimates a distance by at most step^2 / (8 D), about 2 mm with the defaults.
  inline std::vector<ConflictZone> findConflictZones(const std::vector<Path2> &paths, double D, double step = 0.1)
  {
    std::vector<detail::Sampled> sampled;
    sampled.reserve(paths.size());
    for (const auto &p : paths)
      sampled.push_back(detail::sample(p, D, step));

    std::vector<ConflictZone> zones;
    for (size_t a = 0; a < paths.size(); ++a)
      for (size_t b = a + 1; b < paths.size(); ++b)
        detail::zonesOfPair(sampled[a], sampled[b], static_cast<int>(a), static_cast<int>(b), D, zones);
    return zones;
  }

  inline ZoneStats summarizeZones(const std::vector<ConflictZone> &zones, const std::vector<Path2> &paths)
  {
    ZoneStats st;
    st.n_zones = static_cast<int>(zones.size());
    std::vector<std::vector<std::pair<double, double>>> covered(paths.size());
    std::vector<std::pair<int, int>> pairs;
    for (const auto &z : zones)
    {
      st.endpoint_zones += z.hasEndpoint();
      st.head_on_zones += z.direction < 0;
      st.same_dir_zones += z.direction > 0;
      st.zone_length += z.length();
      st.min_dist = std::min(st.min_dist, z.min_dist);
      pairs.emplace_back(z.a, z.b);
      covered[z.a].emplace_back(z.a_in, z.a_out);
      covered[z.b].emplace_back(z.b_in, z.b_out);
    }
    std::sort(pairs.begin(), pairs.end());
    st.conflict_pairs = static_cast<int>(std::unique(pairs.begin(), pairs.end()) - pairs.begin());

    double inside = 0.0, total = 0.0;
    for (size_t i = 0; i < paths.size(); ++i)
    {
      total += pathLength(paths[i]);
      auto &iv = covered[i];
      std::sort(iv.begin(), iv.end());
      double end = -1.0;
      for (const auto &p : iv)
      {
        double from = std::max(p.first, end);
        if (p.second > from)
        {
          inside += p.second - from;
          end = p.second;
        }
      }
    }
    st.overlap_fraction = total > 0.0 ? inside / total : 0.0;
    return st;
  }
} // namespace multi

#endif
