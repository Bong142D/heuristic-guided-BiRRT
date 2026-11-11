// Checks conflict_zones.h on paths whose answer is known by hand. Exit code 0 = all passed.
#include "path_finder/multi/conflict_zones.h"

#include <cstdio>

using multi::ConflictZone;
using multi::Path2;
using Eigen::Vector2d;

static int failures = 0;

#define CHECK(cond)                                         \
  do                                                        \
  {                                                         \
    if (!(cond))                                            \
    {                                                       \
      std::printf("FAIL line %d: %s\n", __LINE__, #cond);   \
      ++failures;                                           \
    }                                                       \
  } while (0)

static bool near(double a, double b, double tol = 0.15) { return std::abs(a - b) <= tol; }

// straight line sampled like a planner path, every 0.5 m
static Path2 line(Vector2d a, Vector2d b)
{
  Path2 p;
  int n = std::max(1, static_cast<int>(std::ceil((b - a).norm() / 0.5)));
  for (int i = 0; i <= n; ++i)
    p.push_back(a + (b - a) * (static_cast<double>(i) / n));
  return p;
}

static const double D = 0.6;

static std::vector<ConflictZone> zonesOf(const Path2 &a, const Path2 &b) { return multi::findConflictZones({a, b}, D); }

int main()
{
  { // resampling keeps both ends and the arc lengths
    Path2 pts;
    std::vector<double> arcs;
    multi::resample({Vector2d(0, 0), Vector2d(1, 0), Vector2d(1, 1)}, 0.3, pts, arcs);
    CHECK(near(arcs.back(), 2.0, 1e-9));
    CHECK((pts.back() - Vector2d(1, 1)).norm() < 1e-9);
    for (size_t i = 1; i < arcs.size(); ++i)
      CHECK(arcs[i] > arcs[i - 1]);
    multi::resample({Vector2d(3, 4)}, 0.3, pts, arcs);
    CHECK(pts.size() == 1);
    CHECK(near(multi::pathLength({Vector2d(0, 0), Vector2d(3, 4)}), 5.0, 1e-9));
  }

  { // two paths crossing at a right angle: one short zone with no direction
    auto z = zonesOf(line({-10, 0}, {10, 0}), line({0, -10}, {0, 10}));
    CHECK(z.size() == 1);
    if (z.size() == 1)
    {
      CHECK(z[0].direction == 0);
      CHECK(near(z[0].a_in, 10 - D));
      CHECK(near(z[0].a_out, 10 + D));
      CHECK(z[0].min_dist < 0.1);
      CHECK(!z[0].hasEndpoint());
    }
  }

  { // crossing at 45 degrees is still a crossing, whichever way the robots go
    auto z = zonesOf(line({-10, 0}, {10, 0}), line({-7, -7}, {7, 7}));
    CHECK(z.size() == 1);
    if (z.size() == 1)
      CHECK(z[0].direction == 0);
    z = zonesOf(line({-10, 0}, {10, 0}), line({7, 7}, {-7, -7}));
    CHECK(z.size() == 1);
    if (z.size() == 1)
      CHECK(z[0].direction == 0);
  }

  { // head-on in a corridor, both ends of b lie inside the zone
    auto z = zonesOf(line({-10, 0}, {10, 0}), line({5, 0.3}, {-5, 0.3}));
    CHECK(z.size() == 1);
    if (z.size() == 1)
    {
      CHECK(z[0].direction == -1);
      double half = std::sqrt(D * D - 0.3 * 0.3);
      CHECK(near(z[0].a_in, 5 - half));
      CHECK(near(z[0].a_out, 15 + half));
      CHECK(z[0].start_b && z[0].goal_b);
      CHECK(!z[0].start_a && !z[0].goal_a);
    }
  }

  { // same direction, all ends inside
    Path2 a = line({-10, 0}, {10, 0}), b = line({-10, 0.3}, {10, 0.3});
    auto z = zonesOf(a, b);
    CHECK(z.size() == 1);
    if (z.size() == 1)
    {
      CHECK(z[0].direction == 1);
      CHECK(z[0].start_a && z[0].goal_a && z[0].start_b && z[0].goal_b);
      auto st = multi::summarizeZones(z, {a, b});
      CHECK(near(st.overlap_fraction, 1.0, 0.01));
      CHECK(st.same_dir_zones == 1);
    }
  }

  { // parallel at 1.5 D: no zone. Just inside D: a zone
    CHECK(zonesOf(line({-10, 0}, {10, 0}), line({-10, 1.5 * D}, {10, 1.5 * D})).empty());
    CHECK(zonesOf(line({-10, 0}, {10, 0}), line({-10, 0.95 * D}, {10, 0.95 * D})).size() == 1);
  }

  { // the goal of b lies on a's path
    auto z = zonesOf(line({-10, 0}, {10, 0}), line({0, 10}, {0, 0.1}));
    CHECK(z.size() == 1);
    if (z.size() == 1)
    {
      CHECK(z[0].goal_b);
      CHECK(!z[0].start_b);
      CHECK(!z[0].start_a && !z[0].goal_a);
    }
  }

  { // a U-shaped path crosses twice, far apart: two zones in order along both paths
    Path2 u = line({-5, 10}, {-5, -10});
    Path2 back = line({-5, -10}, {5, -10});
    Path2 up = line({5, -10}, {5, 10});
    u.insert(u.end(), back.begin() + 1, back.end());
    u.insert(u.end(), up.begin() + 1, up.end());
    auto z = zonesOf(line({-20, 0}, {20, 0}), u);
    CHECK(z.size() == 2);
    if (z.size() == 2)
    {
      CHECK(z[0].a_out < z[1].a_in);
      CHECK(z[0].b_out < z[1].b_in);
    }
    auto st = multi::summarizeZones(z, {line({-20, 0}, {20, 0}), u});
    CHECK(st.conflict_pairs == 1);
    CHECK(st.n_zones == 2);
  }

  { // weaving in and out along the other path counts as one zone
    Path2 w;
    for (int i = 0; i <= 40; ++i)
      w.push_back(Vector2d(-10 + i * 0.5, (i % 2) ? 0.65 : 0.5));
    CHECK(zonesOf(line({-10, 0}, {10, 0}), w).size() == 1);
  }

  { // zones are counted per pair
    Path2 a = line({-10, 0}, {10, 0}), b = line({0, -10}, {0, 10}), c = line({5, -10}, {5, 10});
    auto z = multi::findConflictZones({a, b, c}, D);
    auto st = multi::summarizeZones(z, {a, b, c});
    CHECK(st.n_zones == 2);
    CHECK(st.conflict_pairs == 2);
    CHECK(z[0].a == 0 && z[0].b == 1);
    CHECK(z[1].a == 0 && z[1].b == 2);
    // a is covered twice over 2 D, b and c once each
    CHECK(near(st.overlap_fraction, (4 * D + 2 * D + 2 * D) / 60.0, 0.02));
  }

  { // overlapping zones on one path are not counted twice in the overlap: on a, the zones with
    // b and c cover 1.3 m together, not 2.4 m. b and c run side by side, so they are fully covered.
    Path2 a = line({-10, 0}, {10, 0}), b = line({0, -10}, {0, 10}), c = line({0.1, -10}, {0.1, 10});
    auto st = multi::summarizeZones(multi::findConflictZones({a, b, c}, D), {a, b, c});
    CHECK(st.conflict_pairs == 3);
    CHECK(near(st.overlap_fraction, (1.3 + 40.0) / 60.0, 0.005));
  }

  { // turning angle
    auto t = multi::pathTurning({Vector2d(0, 0), Vector2d(1, 0), Vector2d(1, 1)});
    CHECK(near(t.total, M_PI / 2, 1e-9));
    CHECK(near(t.max, M_PI / 2, 1e-9));
  }

  if (failures == 0)
    std::printf("all conflict zone tests passed\n");
  return failures == 0 ? 0 : 1;
}
