// Geometry of one robot's path on the XY plane. No ROS here, so the tests run without a roscore.
#ifndef MULTI_PATH_UTILS_H
#define MULTI_PATH_UTILS_H

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <vector>

namespace multi
{
  using Path2 = std::vector<Eigen::Vector2d>;

  // Planner nodes can sit on different z layers, so everything multi-robot is measured on XY.
  inline Path2 toXY(const std::vector<Eigen::Vector3d> &path)
  {
    Path2 out;
    out.reserve(path.size());
    for (const auto &p : path)
    {
      Eigen::Vector2d q = p.head<2>();
      if (out.empty() || (q - out.back()).norm() > 1e-9)
        out.push_back(q);
    }
    return out;
  }

  inline double pathLength(const Path2 &path)
  {
    double len = 0.0;
    for (size_t i = 1; i < path.size(); ++i)
      len += (path[i] - path[i - 1]).norm();
    return len;
  }

  struct Turning
  {
    double total = 0.0; // sum of the turning angles at the vertices, rad
    double max = 0.0;   // largest single turning angle, rad
  };

  inline Turning pathTurning(const Path2 &path)
  {
    Turning t;
    for (size_t i = 1; i + 1 < path.size(); ++i)
    {
      Eigen::Vector2d u = path[i] - path[i - 1], v = path[i + 1] - path[i];
      double angle = std::atan2(std::abs(u.x() * v.y() - u.y() * v.x()), u.dot(v));
      t.total += angle;
      t.max = std::max(t.max, angle);
    }
    return t;
  }

  // Points every `step` along the path, always including both ends. arcs[i] is the arc length of points[i].
  inline void resample(const Path2 &path, double step, Path2 &points, std::vector<double> &arcs)
  {
    points.clear();
    arcs.clear();
    if (path.empty())
      return;
    points.push_back(path.front());
    arcs.push_back(0.0);
    double walked = 0.0, next = step;
    for (size_t i = 1; i < path.size(); ++i)
    {
      Eigen::Vector2d d = path[i] - path[i - 1];
      double len = d.norm();
      while (next < walked + len)
      {
        points.push_back(path[i - 1] + d * ((next - walked) / len));
        arcs.push_back(next);
        next += step;
      }
      walked += len;
    }
    if (path.size() > 1 && walked > arcs.back() + 1e-9)
    {
      points.push_back(path.back());
      arcs.push_back(walked);
    }
  }
} // namespace multi

#endif
