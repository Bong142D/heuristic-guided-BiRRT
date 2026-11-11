/*
Copyright (C) 2022 Hongkai Ye (kyle_yeh@163.com)
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/
#include "occ_grid/occ_map.h"
#include "occ_grid/raycast.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <random>
#include <queue>
#include <set>

namespace env
{
  Eigen::Vector3d OccMap::findNearestFree(const Eigen::Vector3d &pos)
  {
    if (isStateValid(pos))
      return pos;

    Eigen::Vector3i start_idx = posToIndex(pos);
    std::queue<Eigen::Vector3i> q;
    q.push(start_idx);

    std::set<std::vector<int>> visited;
    visited.insert({start_idx[0], start_idx[1], start_idx[2]});

    int max_iterations = 10000; // Safety limit
    int iterations = 0;

    while (!q.empty() && iterations < max_iterations)
    {
      Eigen::Vector3i curr = q.front();
      q.pop();
      iterations++;

      // Check 8-neighbors (X-Y plane only)
      for (int dx = -1; dx <= 1; ++dx)
      {
        for (int dy = -1; dy <= 1; ++dy)
        {
          if (dx == 0 && dy == 0)
            continue;

          Eigen::Vector3i next = curr + Eigen::Vector3i(dx, dy, 0);
          if (!isInMap(next))
            continue;

          std::vector<int> next_vec = {next[0], next[1], next[2]};
          if (visited.find(next_vec) != visited.end())
            continue;
          visited.insert(next_vec);

          if (occupancy_buffer_[idxToAddress(next)] == false)
          {
            Eigen::Vector3d free_pos;
            indexToPos(next, free_pos);
            free_pos.z() = pos.z(); // Keep exact Z value
            return free_pos;
          }
          q.push(next);
        }
      }
    }

    ROS_WARN("[OccMap]: Could not find a free neighbor for start/goal point.");
    return pos;
  }

  inline void OccMap::setOccupancy(const Eigen::Vector3d &pos)
  {
    Eigen::Vector3i id;
    posToIndex(pos, id);
    // cout << "id: " << id.transpose() << ", idx: " << idxToAddress(id) << ", is in map? " << isInMap(id) << endl;
    if (!isInMap(id))
      return;

    occupancy_buffer_[idxToAddress(id)] = true;
  }

  void OccMap::globalOccVisCallback(const ros::TimerEvent &e)
  {
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*glb_cloud_ptr_, cloud_msg);
    glb_occ_pub_.publish(cloud_msg);
  }

  void OccMap::globalCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
  {
    if (is_global_map_valid_)
      return;

    pcl::PointCloud<pcl::PointXYZ> global_cloud;
    pcl::fromROSMsg(*msg, global_cloud);
    // ROS_ERROR_STREAM(", global_cloud.points.size(): " << global_cloud.points.size());

    if (global_cloud.points.size() == 0)
      return;

    pcl::PointXYZ pt;
    Eigen::Vector3d p3d;
    for (size_t i = 0; i < global_cloud.points.size(); ++i)
    {
      pt = global_cloud.points[i];
      p3d(0) = pt.x;
      p3d(1) = pt.y;
      p3d(2) = pt.z;
      this->setOccupancy(p3d);
    }
    is_global_map_valid_ = true;

    glb_cloud_ptr_->points.clear();
    for (int x = 0; x < grid_size_[0]; ++x)
      for (int y = 0; y < grid_size_[1]; ++y)
        for (int z = 0; z < grid_size_[2]; ++z)
        {
          if (occupancy_buffer_[idxToAddress(x, y, z)] == true)
          {
            Eigen::Vector3d pos;
            indexToPos(x, y, z, pos);
            glb_cloud_ptr_->points.emplace_back(pos[0], pos[1], pos[2]);
          }
        }
    glb_cloud_ptr_->width = glb_cloud_ptr_->points.size();
    glb_cloud_ptr_->height = 1;
    glb_cloud_ptr_->is_dense = true;
    glb_cloud_ptr_->header.frame_id = "map";

    cout << "glb occ set" << endl;
    global_cloud_sub_.shutdown();
  }

  bool OccMap::columnOccupied(int x, int y) const
  {
    for (int z = 1; z < grid_size_(2); ++z)
      if (occupancy_buffer_[idxToAddress(x, y, z)])
        return true;
    return false;
  }

  void OccMap::setColumn(int x, int y)
  {
    for (int z = 1; z < grid_size_(2); ++z)
      occupancy_buffer_[idxToAddress(x, y, z)] = true;
  }

  void OccMap::inflateXY(double radius)
  {
    // gap between the squares of two cells that are (dx, dy) apart
    const int reach = (int)ceil(radius * resolution_inv_) + 1;
    std::vector<Eigen::Vector2i> stencil;
    for (int dx = -reach; dx <= reach; ++dx)
      for (int dy = -reach; dy <= reach; ++dy)
      {
        double gx = max(0, abs(dx) - 1) * resolution_;
        double gy = max(0, abs(dy) - 1) * resolution_;
        if (gx * gx + gy * gy < radius * radius)
          stencil.emplace_back(dx, dy);
      }

    std::vector<char> occupied(grid_size_(0) * grid_size_(1));
    for (int x = 0; x < grid_size_(0); ++x)
      for (int y = 0; y < grid_size_(1); ++y)
        occupied[x * grid_size_(1) + y] = columnOccupied(x, y);

    for (int x = 0; x < grid_size_(0); ++x)
      for (int y = 0; y < grid_size_(1); ++y)
      {
        if (!occupied[x * grid_size_(1) + y])
          continue;
        for (const auto &d : stencil)
        {
          int nx = x + d(0), ny = y + d(1);
          if (nx >= 0 && nx < grid_size_(0) && ny >= 0 && ny < grid_size_(1))
            setColumn(nx, ny);
        }
      }
  }

  void OccMap::saveBase()
  {
    base_buffer_ = occupancy_buffer_;
  }

  void OccMap::restoreBase()
  {
    occupancy_buffer_ = base_buffer_;
  }

  static double pointSquareDist(const Eigen::Vector2d &p, const Eigen::Vector2d &lo, const Eigen::Vector2d &hi)
  {
    double gx = max(max(lo(0) - p(0), 0.0), p(0) - hi(0));
    double gy = max(max(lo(1) - p(1), 0.0), p(1) - hi(1));
    return std::hypot(gx, gy);
  }

  static double pointSegmentDist(const Eigen::Vector2d &p, const Eigen::Vector2d &a, const Eigen::Vector2d &b)
  {
    Eigen::Vector2d ab = b - a;
    double len2 = ab.squaredNorm();
    double t = len2 > 1e-18 ? std::min(1.0, std::max(0.0, (p - a).dot(ab) / len2)) : 0.0;
    return (a + t * ab - p).norm();
  }

  static double segmentSquareDist(const Eigen::Vector2d &a, const Eigen::Vector2d &b, const Eigen::Vector2d &lo, const Eigen::Vector2d &hi)
  {
    // the segment enters the square: Liang-Barsky clipping
    Eigen::Vector2d d = b - a;
    double t0 = 0.0, t1 = 1.0;
    bool hit = true;
    for (int i = 0; i < 2 && hit; ++i)
    {
      if (std::abs(d(i)) < 1e-12)
      {
        hit = a(i) >= lo(i) && a(i) <= hi(i);
        continue;
      }
      double ta = (lo(i) - a(i)) / d(i), tb = (hi(i) - a(i)) / d(i);
      if (ta > tb)
        std::swap(ta, tb);
      t0 = max(t0, ta);
      t1 = min(t1, tb);
      hit = t0 <= t1;
    }
    if (hit)
      return 0.0;
    // otherwise the closest pair involves an end of the segment or a corner of the square
    double best = min(pointSquareDist(a, lo, hi), pointSquareDist(b, lo, hi));
    best = min(best, pointSegmentDist(lo, a, b));
    best = min(best, pointSegmentDist(hi, a, b));
    best = min(best, pointSegmentDist(Eigen::Vector2d(lo(0), hi(1)), a, b));
    best = min(best, pointSegmentDist(Eigen::Vector2d(hi(0), lo(1)), a, b));
    return best;
  }

  void OccMap::blockPath(const std::vector<Eigen::Vector3d> &path, double radius)
  {
    for (size_t k = 0; k < path.size(); ++k)
    {
      Eigen::Vector2d a = path[k].head<2>();
      Eigen::Vector2d b = path[k + 1 < path.size() ? k + 1 : k].head<2>();
      int x0 = max(0, (int)floor((min(a(0), b(0)) - radius - origin_(0)) * resolution_inv_));
      int x1 = min(grid_size_(0) - 1, (int)floor((max(a(0), b(0)) + radius - origin_(0)) * resolution_inv_));
      int y0 = max(0, (int)floor((min(a(1), b(1)) - radius - origin_(1)) * resolution_inv_));
      int y1 = min(grid_size_(1) - 1, (int)floor((max(a(1), b(1)) + radius - origin_(1)) * resolution_inv_));
      for (int x = x0; x <= x1; ++x)
        for (int y = y0; y <= y1; ++y)
        {
          Eigen::Vector2d lo(origin_(0) + x * resolution_, origin_(1) + y * resolution_);
          Eigen::Vector2d hi(lo(0) + resolution_, lo(1) + resolution_);
          if (segmentSquareDist(a, b, lo, hi) < radius)
            setColumn(x, y);
        }
    }
  }

  void OccMap::blockDisc(const Eigen::Vector3d &center, double radius)
  {
    blockPath(std::vector<Eigen::Vector3d>{center}, radius);
  }

  bool OccMap::connectedXY(const Eigen::Vector3d &a, const Eigen::Vector3d &b) const
  {
    Eigen::Vector3i ia = posToIndex(a), ib = posToIndex(b);
    const int nx = grid_size_(0), ny = grid_size_(1);
    if (ia(0) < 0 || ia(0) >= nx || ia(1) < 0 || ia(1) >= ny || ib(0) < 0 || ib(0) >= nx || ib(1) < 0 || ib(1) >= ny)
      return false;
    if (columnOccupied(ia(0), ia(1)) || columnOccupied(ib(0), ib(1)))
      return false;

    // 4 neighbours: two cells that share an edge leave a passage of real width. Cells that only
    // touch at a corner leave none, and no planner gets through there.
    static const int steps[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    std::vector<char> seen(nx * ny, 0);
    std::vector<int> stack{ia(0) * ny + ia(1)};
    seen[stack.back()] = 1;
    const int target = ib(0) * ny + ib(1);
    while (!stack.empty())
    {
      int cur = stack.back();
      stack.pop_back();
      if (cur == target)
        return true;
      int cx = cur / ny, cy = cur % ny;
      for (const auto &step : steps)
      {
        int x = cx + step[0], y = cy + step[1];
        if (x < 0 || x >= nx || y < 0 || y >= ny || seen[x * ny + y] || columnOccupied(x, y))
          continue;
        seen[x * ny + y] = 1;
        stack.push_back(x * ny + y);
      }
    }
    return false;
  }

  void OccMap::init(const ros::NodeHandle &nh)
  {
    node_ = nh;
    /* ---------- param ---------- */
    node_.param("occ_map/origin_x", origin_(0), -50.0);
    node_.param("occ_map/origin_y", origin_(1), -50.0);
    node_.param("occ_map/origin_z", origin_(2), -1.0);
    node_.param("occ_map/map_size_x", map_size_(0), 100.0);
    node_.param("occ_map/map_size_y", map_size_(1), 100.0);
    node_.param("occ_map/map_size_z", map_size_(2), 1.5);
    node_.param("occ_map/resolution", resolution_, 0.5);
    resolution_inv_ = 1 / resolution_;

    is_global_map_valid_ = false;

    for (int i = 0; i < 3; ++i)
    {
      grid_size_(i) = ceil(map_size_(i) * resolution_inv_);
    }

    min_range_ = origin_;
    max_range_ = origin_ + map_size_;

    // initialize size of buffer
    grid_size_y_multiply_z_ = grid_size_(1) * grid_size_(2);
    int buffer_size = grid_size_(0) * grid_size_y_multiply_z_;
    occupancy_buffer_.resize(buffer_size);
    fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), false);

    //set x-y boundary occ
    for (double cx = min_range_[0] + resolution_ / 2; cx <= max_range_[0] - resolution_ / 2; cx += resolution_)
      for (double cz = min_range_[2] + resolution_ / 2; cz <= max_range_[2] - resolution_ / 2; cz += resolution_)
      {
        this->setOccupancy(Eigen::Vector3d(cx, min_range_[1] + resolution_ / 2, cz));
        this->setOccupancy(Eigen::Vector3d(cx, max_range_[1] - resolution_ / 2, cz));
      }
    for (double cy = min_range_[1] + resolution_ / 2; cy <= max_range_[1] - resolution_ / 2; cy += resolution_)
      for (double cz = min_range_[2] + resolution_ / 2; cz <= max_range_[2] - resolution_ / 2; cz += resolution_)
      {
        this->setOccupancy(Eigen::Vector3d(min_range_[0] + resolution_ / 2, cy, cz));
        this->setOccupancy(Eigen::Vector3d(max_range_[0] - resolution_ / 2, cy, cz));
      }
    //set z-low boundary occ
    for (double cx = min_range_[0] + resolution_ / 2; cx <= max_range_[0] - resolution_ / 2; cx += resolution_)
      for (double cy = min_range_[1] + resolution_ / 2; cy <= max_range_[1] - resolution_ / 2; cy += resolution_)
      {
        this->setOccupancy(Eigen::Vector3d(cx, cy, min_range_[2] + resolution_ / 2));
      }

    global_occ_vis_timer_ = node_.createTimer(ros::Duration(5), &OccMap::globalOccVisCallback, this);
    global_cloud_sub_ = node_.subscribe<sensor_msgs::PointCloud2>("/global_cloud", 1, &OccMap::globalCloudCallback, this);
    glb_occ_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/occ_map/glb_map", 1);

    glb_cloud_ptr_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cout << "map initialized: " << endl;
  }

} // namespace env
