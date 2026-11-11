// Stage 1 of the multi-robot plan: geometric paths for N robots on a static map, and the conflict
// zones between them that the coordination stage will have to order. No time or speed here.
//
// A method is "<planner>+<mode>":
//   planner: connect (RRT-Connect), fconnect (RRT-FConnect)
//   mode:    independent  each robot alone on the static map; paths may touch anything
//            reserve      also keeps away from the start and goal of every other robot
//            separated    reserve + earlier paths become obstacles, so that no two paths come closer than D
#include "occ_grid/occ_map.h"
#include "path_finder/brrt.h"
#include "path_finder/SOF_brrt_simple_case3.h"
#include "path_finder/testcase.h"
#include "path_finder/multi/conflict_zones.h"
#include <ros/ros.h>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <numeric>
#include <random>
#include <sstream>

using multi::Path2;

namespace
{
  struct PlanCall
  {
    bool ok = false;
    double time = 0.0;
    int nodes = 0;
    int iterations = 0;
    std::vector<Eigen::Vector3d> path;
  };

  struct Method
  {
    std::string name;
    bool fconnect = false;
    std::string mode;
  };

  struct RobotResult
  {
    PlanCall call;
    Path2 path;
    bool planned = false;
  };

  double seconds(std::chrono::steady_clock::time_point since)
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - since).count();
  }

  double round4(double x) { return std::round(x * 1e4) / 1e4; }
}

class MultiRobotRunner
{
public:
  explicit MultiRobotRunner(const ros::NodeHandle &nh) : nh_(nh)
  {
    map_ = std::make_shared<env::OccMap>();
    map_->init(nh_);

    std::string input_param, methods;
    nh_.param("input_param", input_param, std::string(""));
    nh_.param("scenarios_file", scenarios_file_, std::string(""));
    nh_.param("output_result", output_result_, std::string(""));
    nh_.param("self_test_output", self_test_output_, std::string(""));
    nh_.param("methods", methods, std::string("fconnect+independent"));
    nh_.param("robot_radius", radius_, 0.25);
    nh_.param("margin", margin_, 0.1);
    nh_.param("trials", trials_, 1);
    nh_.param("seed", seed_, 1);
    nh_.param("time_limit_call", time_limit_call_, 5.0);
    nh_.param("time_limit_total", time_limit_total_, 60.0);
    nh_.param("zone_step", zone_step_, 0.1);
    nh_.param("path_trials", path_trials_, 1);
    nh_.param("separated_max_n", separated_max_n_, 1000);
    D_ = 2.0 * radius_ + margin_;

    std::stringstream names(methods);
    std::string item;
    while (std::getline(names, item, ','))
    {
      auto plus = item.find('+');
      Method m;
      m.name = item;
      m.fconnect = item.substr(0, plus) == "fconnect";
      m.mode = item.substr(plus + 1);
      methods_.push_back(m);
    }

    BRRTExperimentMultiAlgo config(input_param, "");
    input_ = config.get_input();

    connect_ = std::make_shared<path_plan::BRRT>(nh_, map_);
    connect_->set_test_param(input_.epsilon);
    connect_->set_time_limit(time_limit_call_);

    fconnect_ = std::make_shared<path_plan::SOF_BRRT_Simple_Case3>(nh_, map_);
    fconnect_->set_heuristic_param(input_.p1, input_.u_p, input_.alpha, input_.beta, input_.gamma, input_.epsilon, input_.p2, input_.n_rays, input_.weight_grade, input_.ray_step_size);
    fconnect_->set_variant(input_.r_max, input_.exploration, input_.knn_k, input_.knn_exact, input_.pool_size, input_.pop_guide);
    fconnect_->set_sampling_variant(input_.reject_invalid_sample, input_.apf_always, input_.explore_trigger, input_.explore_len);
    fconnect_->set_time_limit(time_limit_call_);

    timer_ = nh_.createTimer(ros::Duration(0.5), &MultiRobotRunner::tick, this);
  }

private:
  ros::NodeHandle nh_;
  ros::Timer timer_;
  env::OccMap::Ptr map_;
  std::shared_ptr<path_plan::BRRT> connect_;
  std::shared_ptr<path_plan::SOF_BRRT_Simple_Case3> fconnect_;
  BRRTInput input_;
  std::vector<Method> methods_;
  std::string scenarios_file_, output_result_, self_test_output_;
  double radius_, margin_, D_, time_limit_call_, time_limit_total_, zone_step_;
  int trials_, seed_;
  int path_trials_; // paths are saved for the first path_trials_ trials of a scenario; < 0 = all
  int separated_max_n_; // separated mode is the costly one: skip it above this many robots

  void tick(const ros::TimerEvent &)
  {
    if (!map_->mapValid())
      return;
    timer_.stop();
    // obstacles grow by D/2 and paths stay D apart, so each path owns a lane of width D
    map_->inflateXY(0.5 * D_);
    map_->saveBase();
    if (!self_test_output_.empty())
      selfTest();
    else
      runAll();
    ros::shutdown();
  }

  // Same seed for the same robot in the same scenario and trial, whatever the method, so that the
  // methods are compared on the same random numbers wherever their maps agree.
  long seedFor(int scenario, int trial, int robot, int attempt) const
  {
    if (seed_ < 0)
      return -1;
    return seed_ + 1000003L * scenario + 10007L * trial + 101L * robot + 7L * attempt;
  }

  template <typename PlannerPtr>
  PlanCall callPlanner(PlannerPtr &planner, const Eigen::Vector3d &s, const Eigen::Vector3d &g, long seed)
  {
    PlanCall c;
    planner->set_seed(seed);
    c.ok = planner->plan(s, g);
    c.nodes = planner->get_valid_tree_node_nums();
    c.iterations = planner->get_number_of_iteration();
    if (c.ok)
    {
      c.time = planner->getSolutions().back().second;
      c.path = planner->getPath();
    }
    else
    {
      c.time = planner->get_final_path_use_time_();
    }
    return c;
  }

  PlanCall plan(const Method &m, const Eigen::Vector3d &s, const Eigen::Vector3d &g, long seed)
  {
    return m.fconnect ? callPlanner(fconnect_, s, g, seed) : callPlanner(connect_, s, g, seed);
  }

  void blockOtherEndpoints(const std::vector<Eigen::Vector3d> &starts, const std::vector<Eigen::Vector3d> &goals, int robot)
  {
    for (int j = 0; j < (int)starts.size(); ++j)
    {
      if (j == robot)
        continue;
      map_->blockDisc(starts[j], D_);
      map_->blockDisc(goals[j], D_);
    }
  }

  json runOne(const json &sc, const Method &m, int trial)
  {
    const int scenario = sc.at("id");
    std::vector<Eigen::Vector3d> starts, goals;
    for (const auto &p : sc.at("starts"))
      starts.emplace_back(p[0].get<double>(), p[1].get<double>(), 0.0);
    for (const auto &p : sc.at("goals"))
      goals.emplace_back(p[0].get<double>(), p[1].get<double>(), 0.0);
    const int n = (int)starts.size();

    std::vector<RobotResult> robots(n);
    std::vector<int> order(n);
    std::iota(order.begin(), order.end(), 0);
    std::string fail_class;
    double search_time = 0.0, map_time = 0.0;
    int attempts = 1, reserve_dropped = 0;

    auto store = [&](int k, const PlanCall &c) {
      search_time += c.time;
      robots[k].call = c;
      robots[k].path = multi::toXY(c.path);
      robots[k].planned = true;
    };

    if (m.mode == "separated")
    {
      // shortest straight line first: a short path blocks less of the map
      std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
        return (starts[a] - goals[a]).norm() < (starts[b] - goals[b]).norm();
      });
      for (attempts = 1; attempts <= n + 1; ++attempts)
      {
        for (auto &r : robots)
          r = RobotResult();
        fail_class.clear();
        int failed = -1;
        std::vector<std::vector<Eigen::Vector3d>> placed;
        for (int k : order)
        {
          auto t0 = std::chrono::steady_clock::now();
          map_->restoreBase();
          blockOtherEndpoints(starts, goals, k);
          for (const auto &p : placed)
            map_->blockPath(p, D_);
          bool reachable = map_->connectedXY(starts[k], goals[k]);
          map_time += seconds(t0);
          if (!reachable)
          {
            failed = k;
            fail_class = "blocked";
            break;
          }
          PlanCall c = plan(m, starts[k], goals[k], seedFor(scenario, trial, k, attempts - 1));
          store(k, c);
          if (!c.ok)
          {
            failed = k;
            fail_class = "planner_limit";
            break;
          }
          placed.push_back(c.path);
        }
        if (failed < 0 || search_time > time_limit_total_)
          break;
        // the robot that could not get through goes first next time
        order.erase(std::find(order.begin(), order.end(), failed));
        order.insert(order.begin(), failed);
      }
      attempts = std::min(attempts, n + 1);
    }
    else
    {
      for (int k = 0; k < n; ++k)
      {
        if (search_time > time_limit_total_)
        {
          fail_class = "time_total";
          break;
        }
        auto t0 = std::chrono::steady_clock::now();
        map_->restoreBase();
        if (m.mode == "reserve")
        {
          blockOtherEndpoints(starts, goals, k);
          if (!map_->connectedXY(starts[k], goals[k]))
          {
            ++reserve_dropped;
            map_->restoreBase();
          }
        }
        bool reachable = map_->connectedXY(starts[k], goals[k]);
        map_time += seconds(t0);
        if (!reachable)
        {
          fail_class = "no_path";
          continue;
        }
        PlanCall c = plan(m, starts[k], goals[k], seedFor(scenario, trial, k, 0));
        store(k, c);
        if (!c.ok)
          fail_class = "planner_limit";
      }
    }
    map_->restoreBase();

    bool success = fail_class.empty();
    json out;
    out["scenario"] = scenario;
    out["kind"] = sc.at("kind");
    out["N"] = n;
    out["trial"] = trial;
    out["method"] = m.name;
    out["seed"] = seedFor(scenario, trial, 0, 0);
    out["success"] = success;
    out["fail_class"] = fail_class;
    out["attempts"] = attempts;
    out["reserve_dropped"] = reserve_dropped;
    out["order"] = order;
    out["search_time"] = search_time;
    out["map_time"] = map_time;

    json robot_list = json::array();
    std::vector<Path2> paths;
    double total_length = 0.0, max_length = 0.0, total_turn = 0.0, max_turn = 0.0;
    long iterations = 0, nodes = 0;
    for (int k = 0; k < n; ++k)
    {
      const RobotResult &r = robots[k];
      json jr;
      jr["success"] = r.planned && r.call.ok;
      jr["time"] = r.call.time;
      jr["nodes"] = r.call.nodes;
      jr["iterations"] = r.call.iterations;
      iterations += r.call.iterations;
      nodes += r.call.nodes;
      if (r.planned && r.call.ok)
      {
        double len = multi::pathLength(r.path);
        multi::Turning turn = multi::pathTurning(r.path);
        jr["length"] = len;
        jr["turn_total"] = turn.total;
        jr["turn_max"] = turn.max;
        total_length += len;
        max_length = std::max(max_length, len);
        total_turn += turn.total;
        max_turn = std::max(max_turn, turn.max);
        if (path_trials_ < 0 || trial < path_trials_)
        {
          json pts = json::array();
          for (const auto &p : r.path)
            pts.push_back({round4(p.x()), round4(p.y())});
          jr["path"] = pts;
        }
        paths.push_back(r.path);
      }
      robot_list.push_back(jr);
    }
    out["robots"] = robot_list;
    out["iterations"] = iterations;
    out["nodes"] = nodes;

    if (success)
    {
      auto t0 = std::chrono::steady_clock::now();
      std::vector<multi::ConflictZone> zones = multi::findConflictZones(paths, D_, zone_step_);
      multi::ZoneStats st = multi::summarizeZones(zones, paths);
      out["zone_time"] = seconds(t0);
      out["total_length"] = total_length;
      out["max_length"] = max_length;
      out["turn_total"] = total_turn;
      out["turn_max"] = max_turn;
      out["n_zones"] = st.n_zones;
      out["conflict_pairs"] = st.conflict_pairs;
      out["endpoint_zones"] = st.endpoint_zones;
      out["head_on_zones"] = st.head_on_zones;
      out["same_dir_zones"] = st.same_dir_zones;
      out["zone_length"] = st.zone_length;
      out["overlap_fraction"] = st.overlap_fraction;
      json zone_list = json::array();
      for (const auto &z : zones)
        zone_list.push_back({{"a", z.a}, {"b", z.b}, {"a_in", z.a_in}, {"a_out", z.a_out}, {"b_in", z.b_in}, {"b_out", z.b_out}, {"min_dist", z.min_dist}, {"direction", z.direction}, {"start_a", z.start_a}, {"goal_a", z.goal_a}, {"start_b", z.start_b}, {"goal_b", z.goal_b}});
      out["zones"] = zone_list;
    }
    return out;
  }

  void runAll()
  {
    std::ifstream file(scenarios_file_);
    if (!file.is_open())
    {
      ROS_ERROR("Failed to open scenario file: %s", scenarios_file_.c_str());
      return;
    }
    json doc;
    file >> doc;

    json runs = json::array();
    const auto &scenarios = doc.at("scenarios");
    for (size_t i = 0; i < scenarios.size(); ++i)
    {
      // every method runs on the same scenario and trial in turn, so that a change of CPU speed
      // over a long run hits them all alike
      for (int t = 0; t < trials_; ++t)
        for (const auto &m : methods_)
        {
          if (m.mode == "separated" && (int)scenarios[i].at("starts").size() > separated_max_n_)
            continue;
          runs.push_back(runOne(scenarios[i], m, t));
        }
      ROS_INFO("[multi] scenario %zu/%zu done", i + 1, scenarios.size());
    }

    json result;
    result["map"] = doc.at("map");
    result["robot_radius"] = radius_;
    result["margin"] = margin_;
    result["D"] = D_;
    result["seed"] = seed_;
    result["time_limit_call"] = time_limit_call_;
    result["time_limit_total"] = time_limit_total_;
    result["environment"] = input_.environment;
    result["runs"] = runs;

    // written under a temporary name: a result file that exists is always complete
    const std::string tmp = output_result_ + ".tmp";
    {
      std::ofstream out(tmp);
      out << result.dump();
    }
    std::rename(tmp.c_str(), output_result_.c_str());
    ROS_INFO("[multi] saved %zu runs to %s", runs.size(), output_result_.c_str());
  }

  std::string dumpGrid() const
  {
    const Eigen::Vector3d origin = map_->getOrigin(), size = map_->getMapSize();
    const double res = map_->getResolution();
    const int nx = (int)std::ceil(size.x() / res), ny = (int)std::ceil(size.y() / res);
    std::string out;
    out.reserve(nx * ny);
    for (int x = 0; x < nx; ++x)
      for (int y = 0; y < ny; ++y)
        out.push_back(map_->isStateValid(Eigen::Vector3d(origin.x() + (x + 0.5) * res, origin.y() + (y + 0.5) * res, 0.0)) ? '0' : '1');
    return out;
  }

  // Writes what the map helpers produce, for check_multi_map.py to compare with its own geometry.
  void selfTest()
  {
    json out;
    out["D"] = D_;
    out["inflated"] = dumpGrid();

    const Eigen::Vector3d disc(10.3, -20.7, 0.0);
    map_->blockDisc(disc, D_);
    out["disc_center"] = {disc.x(), disc.y()};
    out["disc"] = dumpGrid();
    map_->restoreBase();
    out["restored"] = dumpGrid();

    const std::vector<Eigen::Vector3d> path{{-30.2, -40.1, 0.0}, {-10.7, -12.4, 0.2}, {25.3, -18.9, -0.3}, {40.1, 33.3, 0.0}};
    map_->blockPath(path, D_);
    json pts = json::array();
    for (const auto &p : path)
      pts.push_back({p.x(), p.y()});
    out["path"] = pts;
    out["blocked_path"] = dumpGrid();

    // connectivity, both on the plain map and with the path above acting as a wall
    std::mt19937 gen(1);
    std::uniform_real_distribution<double> coord(-99.0, 99.0);
    json blocked_pairs = json::array();
    for (int i = 0; i < 300; ++i)
    {
      Eigen::Vector3d a(coord(gen), coord(gen), 0.0), b(coord(gen), coord(gen), 0.0);
      blocked_pairs.push_back({{"a", {a.x(), a.y()}}, {"b", {b.x(), b.y()}}, {"connected", map_->connectedXY(a, b)}});
    }
    out["blocked_pairs"] = blocked_pairs;
    map_->restoreBase();
    json plain_pairs = json::array();
    for (int i = 0; i < 300; ++i)
    {
      Eigen::Vector3d a(coord(gen), coord(gen), 0.0), b(coord(gen), coord(gen), 0.0);
      plain_pairs.push_back({{"a", {a.x(), a.y()}}, {"b", {b.x(), b.y()}}, {"connected", map_->connectedXY(a, b)}});
    }
    out["plain_pairs"] = plain_pairs;

    std::ofstream file(self_test_output_);
    file << out.dump();
    ROS_INFO("[multi] self test written to %s", self_test_output_.c_str());
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "multi_robot_node");
  ros::NodeHandle nh("~");

  MultiRobotRunner runner(nh);

  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
