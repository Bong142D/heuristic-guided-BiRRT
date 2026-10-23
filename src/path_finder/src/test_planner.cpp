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
#include "self_msgs_and_srvs/GlbObsRcv.h"
#include "occ_grid/occ_map.h"
#include "path_finder/rrt_sharp.h"
#include "path_finder/rrt_star.h"
#include "path_finder/rrt.h"
#include "path_finder/brrt.h"
#include "path_finder/goal_bias_brrt.h"
#include "path_finder/brrt_star.h"
#include "path_finder/brrt_sample_gravity.h"
#include "path_finder/brrt_simple_case1.h"
#include "path_finder/brrt_simple_case2.h"
#include "path_finder/SOF_brrt_simple_case1.h"
#include "path_finder/SOF_brrt_simple_case2.h"
#include "path_finder/SOF_brrt_simple_case3.h"
#include "path_finder/testcase.h"
#include "visualization/visualization.hpp"
#include "path_finder/sampler.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <std_msgs/Empty.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

#define NUMBER_TEST_TIMES 100
class TesterPathFinder
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber goal_sub_;
    ros::Timer execution_timer_;
    ros::ServiceClient rcv_glb_obs_client_;

    env::OccMap::Ptr env_ptr_;
    std::shared_ptr<visualization::Visualization> vis_ptr_;
    shared_ptr<path_plan::RRTSharp> rrt_sharp_ptr_;
    shared_ptr<path_plan::RRTStar> rrt_star_ptr_;
    shared_ptr<path_plan::RRT> rrt_ptr_;
    shared_ptr<path_plan::BRRT> brrt_ptr_;
    shared_ptr<path_plan::GoalBiasBRRT> goal_bias_brrt_ptr_;
    shared_ptr<path_plan::BRRTStar> brrt_star_ptr_;
    shared_ptr<path_plan::BRRT_Optimize> brrt_optimize_ptr_;
    shared_ptr<path_plan::BRRT_Simple_Case1> brrt_simple_case1_ptr_;
    shared_ptr<path_plan::BRRT_Simple_Case2> brrt_simple_case2_ptr_;
    shared_ptr<path_plan::SOF_BRRT_Simple_Case1> sof_brrt_simple_case1_ptr_;
    shared_ptr<path_plan::SOF_BRRT_Simple_Case2> sof_brrt_simple_case2_ptr_;
    shared_ptr<path_plan::SOF_BRRT_Simple_Case3> sof_brrt_simple_case3_ptr_;
    Eigen::Vector3d start_, goal_;

    bool run_rrt_, run_rrt_star_, run_rrt_sharp_;
    bool run_brrt_, run_brrt_star_, run_brrt_optimize_, run_goal_bias_brrt_;
    bool experiment;
    // Implement for testing path planning algorithms
    BiasSampler sampler_;
    BRRTExperimentMultiAlgo *manager;
    std::string input_param_;
    std::string output_result_;
    
    bool generate_mode_;
    double gen_min_dist_;
    std::string dataset_path_;
    bool use_fixed_start_goal_;
    double fixed_start_x_, fixed_start_y_, fixed_start_z_;
    double fixed_goal_x_, fixed_goal_y_, fixed_goal_z_;
    ros::Subscriber step_sub_;
    std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
    bool step_mode_enabled_ = false;
    std::string scenarios_file_;
    int trials_per_query_;

    void stepCallback(const std_msgs::Empty::ConstPtr &msg)
    {
        if (brrt_ptr_)
            brrt_ptr_->triggerNextStep();
        if (brrt_simple_case1_ptr_)
            brrt_simple_case1_ptr_->triggerNextStep();
        if (goal_bias_brrt_ptr_)
            goal_bias_brrt_ptr_->triggerNextStep();
        if (sof_brrt_simple_case1_ptr_)
            sof_brrt_simple_case1_ptr_->triggerNextStep();
        if (sof_brrt_simple_case2_ptr_)
            sof_brrt_simple_case2_ptr_->triggerNextStep();
        if (sof_brrt_simple_case3_ptr_)
            sof_brrt_simple_case3_ptr_->triggerNextStep();
    }

    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
    {
        if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN)
        {
            if (feedback->marker_name == "next_step")
            {
                ROS_INFO("Next Step Button Clicked");
                if (brrt_ptr_)
                    brrt_ptr_->triggerNextStep();
                if (brrt_simple_case1_ptr_)
                    brrt_simple_case1_ptr_->triggerNextStep();
                if (goal_bias_brrt_ptr_)
                    goal_bias_brrt_ptr_->triggerNextStep();
                if (sof_brrt_simple_case1_ptr_)
                    sof_brrt_simple_case1_ptr_->triggerNextStep();
                if (sof_brrt_simple_case2_ptr_)
                    sof_brrt_simple_case2_ptr_->triggerNextStep();
                if (sof_brrt_simple_case3_ptr_)
                    sof_brrt_simple_case3_ptr_->triggerNextStep();
            }
        }
    }

    void makeButtonMarker(const std::string& name, const Eigen::Vector3d& pos)
    {
        visualization_msgs::InteractiveMarker int_marker;
        int_marker.header.frame_id = "map";
        int_marker.pose.position.x = pos.x();
        int_marker.pose.position.y = pos.y();
        int_marker.pose.position.z = pos.z();
        int_marker.scale = 2.0;
        int_marker.name = name;
        int_marker.description = "Click to Advance Iteration";

        visualization_msgs::Marker button_marker;
        button_marker.type = visualization_msgs::Marker::CUBE;
        button_marker.scale.x = 2.0;
        button_marker.scale.y = 2.0;
        button_marker.scale.z = 0.5;
        button_marker.color.r = 0.0;
        button_marker.color.g = 1.0;
        button_marker.color.b = 0.0;
        button_marker.color.a = 0.8;

        visualization_msgs::InteractiveMarkerControl control;
        control.always_visible = true;
        control.markers.push_back(button_marker);

        visualization_msgs::Marker text_marker;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.scale.z = 1.0;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        text_marker.text = "NEXT STEP";
        text_marker.pose.position.z = 1.0;
        control.markers.push_back(text_marker);

        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
        control.name = "button_control";

        int_marker.controls.push_back(control);

        server_->insert(int_marker);
        server_->setCallback(int_marker.name, boost::bind(&TesterPathFinder::processFeedback, this, _1));
        server_->applyChanges();
    }

public:
    TesterPathFinder(const ros::NodeHandle &nh) : nh_(nh)
    {
        env_ptr_ = std::make_shared<env::OccMap>();
        env_ptr_->init(nh_);
      
        vis_ptr_ = std::make_shared<visualization::Visualization>(nh_);
        vis_ptr_->registe<visualization_msgs::Marker>("start");
        vis_ptr_->registe<visualization_msgs::Marker>("goal");

        rrt_sharp_ptr_ = std::make_shared<path_plan::RRTSharp>(nh_, env_ptr_);
        rrt_sharp_ptr_->setVisualizer(vis_ptr_);
        vis_ptr_->registe<nav_msgs::Path>("rrt_sharp_final_path");
        vis_ptr_->registe<sensor_msgs::PointCloud2>("rrt_sharp_final_wpts");

        rrt_star_ptr_ = std::make_shared<path_plan::RRTStar>(nh_, env_ptr_);
        rrt_star_ptr_->setVisualizer(vis_ptr_);
        vis_ptr_->registe<nav_msgs::Path>("rrt_star_final_path");
        vis_ptr_->registe<sensor_msgs::PointCloud2>("rrt_star_final_wpts");
        vis_ptr_->registe<visualization_msgs::MarkerArray>("rrt_star_paths");

        rrt_ptr_ = std::make_shared<path_plan::RRT>(nh_, env_ptr_);
        rrt_ptr_->setVisualizer(vis_ptr_);
        vis_ptr_->registe<nav_msgs::Path>("rrt_final_path");
        vis_ptr_->registe<sensor_msgs::PointCloud2>("rrt_final_wpts");

        brrt_ptr_ = std::make_shared<path_plan::BRRT>(nh_, env_ptr_);
        brrt_ptr_->setVisualizer(vis_ptr_);
        vis_ptr_->registe<nav_msgs::Path>("brrt_final_path");
        vis_ptr_->registe<sensor_msgs::PointCloud2>("brrt_final_wpts");

        goal_bias_brrt_ptr_ = std::make_shared<path_plan::GoalBiasBRRT>(nh_, env_ptr_);
        goal_bias_brrt_ptr_->setVisualizer(vis_ptr_);
        vis_ptr_->registe<nav_msgs::Path>("goal_bias_brrt_final_path");
        vis_ptr_->registe<sensor_msgs::PointCloud2>("goal_bias_brrt_final_wpts");

        brrt_star_ptr_ = std::make_shared<path_plan::BRRTStar>(nh_, env_ptr_);
        brrt_star_ptr_->setVisualizer(vis_ptr_);
        vis_ptr_->registe<nav_msgs::Path>("brrt_star_final_path");
        vis_ptr_->registe<sensor_msgs::PointCloud2>("brrt_star_final_wpts");

        brrt_optimize_ptr_ = std::make_shared<path_plan::BRRT_Optimize>(nh_, env_ptr_);
        brrt_optimize_ptr_->setVisualizer(vis_ptr_);
        vis_ptr_->registe<nav_msgs::Path>("brrt_optimize_final_path");
        vis_ptr_->registe<sensor_msgs::PointCloud2>("brrt_optimize_final_wpts");

        brrt_simple_case1_ptr_ = std::make_shared<path_plan::BRRT_Simple_Case1>(nh_, env_ptr_);
        brrt_simple_case1_ptr_->setVisualizer(vis_ptr_);
        vis_ptr_->registe<nav_msgs::Path>("brrt_case1_final_path");
        vis_ptr_->registe<sensor_msgs::PointCloud2>("brrt_case1_final_wpts");

        brrt_simple_case2_ptr_ = std::make_shared<path_plan::BRRT_Simple_Case2>(nh_, env_ptr_);
        brrt_simple_case2_ptr_->setVisualizer(vis_ptr_);
        vis_ptr_->registe<nav_msgs::Path>("brrt_case2_final_path");
        vis_ptr_->registe<sensor_msgs::PointCloud2>("brrt_case2_final_wpts");

        sof_brrt_simple_case1_ptr_ = std::make_shared<path_plan::SOF_BRRT_Simple_Case1>(nh_, env_ptr_);
        sof_brrt_simple_case1_ptr_->setVisualizer(vis_ptr_);
        vis_ptr_->registe<nav_msgs::Path>("sof_brrt_case1_final_path");
        vis_ptr_->registe<sensor_msgs::PointCloud2>("sof_brrt_case1_final_wpts");
        vis_ptr_->registe<visualization_msgs::Marker>("sof_brrt/sampling_rays");
        vis_ptr_->registe<visualization_msgs::Marker>("sof_brrt/selected_sector");
        vis_ptr_->registe<visualization_msgs::Marker>("sof_brrt/sector_boundaries");
        vis_ptr_->registe<visualization_msgs::Marker>("sof_brrt/x_rand");
        vis_ptr_->registe<visualization_msgs::Marker>("sof_brrt/new_node");
        vis_ptr_->registe<visualization_msgs::Marker>("sof_brrt/parent_node");
        vis_ptr_->registe<visualization_msgs::MarkerArray>("sof_brrt/tree_vertice");
        vis_ptr_->registe<visualization_msgs::Marker>("sof_brrt/tree_edges");

        sof_brrt_simple_case2_ptr_ = std::make_shared<path_plan::SOF_BRRT_Simple_Case2>(nh_, env_ptr_);
        sof_brrt_simple_case2_ptr_->setVisualizer(vis_ptr_);
        vis_ptr_->registe<nav_msgs::Path>("sof_brrt_case2_final_path");
        vis_ptr_->registe<sensor_msgs::PointCloud2>("sof_brrt_case2_final_wpts");
        vis_ptr_->registe<visualization_msgs::Marker>("sof_brrt_case2/sampling_rays");
        vis_ptr_->registe<visualization_msgs::Marker>("sof_brrt_case2/selected_sector");
        vis_ptr_->registe<visualization_msgs::Marker>("sof_brrt_case2/sector_boundaries");
        vis_ptr_->registe<visualization_msgs::Marker>("sof_brrt_case2/x_rand");
        vis_ptr_->registe<visualization_msgs::Marker>("sof_brrt_case2/new_node");
        vis_ptr_->registe<visualization_msgs::Marker>("sof_brrt_case2/parent_node");
        vis_ptr_->registe<visualization_msgs::MarkerArray>("sof_brrt_case2/tree_vertice");
        vis_ptr_->registe<visualization_msgs::Marker>("sof_brrt_case2/tree_edges");
        
        sof_brrt_simple_case3_ptr_ = std::make_shared<path_plan::SOF_BRRT_Simple_Case3>(nh_, env_ptr_);
        sof_brrt_simple_case3_ptr_->setVisualizer(vis_ptr_);
        vis_ptr_->registe<nav_msgs::Path>("sof_brrt_case3_final_path");
        vis_ptr_->registe<sensor_msgs::PointCloud2>("sof_brrt_case3_final_wpts");
        vis_ptr_->registe<visualization_msgs::Marker>("sof_brrt_case3/sampling_rays");
        vis_ptr_->registe<visualization_msgs::Marker>("sof_brrt_case3/selected_sector");
        vis_ptr_->registe<visualization_msgs::Marker>("sof_brrt_case3/sector_boundaries");
        vis_ptr_->registe<visualization_msgs::Marker>("sof_brrt_case3/x_rand");
        vis_ptr_->registe<visualization_msgs::Marker>("sof_brrt_case3/new_node");
        vis_ptr_->registe<visualization_msgs::Marker>("sof_brrt_case3/parent_node");
        vis_ptr_->registe<visualization_msgs::MarkerArray>("sof_brrt_case3/tree_vertice");
        vis_ptr_->registe<visualization_msgs::Marker>("sof_brrt_case3/tree_edges");
        vis_ptr_->registe<visualization_msgs::MarkerArray>("sof_brrt_case3/rand_nodes");

        goal_ = get_sample_valid();
        goal_sub_ = nh_.subscribe("/goal", 1, &TesterPathFinder::goalCallback, this);
        execution_timer_ = nh_.createTimer(ros::Duration(1), &TesterPathFinder::executionCallback, this);
        rcv_glb_obs_client_ = nh_.serviceClient<self_msgs_and_srvs::GlbObsRcv>("/pub_glb_obs");

        start_.setZero();

        nh_.param("run_rrt", run_rrt_, true);
        nh_.param("run_rrt_star", run_rrt_star_, true);
        nh_.param("run_rrt_sharp", run_rrt_sharp_, true);
        nh_.param("run_brrt", run_brrt_, false);
        nh_.param("run_goal_bias_brrt", run_goal_bias_brrt_, false);
        nh_.param("run_brrt_star", run_brrt_star_, false);
        nh_.param("run_brrt_optimize", run_brrt_optimize_, false);

        nh_.param("generate_mode", generate_mode_, false);
        nh_.param("gen_min_dist", gen_min_dist_, 0.0);
        nh_.param("dataset_path", dataset_path_, std::string(""));
        nh_.param("use_fixed_start_goal", use_fixed_start_goal_, false);
        nh_.param("fixed_start_x", fixed_start_x_, 0.0);
        nh_.param("fixed_start_y", fixed_start_y_, 0.0);
        nh_.param("fixed_start_z", fixed_start_z_, 0.0);
        nh_.param("fixed_goal_x", fixed_goal_x_, 0.0);
        nh_.param("fixed_goal_y", fixed_goal_y_, 0.0);
        nh_.param("fixed_goal_z", fixed_goal_z_, 0.0);

        nh_.param("input_param", input_param_, std::string("/home/huy/rrt/brrt_input.json"));
        nh_.param("output_result", output_result_, std::string("/home/huy/evaluation/20250705/"));
        nh_.param("scenarios_file", scenarios_file_, std::string(""));
        nh_.param("trials_per_query", trials_per_query_, 1);
        nh_.param("experiment", experiment, false);
        nh_.param("sof_brrt/step_mode", step_mode_enabled_, false);
        
        if (step_mode_enabled_) {
            int step_interval;
            nh_.param("sof_brrt/step_interval", step_interval, 50);
            brrt_ptr_->setStepMode(true);
            brrt_ptr_->setStepInterval(step_interval);
            brrt_simple_case1_ptr_->setStepMode(true);
            brrt_simple_case1_ptr_->setStepInterval(step_interval);
            goal_bias_brrt_ptr_->setStepMode(true);
            goal_bias_brrt_ptr_->setStepInterval(step_interval);
            sof_brrt_simple_case1_ptr_->setStepMode(true);
            sof_brrt_simple_case1_ptr_->setStepInterval(step_interval);
            sof_brrt_simple_case2_ptr_->setStepMode(true);
            sof_brrt_simple_case2_ptr_->setStepInterval(step_interval);
            sof_brrt_simple_case3_ptr_->setStepMode(true);
            sof_brrt_simple_case3_ptr_->setStepInterval(step_interval);
            server_.reset(new interactive_markers::InteractiveMarkerServer("sof_brrt_controls", "", false));
            makeButtonMarker("next_step", Eigen::Vector3d(0, 0, 2.0));
            step_sub_ = nh_.subscribe("/sof_brrt/step", 1, &TesterPathFinder::stepCallback, this);
            ROS_WARN("SOF_BRRT Step Mode Enabled with interval: %d. Use the green box in RViz or publish to /sof_brrt/step to advance.", step_interval);
        }
        std::cout <<"input_param: " << input_param_ << std::endl;
        std::cout <<"output_result: " << output_result_ << std::endl;
        manager = new BRRTExperimentMultiAlgo(
        input_param_,
        output_result_);
    }
    ~TesterPathFinder()
    {
        delete manager;
    };

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal_msg)
    {
        goal_[0] = goal_msg->pose.position.x;
        goal_[1] = goal_msg->pose.position.y;
        goal_[2] = goal_msg->pose.position.z;
        if (!env_ptr_->isStateValid(goal_)) {
            Eigen::Vector3d old_goal = goal_;
            goal_ = env_ptr_->findNearestFree(goal_);
            ROS_WARN("Goal position [%.2f, %.2f, %.2f] is in obstacle, shifted to [%.2f, %.2f, %.2f]", 
                     old_goal.x(), old_goal.y(), old_goal.z(), goal_.x(), goal_.y(), goal_.z());
        }
        if (!env_ptr_->isStateValid(start_)) {
            Eigen::Vector3d old_start = start_;
            start_ = env_ptr_->findNearestFree(start_);
             ROS_WARN("Start position [%.2f, %.2f, %.2f] is in obstacle, shifted to [%.2f, %.2f, %.2f]", 
                     old_start.x(), old_start.y(), old_start.z(), start_.x(), start_.y(), start_.z());
        }
        ROS_INFO_STREAM("\n-----------------------------\ngoal rcved at " << goal_.transpose());
        vis_ptr_->visualize_a_ball(start_, 1.5, "start", visualization::Color::green);
        vis_ptr_->visualize_a_ball(goal_, 1.5, "goal", visualization::Color::red);

        // BiasSampler sampler;
        // sampler.setSamplingRange(env_ptr_->getOrigin(), env_ptr_->getMapSize());
        // vector<Eigen::Vector3d> preserved_samples;
        // for (int i = 0; i < 5000; ++i)
        // {
        //     Eigen::Vector3d rand_sample;
        //     sampler.uniformSamplingOnce(rand_sample);
        //     preserved_samples.push_back(rand_sample);
        // }
        // rrt_ptr_->setPreserveSamples(preserved_samples);
        // rrt_star_ptr_->setPreserveSamples(preserved_samples);
        // rrt_sharp_ptr_->setPreserveSamples(preserved_samples);

        const auto &input = manager->get_input();

        if (run_rrt_)
        {
            bool rrt_res = rrt_ptr_->plan(start_, goal_);
            if (rrt_res)
            {
                vector<Eigen::Vector3d> final_path = rrt_ptr_->getPath();
                vis_ptr_->visualize_path(final_path, "rrt_final_path");
                vis_ptr_->visualize_pointcloud(final_path, "rrt_final_wpts");
                vector<std::pair<double, double>> slns = rrt_ptr_->getSolutions();
                ROS_INFO_STREAM("[RRT] final path len: " << slns.back().first);
            }
        }

        if (run_rrt_star_)
        {
            bool rrt_star_res = rrt_star_ptr_->plan(start_, goal_);
            if (rrt_star_res)
            {
                vector<vector<Eigen::Vector3d>> routes = rrt_star_ptr_->getAllPaths();
                vis_ptr_->visualize_path_list(routes, "rrt_star_paths", visualization::blue);
                vector<Eigen::Vector3d> final_path = rrt_star_ptr_->getPath();
                vis_ptr_->visualize_path(final_path, "rrt_star_final_path");
                vis_ptr_->visualize_pointcloud(final_path, "rrt_star_final_wpts");
                vector<std::pair<double, double>> slns = rrt_star_ptr_->getSolutions();
                ROS_INFO_STREAM("[RRT*] final path len: " << slns.back().first);
            }
        }

        if (run_rrt_sharp_)
        {
            bool rrt_sharp_res = rrt_sharp_ptr_->plan(start_, goal_);
            if (rrt_sharp_res)
            {
                vector<Eigen::Vector3d> final_path = rrt_sharp_ptr_->getPath();
                vis_ptr_->visualize_path(final_path, "rrt_sharp_final_path");
                vis_ptr_->visualize_pointcloud(final_path, "rrt_sharp_final_wpts");
                vector<std::pair<double, double>> slns = rrt_sharp_ptr_->getSolutions();
                ROS_INFO_STREAM("[RRT#] final path len: " << slns.back().first);
            }
        }

        if (run_brrt_)
        {
            brrt_ptr_->set_test_param(input.epsilon);
            bool brrt_res = brrt_ptr_->plan(start_, goal_);
            if (brrt_res)
            {
                vector<Eigen::Vector3d> final_path = brrt_ptr_->getPath();
                vis_ptr_->visualize_path(final_path, "brrt_final_path");
                vis_ptr_->visualize_pointcloud(final_path, "brrt_final_wpts");
                vector<std::pair<double, double>> slns = brrt_ptr_->getSolutions();
                ROS_INFO_STREAM("[BRRT] final path len: " << slns.back().first);
            }
        }

        if (run_goal_bias_brrt_)
        {
            goal_bias_brrt_ptr_->set_test_param(input.epsilon);
            bool goal_bias_brrt_res = goal_bias_brrt_ptr_->plan(start_, goal_);
            if (goal_bias_brrt_res)
            {
                vector<Eigen::Vector3d> final_path = goal_bias_brrt_ptr_->getPath();
                vis_ptr_->visualize_path(final_path, "goal_bias_brrt_final_path");
                vis_ptr_->visualize_pointcloud(final_path, "goal_bias_brrt_final_wpts");
                vector<std::pair<double, double>> slns = goal_bias_brrt_ptr_->getSolutions();
                ROS_INFO_STREAM("[GoalBiasBRRT] final path len: " << slns.back().first);
            }
        }

        if (run_brrt_star_)
        {
            bool brrt_star_res = brrt_star_ptr_->plan(start_, goal_);
            if (brrt_star_res)
            {
                vector<Eigen::Vector3d> final_path = brrt_star_ptr_->getPath();
                vis_ptr_->visualize_path(final_path, "brrt_star_final_path");
                vis_ptr_->visualize_pointcloud(final_path, "brrt_star_final_wpts");
                vector<std::pair<double, double>> slns = brrt_star_ptr_->getSolutions();
                ROS_INFO_STREAM("[BRRT*] final path len: " << slns.back().first);
            }
        }
        if (run_brrt_optimize_)
        {
            // bool brrt_optimize_res = brrt_optimize_ptr_->plan(start_, goal_);
            // if (brrt_optimize_res)
            // {
            //     vector<Eigen::Vector3d> final_path = brrt_optimize_ptr_->getPath();
            //     vis_ptr_->visualize_path(final_path, "brrt_optimize_final_path");
            //     vis_ptr_->visualize_pointcloud(final_path, "brrt_optimize_final_wpts");
            //     vector<std::pair<double, double>> slns = brrt_optimize_ptr_->getSolutions();
            //     ROS_INFO_STREAM("[BRRTOpitmize*] final path len: " << slns.back().first);
            // }
            brrt_simple_case1_ptr_->set_heuristic_param(input.p1, input.u_p, input.alpha, input.beta, input.gamma, input.epsilon);
            bool brrt_optimize_case1_res = brrt_simple_case1_ptr_->plan(start_, goal_);
            if (brrt_optimize_case1_res)
            {
                vector<Eigen::Vector3d> final_path = brrt_simple_case1_ptr_->getPath();
                vis_ptr_->visualize_path(final_path, "brrt_case1_final_path");
                vis_ptr_->visualize_pointcloud(final_path, "brrt_case1_final_wpts");
                vector<std::pair<double, double>> slns = brrt_simple_case1_ptr_->getSolutions();
                ROS_INFO_STREAM("[BRRTOpitmize*] final path len: " << slns.back().first);
            }
            // bool brrt_optimize_case2_res = brrt_simple_case2_ptr_->plan(start_, goal_);
            // if (brrt_optimize_case2_res)
            // {
            //     vector<Eigen::Vector3d> final_path = brrt_simple_case2_ptr_->getPath();
            //     vis_ptr_->visualize_path(final_path, "brrt_case2_final_path");
            //     vis_ptr_->visualize_pointcloud(final_path, "brrt_case2_final_wpts");
            //     vector<std::pair<double, double>> slns = brrt_simple_case2_ptr_->getSolutions();
            //     ROS_INFO_STREAM("[BRRTOpitmize*] final path len: " << slns.back().first);
            // }
            // sof_brrt_simple_case1_ptr_->set_heuristic_param(input.p1, input.u_p, input.alpha, input.beta, input.gamma, input.epsilon, input.p2, input.n_rays, input.weight_grade, input.ray_step_size);
            // bool sof_brrt_optimize_case1_res = sof_brrt_simple_case1_ptr_->plan(start_, goal_);
            // sof_brrt_simple_case1_ptr_->visualizeWholeTree();
            // if (sof_brrt_optimize_case1_res)
            // {
            //     vector<Eigen::Vector3d> final_path = sof_brrt_simple_case1_ptr_->getPath();
            //     vis_ptr_->visualize_path(final_path, "sof_brrt_case1_final_path");
            //     vis_ptr_->visualize_pointcloud(final_path, "sof_brrt_case1_final_wpts");
            //     vector<std::pair<double, double>> slns = sof_brrt_simple_case1_ptr_->getSolutions();
            //     ROS_INFO_STREAM("[SOFBRRTOpitmize*] final path len: " << slns.back().first);
            // }
            // bool sof_brrt_optimize_case2_res = sof_brrt_simple_case2_ptr_->plan(start_, goal_);
            // sof_brrt_simple_case2_ptr_->visualizeWholeTree();
            // if (sof_brrt_optimize_case2_res)
            // {
            //     vector<Eigen::Vector3d> final_path = sof_brrt_simple_case2_ptr_->getPath();
            //     vis_ptr_->visualize_path(final_path, "sof_brrt_case2_final_path");
            //     vis_ptr_->visualize_pointcloud(final_path, "sof_brrt_case2_final_wpts");
            //     vector<std::pair<double, double>> slns = sof_brrt_simple_case2_ptr_->getSolutions();
            //     ROS_INFO_STREAM("[SOFBRRTOpitmize*] Case2 final path len: " << slns.back().first);
            // }
            sof_brrt_simple_case3_ptr_->set_heuristic_param(input.p1, input.u_p, input.alpha, input.beta, input.gamma, input.epsilon, input.p2, input.n_rays, input.weight_grade, input.ray_step_size);
            bool sof_brrt_optimize_case3_res = sof_brrt_simple_case3_ptr_->plan(start_, goal_);
            sof_brrt_simple_case3_ptr_->visualizeWholeTree();
            if (sof_brrt_optimize_case3_res)
            {
                vector<Eigen::Vector3d> final_path = sof_brrt_simple_case3_ptr_->getPath();
                vis_ptr_->visualize_path(final_path, "sof_brrt_case3_final_path");
                vis_ptr_->visualize_pointcloud(final_path, "sof_brrt_case3_final_wpts");
                vector<std::pair<double, double>> slns = sof_brrt_simple_case3_ptr_->getSolutions();
                ROS_INFO_STREAM("[SOFBRRTOpitmize*] Case3 final path len: " << slns.back().first);
            }
        }

        start_ = goal_;
    }
    void print_vector3d(std::string name, Eigen::Vector3d &p)
    {
        std::cout << name << " x: " << p[0] << " y: " << p[1] << " z: " << p[2] << std::endl;
    }
    Eigen::Vector3d get_sample_valid()
    {
        Eigen::Vector3d x_rand;
        sampler_.setSamplingRange(env_ptr_->getOrigin(), env_ptr_->getMapSize());
        sampler_.samplingOnce(x_rand);
        // samplingOnce(x_rand);
        // map_ptr_->isStateValid(g)
        while (!env_ptr_->isStateValid(x_rand))
        {
            sampler_.samplingOnce(x_rand);
        }
        return x_rand;
    }
    void executionCallback(const ros::TimerEvent &event)
    {

        if (!env_ptr_->mapValid())
        {
            ROS_INFO("no map rcved yet.");
            self_msgs_and_srvs::GlbObsRcv srv;
            if (!rcv_glb_obs_client_.call(srv))
                ROS_WARN("Failed to call service /pub_glb_obs");
        }
        else
        {
            execution_timer_.stop();
            ROS_WARN("Timer tick");
            if (experiment){
                experiment_test();
            }
        }
    };

    static double pathLength(const vector<Eigen::Vector3d> &path)
    {
        double len = 0.0;
        for (size_t i = 1; i < path.size(); ++i)
            len += (path[i] - path[i - 1]).norm();
        return len;
    }

    // Run one planner on (start_, goal_) and record its metrics under `name`.
    template <typename PlannerPtr>
    void runAndRecord(const std::string &name, PlannerPtr &planner, std::map<std::string, AlgoResult> &out)
    {
        bool ok = planner->plan(start_, goal_);
        int num_nodes = planner->get_valid_tree_node_nums();
        int num_iterations = planner->get_number_of_iteration();
        if (ok)
        {
            vector<std::pair<double, double>> slns = planner->getSolutions();
            out[name] = {true, slns.back().second, pathLength(planner->getPath()), num_nodes, num_iterations, start_, goal_, slns.back().first};
        }
        else
        {
            out[name] = {false, planner->get_final_path_use_time_(), DBL_MAX, num_nodes, num_iterations, start_, goal_, DBL_MAX};
        }
    }

    void run_planners_and_store(const BRRTInput& input, int scenario = -1, int trial = -1) {
        if (!env_ptr_->isStateValid(start_)) {
            Eigen::Vector3d old_start = start_;
            start_ = env_ptr_->findNearestFree(start_);
            ROS_WARN("Start position [%.2f, %.2f, %.2f] is in obstacle, shifted to [%.2f, %.2f, %.2f]", 
                     old_start.x(), old_start.y(), old_start.z(), start_.x(), start_.y(), start_.z());
        }
        if (!env_ptr_->isStateValid(goal_)) {
            Eigen::Vector3d old_goal = goal_;
            goal_ = env_ptr_->findNearestFree(goal_);
            ROS_WARN("Goal position [%.2f, %.2f, %.2f] is in obstacle, shifted to [%.2f, %.2f, %.2f]", 
                     old_goal.x(), old_goal.y(), old_goal.z(), goal_.x(), goal_.y(), goal_.z());
        }
        
        print_vector3d("start", start_);
        print_vector3d("goal", goal_);
        std::map<std::string, AlgoResult> algo_outputs;

        if (run_rrt_)
            runAndRecord("RRT", rrt_ptr_, algo_outputs);
        if (run_rrt_star_)
            runAndRecord("RRT_Star", rrt_star_ptr_, algo_outputs);

        // RRT-Connect baseline
        if (run_brrt_)
        {
            brrt_ptr_->set_test_param(input.epsilon);
            runAndRecord("BRRT", brrt_ptr_, algo_outputs);
        }

        // GB-RRT-Connect baseline
        if (run_goal_bias_brrt_)
        {
            goal_bias_brrt_ptr_->set_test_param(input.epsilon);
            runAndRecord("GoalBiasBRRT", goal_bias_brrt_ptr_, algo_outputs);
        }

        if (run_brrt_optimize_)
        {
            // Heuristic guide-pair BiRRT without LEA/APF (candidate for the EBi-RRT baseline)
            brrt_simple_case1_ptr_->set_heuristic_param(input.p1, input.u_p, input.alpha, input.beta, input.gamma, input.epsilon);
            runAndRecord("BRRT_Case1", brrt_simple_case1_ptr_, algo_outputs);

            // RRT-FConnect (the paper's method)
            sof_brrt_simple_case3_ptr_->set_heuristic_param(input.p1, input.u_p, input.alpha, input.beta, input.gamma, input.epsilon, input.p2, input.n_rays, input.weight_grade, input.ray_step_size);
            runAndRecord("SOF_BRRT_Case3", sof_brrt_simple_case3_ptr_, algo_outputs);
        }

        manager->store_output_for_run(algo_outputs, scenario, trial);
    }

    void experiment_test()
    {
        if (generate_mode_) {
            std::ofstream json_file;
            json_file.open(dataset_path_ + "_scenarios.json");
            json_file << "[\n";
            for (int i=0; i<20; i++) {
                Eigen::Vector3d s_, g_;
                int attempts = 0;
                while (true) {
                    s_ = get_sample_valid();
                    g_ = get_sample_valid();
                    if ((s_ - g_).norm() >= gen_min_dist_) break;
                    attempts++;
                    if (attempts > 10000) {
                        ROS_WARN("Could not find a goal point far enough (%.2f) from start after 10000 attempts. Using closest found.", gen_min_dist_);
                        break;
                    }
                }
                json_file << "  {\"start\": [" << s_[0] << "," << s_[1] << "," << s_[2] << "],";
                json_file << " \"goal\": [" << g_[0] << "," << g_[1] << "," << g_[2] << "]}";
                if(i < 19) json_file << ",\n";
                else json_file << "\n";
            }
            json_file << "]\n";
            json_file.close();
            ROS_INFO("Generated 20 start/goals at %s with min_dist %.2f", (dataset_path_ + "_scenarios.json").c_str(), gen_min_dist_);
            ros::shutdown();
            return;
        }

        const auto &input = manager->get_input();
        
        if (!scenarios_file_.empty()) {
            std::ifstream file(scenarios_file_);
            if (!file.is_open()) {
                ROS_ERROR("Failed to open scenario file: %s", scenarios_file_.c_str());
                ros::shutdown();
                return;
            }
            json scenarios;
            file >> scenarios;
            
            ROS_INFO("Running batch scenarios from %s", scenarios_file_.c_str());
            for (size_t i = 0; i < scenarios.size(); ++i) {
                ROS_INFO("Scenario %zu/%zu", i + 1, scenarios.size());
                auto s = scenarios[i].at("start");
                auto g = scenarios[i].at("goal");
                for (int t = 0; t < trials_per_query_; ++t) {
                    start_ << s[0], s[1], s[2];
                    goal_ << g[0], g[1], g[2];
                    run_planners_and_store(input, (int)i, t);
                }
            }
            manager->save_json();
            ROS_INFO("Completed all scenarios.");
            ros::shutdown();
            return;
        }

        ROS_INFO("Running Test %d", input.trial);
        
        int num_runs = use_fixed_start_goal_ ? 1 : NUMBER_TEST_TIMES;

        ROS_WARN_STREAM("use_fixed_start_goal: "<<use_fixed_start_goal_);
        for (int i = 0; i < num_runs; ++i)
        {
            ROS_INFO("Running Test %d of %d", i + 1, num_runs);

            if (use_fixed_start_goal_) {
                start_[0] = fixed_start_x_; start_[1] = fixed_start_y_; start_[2] = fixed_start_z_;
                goal_[0] = fixed_goal_x_; goal_[1] = fixed_goal_y_; goal_[2] = fixed_goal_z_;
            } else {
                goal_ = get_sample_valid();
            }
            
            run_planners_and_store(input);

            if (!use_fixed_start_goal_) {
                start_ = goal_;
            }
        }
        manager->save_json();
        ROS_INFO("Completed all runs.");
        ros::shutdown();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_path_finder_node");
    ros::NodeHandle nh("~");

    TesterPathFinder tester(nh);

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}