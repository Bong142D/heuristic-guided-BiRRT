#pragma once
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

struct BRRTInput {
    int trial;
    std::string environment;
    double p1;
    double u_p;
    double alpha;
    double beta;
    double gamma;
    double epsilon;
    double p2;
    int n_rays;
    int weight_grade;
    double ray_step_size;

    BRRTInput() 
        : trial(0), environment(""), p1(0.0), u_p(0.0), alpha(0.0), beta(0.0), 
          gamma(0.0), epsilon(0.0), p2(0.5), n_rays(9), weight_grade(0), ray_step_size(0.1) {}
};

struct AlgoResult {
    bool success;
    double search_time;   // seconds
    double path_length;   // geometric length of the returned path
    int node_count;
    int num_iterations;
    Eigen::Vector3d start;
    Eigen::Vector3d goal;
    double path_cost = 0; // planner's internal cost (steer_length per edge)
};

struct RunResult {
    int run;
    int scenario;
    int trial;
    std::map<std::string, AlgoResult> algorithms;
};

class BRRTExperimentMultiAlgo {
public:
    BRRTExperimentMultiAlgo(const std::string& json_input_path, const std::string& json_output_path)
        : json_out_path_(json_output_path), run_index_(1) {
        load_from_json(json_input_path);
    }

    const BRRTInput& get_input() const {
        return input_;
    }

    void store_output_for_run(const std::map<std::string, AlgoResult>& algo_results, int scenario = -1, int trial = -1) {
        RunResult run;
        run.run = run_index_;
        run.scenario = scenario;
        run.trial = trial;
        run.algorithms = algo_results;
        results_.push_back(run);
        run_index_++;
    }

    int current_run_index() const {
        return run_index_;
    }

    void save_json() const {
        json j;
        j["trial"] = input_.trial;
        j["environment"] = input_.environment;
        j["parameters"] = {
            {"p1", input_.p1},
            {"u_p", input_.u_p},
            {"alpha", input_.alpha},
            {"beta", input_.beta},
            {"gamma", input_.gamma},
            {"epsilon", input_.epsilon},
            {"p2", input_.p2},
            {"n_rays", input_.n_rays},
            {"weight_grade", input_.weight_grade},
            {"ray_step_size", input_.ray_step_size}
        };

        json result_array = json::array();
        for (const auto& run : results_) {
            json algos;
            for (const auto& [name, r] : run.algorithms) {
                algos[name] = {
                    {"success", r.success},
                    {"search_time", r.search_time},
                    {"path_length", r.path_length},
                    {"path_cost", r.path_cost},
                    {"node_count", r.node_count},
                    {"num_iterations", r.num_iterations},
                    {"start", {r.start[0], r.start[1], r.start[2]}},
                    {"goal", {r.goal[0], r.goal[1], r.goal[2]}}
                };
            }
            result_array.push_back({
                {"run", run.run},
                {"scenario", run.scenario},
                {"trial", run.trial},
                {"algorithms", algos}
            });
        }

        j["results"] = result_array;

        std::ofstream file(json_out_path_);
        std::cout << "Saving results to: " << json_out_path_ << std::endl;
        file << j.dump(4);
    }

private:
    BRRTInput input_;
    std::vector<RunResult> results_;
    std::string json_out_path_;
    int run_index_;

    void load_from_json(const std::string& path) {
        std::ifstream file(path);
        if (!file.is_open()) {
            std::cerr << "Failed to open input JSON file: " << path << std::endl;
            return;
        }

        json j;
        file >> j;
        
        // Mandatory fields using .at() which will throw if missing
        if (j.contains("trial")) input_.trial = j.at("trial").get<int>();
        if (j.contains("environment")) input_.environment = j.at("environment").get<std::string>();
        
        // Use .value() for robust default handling where appropriate, or .at() for mandatory ones
        input_.p1 = j.contains("p1") ? j.at("p1").get<double>() : 0.7;
        input_.u_p = j.contains("u_p") ? j.at("u_p").get<double>() : 0.0;
        input_.alpha = j.contains("alpha") ? j.at("alpha").get<double>() : 2.0;
        input_.beta = j.contains("beta") ? j.at("beta").get<double>() : 8.0;
        input_.gamma = j.contains("gamma") ? j.at("gamma").get<double>() : 8.0;
        input_.epsilon = j.contains("epsilon") ? j.at("epsilon").get<double>() : 1.0;
        
        input_.p2 = j.value("p2", 0.5);
        input_.n_rays = j.value("n_rays", 9);
        input_.weight_grade = j.value("weight_grade", 0);
        input_.ray_step_size = j.value("ray_step_size", 0.1);
    }
};
