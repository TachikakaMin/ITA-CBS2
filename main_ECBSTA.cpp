#include <iostream>
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <string>
#include <yaml-cpp/yaml.h>
#include "include/common.hpp"
#include "include/ECBSTA/ECBSTA.hpp"

namespace po = boost::program_options;

unordered_set<Location> obstacles;
vector<unordered_set<Location> > goals;
unordered_map<Location, int> goal_to_idx;
unordered_map<int, Location> idx_to_goal;
vector<State> start_states;
po::variables_map vm;
int row_number,col_number;
string outputFile;

int init_map(int argc, char** argv)
{

    po::options_description desc("Allowed options");
    string inputFile;
    desc.add_options()
            ("help", "produce help message")
            // params for the input instance and experiment settings
            ("input,i", po::value<std::string>(&inputFile)->required(), "input file (YAML)")
            ("output,o", po::value<std::string>(&outputFile)->required(), "output file (YAML)")
            ("weight,w", po::value<float>()->default_value(1), "weight for ECBS")
            ("nodeLimit", po::value<int>()->default_value(MAX_NODES), "node limit")
            ("seed,d", po::value<int>()->default_value(0), "random seed")
            ("stats", po::value<bool>()->default_value(false), "write to files some statistics")
            ("restart", po::value<int>()->default_value(1), "number of restart times (at least 1)");

    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);

        if (vm.count("help") != 0u) {
            std::cout << desc << "\n";
            return -1;
        }
    } catch (po::error &e) {
        std::cerr << e.what() << std::endl << std::endl;
        std::cerr << desc << std::endl;
        return -1;
    }

    YAML::Node config = YAML::LoadFile(inputFile);

    if (config["map"].IsMap()) {
        const auto &dim = config["map"]["dimensions"];
        row_number = dim[0].as<int>();
        col_number = dim[1].as<int>();


        for (const auto &node: config["map"]["obstacles"]) {
            obstacles.insert(Location(node[0].as<int>(), node[1].as<int>()));
        }
    }
    else {
        const auto& file_name = config["map"].as<string>();
        std::filesystem::path fullPath(inputFile);
        std::filesystem::path folderPath = fullPath.parent_path();
        std::filesystem::path map_file_path = folderPath / file_name;
        vector<vector<bool> > ret_map;
        read_map_file(map_file_path, ret_map);
        row_number = ret_map.size();
        col_number = ret_map[0].size();
        for (int i=0;i<row_number;i++)
            for (int j=0;j<col_number;j++)
                if (ret_map[i][j])
                    obstacles.insert(Location(i, j));
    }

    unordered_set<Location> all_goal_location_set;

    for (const auto &node: config["agents"]) {
        const auto &start = node["start"];
        start_states.emplace_back(State(0, start[0].as<int>(), start[1].as<int>()));
        goals.resize(goals.size() + 1);
        for (const auto &goal: node["potentialGoals"]) {
            Location x = Location(goal[0].as<int>(), goal[1].as<int>());
            goals.back().emplace(x);
            all_goal_location_set.insert(x);
        }
    }

    int cnt = 0;
    for (const auto& location:all_goal_location_set)
    {
        goal_to_idx[location] = cnt;
        idx_to_goal[cnt] = location;
        cnt ++;
    }


    // sanity check: no identical start states
    unordered_set<State> all_start_states_set;

    for (const auto &s: start_states) {
        if (all_start_states_set.find(s) != all_start_states_set.end()) {
            std::cout << "Identical start states detected -> no solution!" << std::endl;
            return -1;
        }
        all_start_states_set.insert(s);
    }
    return 0;
}

int main(int argc, char** argv) {

    if (init_map(argc, argv) < 0)
    {
        std::cout<< "Error Map" <<std::endl;
        return 0;
    }
    std::cout<< "Load Map Done" <<std::endl;
    ECBSTA ecbsta(row_number, col_number, obstacles, goals, start_states, goal_to_idx, idx_to_goal, vm["weight"].as<float>());

    int runs = vm["restart"].as<int>();
    for (int i = 0; i < runs; i++) {
        ecbsta.clear();
        ecbsta.total_timer.reset();
        ecbsta.solve();
        ecbsta.total_timer.stop();
        ecbsta.total_runtime = ecbsta.total_timer.elapsedSeconds();
        if (ecbsta.solution_found) break;
    }
    std::ofstream out(outputFile);
    out << "statistics:" << std::endl;
    out << "  cost: " << ecbsta.cost << std::endl;
    out << "  TA_runtime: " << ecbsta.ta_runtime << std::endl;
    out << "  runtime: " << ecbsta.total_runtime << std::endl;
    out << "  newnode_runtime: " << ecbsta.newnode_time << std::endl;
    out << "  firstconflict_runtime: " << ecbsta.conflict_num_time << std::endl;
    out << "  lowlevel_search_time: " << ecbsta.lowlevel_search_time << std::endl;
    out << "  focal_score_time: " << ecbsta.focal_score_time << std::endl;
    out << "  total_lowlevel_node: " << ecbsta.cbsnode_num << std::endl;
    out << "  lowLevelExpanded: " << ecbsta.lowLevelExpanded << std::endl;
    out << "  numTaskAssignments: " << ecbsta.num_ta << std::endl;
    out << "  shortest_path_times: " << ecbsta.invoke_shortest_path_times << std::endl;
    out << "schedule:" << std::endl;
    for (size_t a = 0; a < ecbsta.out_solution.size(); ++a) {
        out << "  agent" << a << ":" << std::endl;
        for (const auto &state: *(ecbsta.out_solution[a])) {
            out << "    - x: " << state.state.x << std::endl
                << "      y: " << state.state.y << std::endl
                << "      t: " << state.state.time << std::endl;
        }
    }
    return 0;
}
