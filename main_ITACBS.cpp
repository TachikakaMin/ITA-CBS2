#include <iostream>
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <string>
#include <yaml-cpp/yaml.h>
#include "include/ITACBS/ITACBS.hpp"
#include "include/common.hpp"

namespace po = boost::program_options;

unordered_set<Location> obstacles;
vector<unordered_set<Location> > goals;
vector<unordered_set<Location> > dropoffGoals;
unordered_map<Location, int> goal_to_idx;
unordered_map<int, Location> idx_to_goal;
unordered_map<int, int> idx_to_ore;
vector<bool> agent_status;
vector<int> agent_past_path_cost;
vector<int> agent_current_hold_ore;
vector<int> agent_capacity;
vector<int> agent_current_target_goal;
unordered_map<Location, int> start_to_idx;
unordered_map<int, Location> idx_to_start;
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
    YAML::Node mapinfo = config["mapinfo"] ? config["mapinfo"] : config;

    if (!mapinfo["map"]) {
        std::cerr << "Missing map/mapinfo.map in input file: " << inputFile << std::endl;
        return -1;
    }

    if (mapinfo["map"].IsMap()) {
        const auto &dim = mapinfo["map"]["dimensions"];
        row_number = dim[0].as<int>();
        col_number = dim[1].as<int>();


        for (const auto &node: mapinfo["map"]["obstacles"]) {
            obstacles.insert(Location(node[0].as<int>(), node[1].as<int>()));
        }
    }
    else {
        const auto& file_name = mapinfo["map"].as<string>();
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
    unordered_set<Location> all_start_location_set;
    vector<Location> indexed_dropoff_goals;

    vector<int> ore_values;
    for (const auto &goal: mapinfo["potentialGoalsOre"])
    {
        ore_values.push_back(goal.as<int>());
    }

    int cnt = 0;
    for (const auto &goal: mapinfo["potentialGoals"])
    {
        Location x = Location(goal[0].as<int>(), goal[1].as<int>());
        if (goal_to_idx.find(x) == goal_to_idx.end()) {
            int idx = static_cast<int>(goal_to_idx.size());
            goal_to_idx[x] = idx;
            idx_to_goal[idx] = x;
        }
        int idx = goal_to_idx[x];
        int ore = 0;
        if (cnt < static_cast<int>(ore_values.size())) ore = ore_values[cnt];
        idx_to_ore[idx] = ore;
        cnt++;
    }

    for (const auto &goal: mapinfo["potentialDropoffGoals"])
    {
        Location x = Location(goal[0].as<int>(), goal[1].as<int>());
        indexed_dropoff_goals.push_back(x);
        if (goal_to_idx.find(x) == goal_to_idx.end()) {
            int idx = static_cast<int>(goal_to_idx.size());
            goal_to_idx[x] = idx;
            idx_to_goal[idx] = x;
            idx_to_ore[idx] = 0;
        }
    }

    for (const auto &node: config["agents"]) {
        const auto &start = node["start"];
        Location xx = Location(start[0].as<int>(), start[1].as<int>());
        all_start_location_set.insert(xx);

        if (node["isDroppingoff"])
            agent_status.push_back(node["isDroppingoff"].as<bool>());
        else
            agent_status.push_back(false);

        if (node["pastPathCost"])
            agent_past_path_cost.push_back(node["pastPathCost"].as<int>());
        else
            agent_past_path_cost.push_back(0);

        if (node["currentHoldOre"])
            agent_current_hold_ore.push_back(node["currentHoldOre"].as<int>());
        else
            agent_current_hold_ore.push_back(0);

        if (node["capacity"])
            agent_capacity.push_back(node["capacity"].as<int>());
        else
            agent_capacity.push_back(0);

        int current_target_goal_idx = -1;
        if (node["currentTarget"] && node["currentTarget"].IsSequence() && node["currentTarget"].size() == 2) {
            Location target(node["currentTarget"][0].as<int>(), node["currentTarget"][1].as<int>());
            auto it = goal_to_idx.find(target);
            if (it != goal_to_idx.end()) {
                current_target_goal_idx = it->second;
            }
        }
        agent_current_target_goal.push_back(current_target_goal_idx);

        start_states.emplace_back(State(0, start[0].as<int>(), start[1].as<int>()));
        goals.resize(goals.size() + 1);
        for (const auto &goal: node["potentialGoals"]) {
            int idx = goal.as<int>();
            if (idx_to_goal.find(idx) == idx_to_goal.end()) {
                std::cerr << "Invalid ore goal index " << idx << " in input file: " << inputFile << std::endl;
                return -1;
            }
            goals.back().emplace(idx_to_goal[idx]);
        }

        dropoffGoals.resize(dropoffGoals.size() + 1);
        for (const auto &goal: node["potentialDropoffGoals"]) {
            Location x;
            if (goal.IsScalar()) {
                int idx = goal.as<int>();
                if (idx < 0 || idx >= static_cast<int>(indexed_dropoff_goals.size())) {
                    std::cerr << "Invalid dropoff goal index " << idx << " in input file: " << inputFile << std::endl;
                    return -1;
                }
                x = indexed_dropoff_goals[idx];
            } else {
                x = Location(goal[0].as<int>(), goal[1].as<int>());
            }
            dropoffGoals.back().emplace(x);
            all_goal_location_set.insert(x);
        }
    }


    cnt = 0;
    for (const auto& location:all_start_location_set)
    {
        start_to_idx[location] = cnt;
        idx_to_start[cnt] = location;
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
        return 1;
    }
    std::cout<< "Load Map Done" <<std::endl;
    ITACBS itacbs(row_number, col_number, obstacles,
        goals, dropoffGoals, start_states, agent_status,
        agent_past_path_cost, agent_current_hold_ore, agent_capacity,
        agent_current_target_goal,
        goal_to_idx, idx_to_goal, idx_to_ore,
        start_to_idx, idx_to_start
    );

    int runs = vm["restart"].as<int>();
    for (int i = 0; i < runs; i++) {
        itacbs.clear();
        itacbs.total_timer.reset();
        itacbs.solve_with_back();
        itacbs.total_timer.stop();
        itacbs.total_runtime = itacbs.total_timer.elapsedSeconds();
        if (itacbs.solution_found) break;
    }
    std::ofstream out(outputFile);
    out << "statistics:" << std::endl;
    out << "  cost: " << itacbs.cost << std::endl;
    out << "  teamSize: " << itacbs.out_solution.size() << std::endl;
    // Emit per-agent task assignment (goal index + coordinates) for downstream tooling.
    out << "task_assignment_field_guide:" << std::endl;
    out << "  agent: \"Agent index.\"" << std::endl;
    out << "  mode: \"pickup or dropoff based on current mission.\"" << std::endl;
    out << "  goal_idx: \"Goal index in the global goal list (idx_to_goal).\"" << std::endl;
    out << "  goal: \"Goal coordinate [x, y].\"" << std::endl;
    out << "  ore: \"Ore amount at goal (0 for dropoff goals).\"" << std::endl;
    out << "task_assignment:" << std::endl;
    for (size_t a = 0; a < itacbs.agent_n; ++a) {
        out << "  - agent: " << a << std::endl;
        out << "    mode: " << (itacbs.agent_status[a] ? "dropoff" : "pickup") << std::endl;
        auto it = itacbs.out_TA_solution.find(static_cast<int>(a));
        if (it == itacbs.out_TA_solution.end()) {
            out << "    goal_idx: -1" << std::endl;
            out << "    goal: null" << std::endl;
            continue;
        }
        int goal_idx = it->second;
        out << "    goal_idx: " << goal_idx << std::endl;
        auto loc_it = itacbs.idx_to_goal.find(goal_idx);
        if (loc_it != itacbs.idx_to_goal.end()) {
            out << "    goal:" << std::endl;
            out << "      - " << loc_it->second.x << std::endl;
            out << "      - " << loc_it->second.y << std::endl;
        } else {
            out << "    goal: null" << std::endl;
        }
        auto ore_it = itacbs.idx_to_ore.find(goal_idx);
        if (ore_it != itacbs.idx_to_ore.end()) {
            out << "    ore: " << ore_it->second << std::endl;
        }
    }
    out << "schedule: " << std::endl;
    for (size_t a = 0; a < itacbs.out_solution.size(); ++a) {
        out << "  agent" << a << ":" << std::endl;
        for (const auto &state: *(itacbs.out_solution[a])) {
            out << "    - x: " << state.state.x << std::endl
                << "      y: " << state.state.y << std::endl
                << "      t: " << state.state.time << std::endl;
        }
    }
    return 0;
}
