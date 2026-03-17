//
// Created by YIMIN TANG on 3/19/23.
//

#ifndef ITACBS_REMAKE_ITACBS_HPP
#define ITACBS_REMAKE_ITACBS_HPP

#include "../common.hpp"

class ITACBS
{

public:

    bool solution_found;
    int diff_goal_n;
    int diff_start_n;
    int agent_n;
    vector<State> start_states;
    unordered_set<Location> obstacles;
    vector<unordered_set<Location> > goals;
    vector<unordered_set<Location> > dropoffGoals;
    vector<bool> agent_status;
    vector<int> agent_past_path_cost;
    vector<int> agent_current_hold_ore;
    vector<int> agent_capacity;
    vector<int> agent_current_target_goal;
    unordered_map<Location, int> goal_to_idx;
    unordered_map<int, Location> idx_to_goal;
    unordered_map<int, int> idx_to_ore;
    unordered_map<Location, int> start_to_idx;
    unordered_map<int, Location> idx_to_start;
    vector<vector<int> > map2d_obstacle;
    unordered_map<int, vector<vector<int> > > prior_hmap;
    unordered_map<int, vector<vector<int> > > prior_hmap_back;
    vector<vector<bool> > assignment_allow_map;
    unordered_set<State, boost::hash<State> > closedSet;
    vector<shared_ptr<Path > > out_solution;
    // Final task assignment: agent id -> goal index in idx_to_goal.
    unordered_map<int, int> out_TA_solution;
    double cost;
    int map_size, cbsnode_num, lowLevelExpanded, num_ta, num_ta_change;
    int row_number, col_number;


    Timer ta_timer, total_timer, lowlevel_search_timer, newnode_timer, firstconflict_timer;
    double ta_runtime, total_runtime,  lowlevel_search_time, newnode_time, firstconflict_time;

    ~ITACBS();
    ITACBS(int row_number, int col_number, unordered_set<Location>& obstacles,
           vector<unordered_set<Location> >& goals,vector<unordered_set<Location> >& dropoffGoals,
           vector<State>& start_states, vector<bool>& agent_status,
           vector<int>& agent_past_path_cost, vector<int>& agent_current_hold_ore, vector<int>& agent_capacity,
           vector<int>& agent_current_target_goal,
           unordered_map<Location, int>& goal_to_idx, unordered_map<int, Location>& idx_to_goal, unordered_map<int, int>& idx_to_ore,
           unordered_map<Location, int>& start_to_idx, unordered_map<int, Location>& idx_to_start);
    void clear();
    int solve_with_back();
    int heuristic(int x1, int y1, Location goal_loc);
    bool searchNodeIsValid(shared_ptr<Constraints>& agent_constraint_set, const State& new_state, const State& org_state);


    shared_ptr<Path> findPath_a_star(
        shared_ptr<Constraints>&  agent_constraint_set,
        int agent_idx, int goal_loc_idx);

    shared_ptr<Path> findPath_a_star_with_back(
        shared_ptr<Constraints>&  agent_constraint_set,
        int agent_idx, int goal_loc_idx,
        bool if_back, State goal_reach);

    shared_ptr<Path> findPath_with_back(
        shared_ptr<Constraints>& agent_constraint_set,
        int agent_idx, int goal_loc_idx);

    typedef typename boost::heap::d_ary_heap<shared_ptr<PathEntry>, boost::heap::arity<2>,
            boost::heap::mutable_<true>, boost::heap::compare<PathEntryCompare> >
            openSet_t;
    using PathEntryHandle = openSet_t::handle_type;
};

#endif //ITACBS_REMAKE_ITACBS_HPP
