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
    int agent_n;
    vector<State> start_states;
    unordered_set<Location> obstacles;
    vector<unordered_set<Location> > goals;
    unordered_map<Location, int> goal_to_idx;
    unordered_map<int, Location> idx_to_goal;
    vector<vector<int> > map2d_obstacle;
    unordered_map<int, vector<vector<int> > > prior_hmap;
    vector<vector<bool> > assignment_allow_map;
    unordered_set<State, boost::hash<State> > closedSet;
    vector<shared_ptr<Path > > out_solution;
    int cost, map_size, cbsnode_num, lowLevelExpanded, num_ta, num_ta_change;
    int row_number, col_number;


    Timer ta_timer, total_timer, lowlevel_search_timer, newnode_timer, firstconflict_timer;
    double ta_runtime, total_runtime,  lowlevel_search_time, newnode_time, firstconflict_time;

    ~ITACBS();
    ITACBS(int row_number, int col_number, unordered_set<Location>& obstacles,
           vector<unordered_set<Location> >& goals, vector<State>& startStates,
           unordered_map<Location, int>& goal_to_idx, unordered_map<int, Location>& idx_to_goal);
    void clear();
    int solve();
//    vector<Constraints>* create_constraints_from_conflict(Conflict& conflict);
    int heuristic(int x1, int y1, Location goal_loc);
    bool searchNodeIsValid(shared_ptr<Constraints>& agent_constraint_set, const State& new_state, const State& org_state);


//    inline int linearizeCoordinate(Location loc) const { return ( this->col_number * loc.x + loc.y); }
//    inline int linearizeCoordinate(State loc) const { return ( this->col_number * loc.x + loc.y); }
//    inline Location getCoordinate(int id) const { return Location(id / this->col_number, id % this->col_number); }
//    inline int getRowCoordinate(int id) const { return id / this->col_number; }
//    inline int getColCoordinate(int id) const { return id % this->col_number; }
//    inline int getManhattanDistance(int loc1, int loc2) const;
//    list<int> getNeighbors(int curr) const;
//    inline bool validMove(int curr, int next) const;

    shared_ptr<Path> findPath_a_star(shared_ptr<Constraints>&  agent_constraint_set, int agent_idx, int goal_loc_idx);

    typedef typename boost::heap::d_ary_heap<shared_ptr<PathEntry>, boost::heap::arity<2>,
            boost::heap::mutable_<true>, boost::heap::compare<PathEntryCompare> >
            openSet_t;
    using PathEntryHandle = openSet_t::handle_type;
};

#endif //ITACBS_REMAKE_ITACBS_HPP
