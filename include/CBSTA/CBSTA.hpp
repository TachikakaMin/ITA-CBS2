//
// Created by YIMIN TANG on 5/17/23.
//

#ifndef ITACBS_REMAKE_CBSTA_HPP
#define ITACBS_REMAKE_CBSTA_HPP

#include "../common.hpp"
#include "TANode.hpp"
#include "CBSTANode.hpp"


class CBSTA
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
    vector<vector<shared_ptr<Path > > > cost_matrix;
    int cost, cbsnode_num, lowLevelExpanded, num_ta;
    int row_number, col_number;

    Timer ta_timer, total_timer, lowlevel_search_timer, newnode_timer, firstconflict_timer;
    double ta_runtime, total_runtime,  lowlevel_search_time, newnode_time, firstconflict_time;



    ~CBSTA();
    CBSTA(int row_number, int col_number, unordered_set<Location>& obstacles,
           vector<unordered_set<Location> >& goals, vector<State>& startStates,
           unordered_map<Location, int>& goal_to_idx, unordered_map<int, Location>& idx_to_goal);
    void clear();
    int solve();
    int heuristic(int x1, int y1, Location goal_loc);
    bool searchNodeIsValid(shared_ptr<Constraints>& agent_constraint_set, const State& new_state, const State& org_state);
    bool get_next_TA(shared_ptr<TANode>& out_TA_node);
    bool init_TA(shared_ptr<TANode>& out_TA_node);
    shared_ptr<Path> findPath(shared_ptr<Constraints>&  agent_constraint_set, int agent_idx, int goal_loc_idx);
    void create_global_cost_matrix();
    bool update_CBSTANode_solution(shared_ptr<CBSTANode>& curnode, int agent_id);

    typedef typename boost::heap::d_ary_heap<shared_ptr<PathEntry>, boost::heap::arity<2>,
            boost::heap::mutable_<true>, boost::heap::compare<PathEntryCompare> >
            openSet_t;

    typedef typename openSet_t::handle_type PathEntryHandle;


    typename boost::heap::d_ary_heap<shared_ptr<TANode>,
            boost::heap::mutable_<true>,
            boost::heap::arity<2>,
            boost::heap::compare<TANodePtrCmp> >
            ta_open;
};

#endif //ITACBS_REMAKE_CBSTA_HPP
