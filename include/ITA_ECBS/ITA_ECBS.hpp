

#ifndef ITA_ECBS_REMAKE_ITA_ECBS_HPP
#define ITA_ECBS_REMAKE_ITA_ECBS_HPP

#include "../common.hpp"

class ITA_ECBS
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
    int cost, map_size, cbsnode_num, lowLevelExpanded, num_ta;
    int row_number, col_number;
    float l_weight;
    int invoke_shortest_path_times;
    Timer ta_timer, total_timer, lowlevel_search_timer, newnode_timer, conflict_num_timer, focal_score_timer;
    double ta_runtime, total_runtime,  lowlevel_search_time, newnode_time, conflict_num_time, focal_score_time;

    ~ITA_ECBS();
    ITA_ECBS(int row_number, int col_number, unordered_set<Location>& obstacles,
           vector<unordered_set<Location> >& goals, vector<State>& startStates,
           unordered_map<Location, int>& goal_to_idx, unordered_map<int, Location>& idx_to_goal, float l_weight);
    void clear();
    int solve();

    int heuristic(int x1, int y1, Location goal_loc);
    bool searchNodeIsValid(shared_ptr<Constraints>& agent_constraint_set, const State& new_state, const State& org_state);

    int low_conflict_state(const int& agent_idx, const State& cur_state, const vector<shared_ptr<Path > >& out_solution);
    int low_conflict_transition(const int& agent_idx, const State& cur_state, const State& next_state,
                                const vector<shared_ptr<Path > >& out_solution);
    int high_focal_score(const vector<shared_ptr<Path > >& out_solution, Conflict& result);


    shared_ptr<Path> findPath_a_star_eps(shared_ptr<Constraints>&  agent_constraint_set, int agent_idx, int goal_loc_idx, const vector<shared_ptr<Path > >& out_solution, int fmin);
    shared_ptr<Path> findPath_a_star(shared_ptr<Constraints>& agent_constraint_set, int agent_idx, int goal_loc_idx);

    typedef typename boost::heap::d_ary_heap<shared_ptr<PathEntry>, boost::heap::arity<2>,
            boost::heap::mutable_<true>, boost::heap::compare<PathEntryCompare> >
            low_openSet_t;

    typedef typename boost::heap::d_ary_heap<shared_ptr<PathEntry>, boost::heap::arity<2>,
            boost::heap::mutable_<true>, boost::heap::compare<PathEntryCompare2> >
            low_openSet_t2;

    typedef typename low_openSet_t::handle_type PathEntryHandle;

    typedef typename low_openSet_t2::handle_type PathEntryHandle2;

    struct FocalPathEntryCompare{
        bool operator()(const PathEntryHandle& a, const PathEntryHandle& b) const {
            if ((*a)->focalScore != (*b)->focalScore) return (*a)->focalScore > (*b)->focalScore;
            else {
                if ((*a)->fScore != (*b)->fScore) return (*a)->fScore > (*b)->fScore;
            }
            return (*a)->gScore < (*b)->gScore;
        }
    };

    typedef typename boost::heap::d_ary_heap<
            PathEntryHandle, boost::heap::arity<2>, boost::heap::mutable_<true>,
            boost::heap::compare<FocalPathEntryCompare> >
            low_focalSet_t;
    using FocalPathEntryHandle = low_focalSet_t::handle_type;


};

#endif //ITA_ECBS_REMAKE_ITA_ECBS_HPP
