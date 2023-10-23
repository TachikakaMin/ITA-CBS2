//
// Created by YIMIN TANG on 3/19/23.
//

#include "ITACBS.hpp"
#include "ITACBSNode.hpp"


ITACBS::~ITACBS() = default;

void ITACBS::clear() {
    solution_found = false;
    this->ta_runtime = 0;
    this->total_runtime = 0;
    this->lowlevel_search_time = 0;
    this->newnode_time = 0;
    this->firstconflict_time = 0;
    this->cbsnode_num = 0;
    this->lowLevelExpanded = 0;
    this->num_ta = 0;
    this->num_ta_change = 0;
}

class PairCompare {
public:
    bool operator()(const pair<int, Location>& p1, const pair<int, Location>& p2) const {
        return p1.first > p2.first;
    }
};


ITACBS::ITACBS(int row_number, int col_number, unordered_set<Location>& obstacles,
               vector<unordered_set<Location> >& goals, vector<State>& start_states,
               unordered_map<Location, int>& goal_to_idx, unordered_map<int, Location>& idx_to_goal) {
    this->row_number = row_number;
    this->col_number = col_number;
    this->map_size = row_number * col_number;
    this->start_states = start_states;
    this->obstacles = obstacles;
    this->goals = goals;
    this->goal_to_idx = goal_to_idx;
    this->idx_to_goal = idx_to_goal;

    this->ta_runtime = 0;
    this->total_runtime = 0;
    this->lowlevel_search_time = 0;
    this->cbsnode_num = 0;
    this->lowLevelExpanded = 0;
    this->num_ta = 0;

    this->map2d_obstacle.resize(this->row_number, vector<int>(this->col_number, 0));
    for (auto obstacle : obstacles) {
        int x = obstacle.x;
        int y = obstacle.y;
        this->map2d_obstacle[x][y] = 1;
    }

    this->diff_goal_n = this->goal_to_idx.size();
    this->agent_n = this->start_states.size();

    this->assignment_allow_map.resize(this->agent_n, vector<bool>(this->diff_goal_n, false));
    for (int i=0; i<this->agent_n; i++)
    {
        const auto& agent_i_goal = this->goals[i];
        for (const auto& location: agent_i_goal)
        {
            int idx = this->goal_to_idx[location];
            this->assignment_allow_map[i][idx] = true;
        }
    }
    printf("Init shortest path\n");
    for (int i=0;i<this->diff_goal_n;i++)
    {
        vector<vector<int> > prior_map;
        prior_map.resize(this->row_number, vector<int>(this->col_number, INF7f));
        Location start_loc = idx_to_goal[i];
        prior_map[start_loc.x][start_loc.y] = 0;
        std::priority_queue<pair<int, Location>, vector<pair<int, Location> >, PairCompare > pq;
        pq.push(make_pair(0, start_loc));
        while (!pq.empty())
        {
            pair<int, Location> x = pq.top(); pq.pop();
            int c = x.first;
            Location cur_loc = x.second;

            for (int k = 0; k<4; k++)
            {
                Location new_loc(cur_loc.x + dx[k], cur_loc.y + dy[k]);
                if (new_loc.x < 0 || new_loc.x >= this->row_number || new_loc.y < 0 || new_loc.y >= this->col_number) continue;
                if (this->map2d_obstacle[new_loc.x][new_loc.y]) continue;
                if (c + 1 >= prior_map[new_loc.x][new_loc.y]) continue;
                prior_map[new_loc.x][new_loc.y] = c + 1;
                pq.push(make_pair(c+1, new_loc));
            }
        }
        this->prior_hmap[i] = std::move(prior_map);
    }
    printf("Init Done\n");
}




int ITACBS::heuristic(int x1, int y1, Location goal_loc) {
    int idx = this->goal_to_idx[goal_loc];
    return this->prior_hmap[idx][x1][y1];
}

bool ITACBS::searchNodeIsValid(shared_ptr<Constraints>&  agent_constraint_set, const State& new_state, const State& org_state) {
//    cout<<(state.x) <<" "<< (this->row_number) <<" "<< (state.y) <<" "<< (this->col_number) <<endl;

    if (new_state.x < 0 || new_state.x >= this->row_number || new_state.y < 0 || new_state.y >= this->col_number) return false;
    if (this->map2d_obstacle[new_state.x][new_state.y]) return false;
    if (closedSet.find(new_state) != closedSet.end()) return false;
    if (new_state.time >= MAX_TIMESTEP) return false;
    if (!agent_constraint_set->stateValid(new_state)) { return false;}
    if (!agent_constraint_set->transitionValid(org_state, new_state)) return false;
    return true;
}

shared_ptr<Path> ITACBS::findPath_a_star(shared_ptr<Constraints>& agent_constraint_set, int agent_idx, int goal_loc_idx)
{

    int search_max = 1e5;
    int m_lastGoalConstraint = -1;
    closedSet.clear();
    unordered_map<State, PathEntryHandle , boost::hash<State> > stateToHeap;
    openSet_t openSet;


    State start_state = this->start_states[agent_idx];

    Location goal_location = this->idx_to_goal[goal_loc_idx];

    for (const auto &vc: agent_constraint_set->vertexConstraints) {
        if (vc.x == goal_location.x && vc.y == goal_location.y) {
            m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
        }
    }

    shared_ptr<PathEntry> startNode(new PathEntry(start_state,
                                                    this->heuristic(start_state.x, start_state.y, goal_location),
                                                    0,
                                                    nullptr));

    auto handle = openSet.push(startNode);
    stateToHeap.insert(make_pair(start_state, handle));

    while (!openSet.empty() && search_max > 0) {
        this->lowLevelExpanded ++;
        search_max--;
        shared_ptr<PathEntry> current = openSet.top();

        if (current->state.x == goal_location.x && current->state.y == goal_location.y && current->state.time > m_lastGoalConstraint)
        {
            shared_ptr<Path> path(new Path);
            while (current != nullptr) {
                path->push_back(PathEntry(current->state, current->fScore, current->gScore, nullptr));
                current = current->parent;
            }
            reverse(path->begin(), path->end());
            return path;
        }

        openSet.pop();
        stateToHeap.erase(current->state);
        closedSet.insert(current->state);

        for (int i = 0; i < 5; i++) {
            State new_state(current->state.time + 1, current->state.x + dx[i], current->state.y + dy[i]);
            if (!this->searchNodeIsValid(agent_constraint_set, new_state, current->state)) continue;
            int tentative_gScore = current->gScore + 1;
            auto iter = stateToHeap.find(new_state);
            if (iter == stateToHeap.end()) {  // Discover a new node
                int fScore = tentative_gScore + this->heuristic(new_state.x, new_state.y, goal_location);
                shared_ptr<PathEntry> nextNode(new PathEntry(new_state, fScore,
                                                             tentative_gScore,
                                                             current));
                auto handle = openSet.push(nextNode);
                stateToHeap.insert(std::move(make_pair(new_state, handle)));

            } else {
                auto handle = iter->second;
                if (tentative_gScore >= (*handle)->gScore)  continue;
                int delta = (*handle)->gScore - tentative_gScore;
                (*handle)->gScore = tentative_gScore;
                (*handle)->fScore -= delta;
                (*handle)->parent = current;
                openSet.increase(handle);
            }
        }
    }
    return nullptr;
}


int ITACBS::solve() {
    this->cbsnode_num ++;
    this->newnode_timer.reset();
    int cnt_idx = 0;
    shared_ptr<ITACBSNode> start_node(new ITACBSNode(cnt_idx)); cnt_idx++;
    this->newnode_timer.stop();
    this->newnode_time += this->newnode_timer.elapsedSeconds();

    start_node->create_cost_matrix(this);
    {
        bool b = start_node->get_first_assignment(this);
        if (!b) return false;
    }
    typename boost::heap::d_ary_heap<shared_ptr<ITACBSNode>,
                boost::heap::mutable_<true>,
                boost::heap::arity<2>,
                boost::heap::compare<ITA_CBSNodePtrCmp> >
            open;

    open.push(start_node);

    while (!open.empty())
    {
        shared_ptr<ITACBSNode> cur_node = open.top(); open.pop();
        Conflict conflict;
        this->firstconflict_timer.reset();
        bool done = !cur_node->get_first_conflict(conflict);
        this->firstconflict_timer.stop();
        this->firstconflict_time += this->firstconflict_timer.elapsedSeconds();

        if (done)
        {
            std::cout << "done; cost: " << cur_node->cost << std::endl;
//            for (int i=0;i<agent_n;i++) printf("%d->(%d, %d)\n", i, idx_to_goal[cur_node->out_TA_solution[i]].x, idx_to_goal[cur_node->out_TA_solution[i]].y);
            this->out_solution = cur_node->out_solution;
            this->cost = cur_node->cost;
            this->solution_found = true;
            return true;
        }

        unordered_map<size_t, Constraints> tmp;
        createConstraintsFromConflict(conflict, tmp);

        for (unsigned short cur_i = 0; auto &[key, value]: tmp)
        {
            shared_ptr<ITACBSNode> new_node;
            cnt_idx++;
            this->newnode_timer.reset();
            if (cur_i == 1) {new_node = cur_node; new_node->idx = cnt_idx;}
                else new_node = shared_ptr<ITACBSNode>(new ITACBSNode(cur_node, cnt_idx));
            this->newnode_timer.stop();
            this->newnode_time += this->newnode_timer.elapsedSeconds();

            assert(!new_node->constraint_sets[key]->overlap(value));
            new_node->constraint_sets[key] = shared_ptr<Constraints>(new Constraints(*(new_node->constraint_sets[key])));
            new_node->constraint_sets[key]->add(value);
            cur_i ++;
            bool b = new_node->update_cost_matrix(this, key, value);
            if (b) {
                this->num_ta ++;
                this->ta_timer.reset();
                this->num_ta_change += new_node->get_next_assignment(key);
                this->ta_timer.stop();
                this->ta_runtime += this->ta_timer.elapsedSeconds();
            }
            this->cbsnode_num ++;
            open.push(new_node);
        }

    }
    return false;
}

