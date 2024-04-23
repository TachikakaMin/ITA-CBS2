#include "ECBSTA.hpp"
#include "TANode.hpp"
#include "ECBSTANode.hpp"


ECBSTA::~ECBSTA() = default;

void ECBSTA::clear() {
    solution_found = false;
    this->ta_runtime = 0;
    this->total_runtime = 0;
    this->lowlevel_search_time = 0;
    this->firstconflict_time = 0;
    this->newnode_time = 0;
    this->cbsnode_num = 0;
    this->lowLevelExpanded = 0;
    this->num_ta = 0;
}


class PairCompare {
public:
    bool operator()(const pair<int, Location>& p1, const pair<int, Location>& p2) const {
        return p1.first > p2.first;
    }
};



shared_ptr<Path> ECBSTA::findPath_a_star_eps(shared_ptr<Constraints>& agent_constraint_set,
                                               int agent_idx, int goal_loc_idx,
                                               const vector<shared_ptr<Path > >& out_solution,
                                               vector<int >& fmin)
{
    this->invoke_shortest_path_times++;
    int search_max = 1e5;
    int m_lastGoalConstraint = -1;
    closedSet.clear();
    unordered_map<State, PathEntryHandle , boost::hash<State> > stateToHeap;

    unordered_map<tuple<int, int, int>, int, tuple_hash, tuple_equal> pointCheckMap;
    unordered_map<tuple<int, int>, int, tuple_hash, tuple_equal> lastPointCheckMap;
    unordered_map<tuple<int, int, int, int, int>, int, tuple_hash, tuple_equal> edgeCheckMap;

    get_block_map(pointCheckMap, lastPointCheckMap,  edgeCheckMap, out_solution, agent_idx);

    low_openSet_t openSet;
    low_focalSet_t focalSet;

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
                                                  0,
                                                  nullptr));

    auto handle = openSet.push(startNode);
    stateToHeap.insert(make_pair(start_state, handle));
    focalSet.push(handle);

    int best_fScore = startNode->fScore;

    while (!openSet.empty() && search_max > 0) {
        this->lowLevelExpanded ++;
        search_max--;

        int old_best_fScore = best_fScore;
        best_fScore = openSet.top()->fScore;
        if (best_fScore > old_best_fScore)
        {
            for (auto iter = openSet.ordered_begin(); iter!=openSet.ordered_end(); iter++)
            {
                int val = (*iter)->fScore;
                if (val > old_best_fScore * this->l_weight && val <= best_fScore * l_weight)
                {
                    auto handle_iter = stateToHeap.find((*iter)->state);
                    focalSet.push(handle_iter->second);
                }
                if (val > best_fScore * l_weight) break;
            }
        }

        auto current_handler = focalSet.top();
        shared_ptr<PathEntry> current = (*current_handler);

        if (current->state.x == goal_location.x && current->state.y == goal_location.y && current->state.time > m_lastGoalConstraint)
        {
            fmin[agent_idx] = best_fScore;
            shared_ptr<Path> path(new Path);
            while (current != nullptr) {
                path->push_back(PathEntry(current->state, current->fScore, current->gScore, nullptr));
                current = current->parent;
            }
            reverse(path->begin(), path->end());
            return path;
        }

        focalSet.pop();
        openSet.erase(current_handler);
        stateToHeap.erase(current->state);
        closedSet.insert(current->state);

        for (int i = 0; i < 5; i++) {
            State new_state(current->state.time + 1, current->state.x + dx[i], current->state.y + dy[i]);
            if (!this->searchNodeIsValid(agent_constraint_set, new_state, current->state)) continue;
            int tentative_gScore = current->gScore + 1;
            auto iter = stateToHeap.find(new_state);
            if (iter == stateToHeap.end()) {  // Discover a new node
                int fScore = tentative_gScore + this->heuristic(new_state.x, new_state.y, goal_location);

                this->focal_score_timer.reset();

                auto idx = make_tuple(new_state.x, new_state.y, new_state.time);
                int conflict_num =  pointCheckMap[idx];
                auto idx2 = make_tuple(new_state.x, new_state.y);
                if (lastPointCheckMap.find(idx2) != lastPointCheckMap.end())
                {
                    int t = lastPointCheckMap[idx2];
                    if (new_state.time > t) conflict_num++;
                }
                auto ky_tuple = make_tuple(current->state.x, current->state.y, current->state.time, new_state.x, new_state.y);
                conflict_num += edgeCheckMap[ky_tuple];
                int focalScore = current->focalScore + conflict_num;
                this->focal_score_timer.stop();
                this->focal_score_time += this->focal_score_timer.elapsedSeconds();

                shared_ptr<PathEntry> nextNode(new PathEntry(new_state,
                                                             fScore,
                                                             tentative_gScore,
                                                             focalScore,
                                                             current));

                auto handle = openSet.push(nextNode);
                stateToHeap.insert(std::move(make_pair(new_state, handle)));
                if (fScore <= best_fScore * this->l_weight)
                    focalSet.push(handle);
            } else {
                auto handle = iter->second;
                if (tentative_gScore >= (*handle)->gScore)  continue;
                int last_fScore = (*handle)->fScore;

                int delta = (*handle)->gScore - tentative_gScore;
                (*handle)->gScore = tentative_gScore;
                (*handle)->fScore -= delta;
                (*handle)->parent = current;
                openSet.increase(handle);
                if ((double) (*handle)->fScore <= (double) best_fScore * this->l_weight &&
                    (double)last_fScore > (double) best_fScore * this->l_weight) {
                    focalSet.push(handle);
                }
            }
        }
    }
    fmin[agent_idx] = INF7f;
    return nullptr;
}

bool ECBSTA::update_ECBSTANode_solution(shared_ptr<ECBSTANode>& curnode, int agent_id) {
    int goal = curnode->out_TA_solution[agent_id];
    if (!this->assignment_allow_map[agent_id][goal] || this->cost_matrix[agent_id][goal] == nullptr) return false;

    curnode->cost -= curnode->out_solution[agent_id]->back().gScore;
    curnode->LB -= curnode->fmin_matrix[agent_id];

    this->lowlevel_search_timer.reset();
    curnode->out_solution[agent_id] = this->findPath_a_star_eps(curnode->constraint_sets[agent_id], agent_id, goal, curnode->out_solution,curnode->fmin_matrix);
    this->lowlevel_search_timer.stop();
    this->lowlevel_search_time += this->lowlevel_search_timer.elapsedSeconds();

    curnode->LB += curnode->fmin_matrix[agent_id];
    if (curnode->out_solution[agent_id] == nullptr) return false;
    curnode->cost += curnode->out_solution[agent_id]->back().gScore;
    return true;
}


ECBSTA::ECBSTA(int row_number, int col_number, unordered_set<Location>& obstacles,
               vector<unordered_set<Location> >& goals, vector<State>& start_states,
               unordered_map<Location, int>& goal_to_idx, unordered_map<int, Location>& idx_to_goal, float l_weight) {
    this->row_number = row_number;
    this->col_number = col_number;
    this->start_states = start_states;
    this->obstacles = obstacles;
    this->goals = goals;
    this->goal_to_idx = goal_to_idx;
    this->idx_to_goal = idx_to_goal;
    this->l_weight = l_weight;

    this->ta_runtime = 0;
    this->total_runtime = 0;
    this->lowlevel_search_time = 0;
    this->newnode_time = 0;
    this->firstconflict_time = 0;
    this->focal_score_time = 0;
    this->invoke_shortest_path_times = 0;

    this->cbsnode_num = 0;
    this->lowLevelExpanded = 0;
    this->num_ta = 0;

    this->ta_open.clear();

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

int ECBSTA::heuristic(int x1, int y1, Location goal_loc) {
    int idx = this->goal_to_idx[goal_loc];
    return this->prior_hmap[idx][x1][y1];
}

void ECBSTA::create_global_cost_matrix() {
    this->cost_matrix.resize(this->agent_n, vector<shared_ptr<Path> >(this->diff_goal_n, shared_ptr<Path >(nullptr)));
    for (int i=0; i< this->agent_n;i++)
    {
        auto tmp_contraint = shared_ptr<Constraints>(new Constraints);
        for (int j=0;j< this->diff_goal_n; j++)
        {
            if (!this->assignment_allow_map[i][j]) continue;
            this->lowlevel_search_timer.reset();
            this->cost_matrix[i][j] = this->findPath(tmp_contraint, i, j);
            this->lowlevel_search_timer.stop();
            this->lowlevel_search_time += this->lowlevel_search_timer.elapsedSeconds();
        }
    }
}


bool ECBSTA::searchNodeIsValid(shared_ptr<Constraints>&  agent_constraint_set, const State& new_state, const State& org_state) {
//    cout<<(state.x) <<" "<< (this->row_number) <<" "<< (state.y) <<" "<< (this->col_number) <<endl;

    if (new_state.x < 0 || new_state.x >= this->row_number || new_state.y < 0 || new_state.y >= this->col_number) return false;
    if (this->map2d_obstacle[new_state.x][new_state.y]) return false;
    if (closedSet.find(new_state) != closedSet.end()) return false;
    if (new_state.time >= MAX_TIMESTEP) return false;
    if (!agent_constraint_set->stateValid(new_state)) { return false;}
    if (!agent_constraint_set->transitionValid(org_state, new_state)) return false;
    return true;
}

shared_ptr<Path> ECBSTA::findPath(shared_ptr<Constraints>& agent_constraint_set, int agent_idx, int goal_loc_idx)
{
    this->invoke_shortest_path_times++;
    int search_max = 1e5;
    int m_lastGoalConstraint = -1;
    closedSet.clear();
    unordered_map<State, PathEntryHandle , boost::hash<State> > stateToHeap;
    low_openSet_t openSet;


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



bool ECBSTA::init_TA(shared_ptr<TANode>& out_TA_node)
{
    DynamicHungarianAssignment new_assignment;
    new_assignment.initHungarian(this->agent_n, this->diff_goal_n);
    shared_ptr<TANode> new_TA_node(new TANode(this->agent_n));
    new_TA_node->update_cost_matrix(this->cost_matrix);

    this->ta_timer.reset();
    int ans = new_assignment.firstSolution(new_TA_node->cost_matrix, new_TA_node->out_TA_solution);
    this->ta_timer.stop();
    this->ta_runtime += this->ta_timer.elapsedSeconds();

    if (ans > BAD_TA_ANS) return false;
    new_TA_node->cost = ans;
    out_TA_node = new_TA_node;
    ta_open.push(new_TA_node);
    return true;
}

bool ECBSTA::get_next_TA(shared_ptr<TANode>& out_TA_node)
{
    if (ta_open.empty()) return false;
    shared_ptr<TANode> cur_TA_node = ta_open.top(); ta_open.pop();
    for (int i=0;i<this->agent_n;i++)
        if (cur_TA_node->in_set.find(i) == cur_TA_node->in_set.end()) {
            shared_ptr<TANode> new_TA_node(new TANode(cur_TA_node));
            new_TA_node->out_set[i].push_back(cur_TA_node->out_TA_solution[i]);
            for (int j=0;j<i;j++)
                new_TA_node->in_set[j] = cur_TA_node->out_TA_solution[j];
            new_TA_node->update_cost_matrix(this->cost_matrix);
            DynamicHungarianAssignment new_assignment;
            this->ta_timer.reset();
            new_assignment.initHungarian(this->agent_n, this->diff_goal_n);
            int ans = new_assignment.firstSolution(new_TA_node->cost_matrix, new_TA_node->out_TA_solution);
            this->ta_timer.stop();
            this->ta_runtime += this->ta_timer.elapsedSeconds();
            if (ans > BAD_TA_ANS) continue;
            new_TA_node->cost = ans;
            ta_open.push(new_TA_node);
        }
    out_TA_node = cur_TA_node;
    return true;
}

int ECBSTA::high_focal_score(const vector<shared_ptr<Path > >& cost_matrix, Conflict& result)
{
    int numConflicts = 0;

    int max_t = 0;
    for (const auto &sol: cost_matrix) {
        max_t = std::max<int>(max_t, sol->size());
    }

    for (int t = 0; t < max_t; ++t) {
        // check drive-drive vertex collisions
        for (size_t i = 0; i < cost_matrix.size(); ++i) {
            State state1 = getState(cost_matrix[i], t);
            for (size_t j = i + 1; j < cost_matrix.size(); ++j) {
                State state2 = getState(cost_matrix[j], t);
                if (state1.equalExceptTime(state2)) {
                    if (numConflicts == 0)
                    {
                        result.time = t;
                        result.agent1 = i;
                        result.agent2 = j;
                        result.type = Conflict::Vertex;
                        result.x1 = state1.x;
                        result.y1 = state1.y;
                    }
                    ++numConflicts;
                }
            }
        }
        // drive-drive edge (swap)
        for (size_t i = 0; i < cost_matrix.size(); ++i) {
            State state1a = getState(cost_matrix[i], t);
            State state1b = getState(cost_matrix[i], t + 1);
            for (size_t j = i + 1; j < cost_matrix.size(); ++j) {
                State state2a = getState(cost_matrix[j], t);
                State state2b = getState(cost_matrix[j], t + 1);
                if (state1a.equalExceptTime(state2b) &&
                    state1b.equalExceptTime(state2a)) {
                    if (numConflicts == 0)
                    {
                        result.time = t;
                        result.agent1 = i;
                        result.agent2 = j;
                        result.type = Conflict::Edge;
                        result.x1 = state1a.x;
                        result.y1 = state1a.y;
                        result.x2 = state1b.x;
                        result.y2 = state1b.y;
                    }
                    ++numConflicts;
                }
            }
        }
    }
    return numConflicts;
}



int ECBSTA::solve() {
    this->create_global_cost_matrix();

    this->newnode_timer.reset();
    shared_ptr<ECBSTANode> start_node(new ECBSTANode(this->agent_n));
    this->newnode_timer.stop();
    this->newnode_time += this->newnode_timer.elapsedSeconds();
    this->cbsnode_num++;
    start_node->is_root = true;
    shared_ptr<TANode> tmp_TA_node = nullptr;
    {
        this->num_ta ++;
        this->ta_timer.reset();
        bool b = this->init_TA(tmp_TA_node);
        this->ta_timer.stop();
        this->ta_runtime += this->ta_timer.elapsedSeconds();
        if (!b) return false;
    }
    start_node->out_TA_solution = tmp_TA_node->out_TA_solution;
    start_node->get_path_from_TA(this->cost_matrix);

    this->conflict_num_timer.reset();
    vector<bool> changed_agent(this->agent_n, 1);
//    start_node->focal_score = this->high_focal_score(start_node->out_solution, start_node->first_conflict);
    start_node->focal_score = high_focal_score_v4(start_node->out_solution, start_node->first_conflict, changed_agent, start_node->conflict_matrix);
    this->conflict_num_timer.stop();
    this->conflict_num_time += this->conflict_num_timer.elapsedSeconds();



    high_openSet_t open;
    high_focalSet_t focal;
    high_LBSet_t LBset;
    unordered_map<shared_ptr<ECBSTANode>, ECBSTANodeHandle, boost::hash<shared_ptr<ECBSTANode> > > stateToHeap;
    unordered_map<shared_ptr<ECBSTANode>, ECBSTANodeLBHandle, boost::hash<shared_ptr<ECBSTANode> > > CTnode2LB_handle;

    auto handle = open.push(start_node);
    auto handle2 = LBset.push(start_node);
    stateToHeap.insert(make_pair(start_node, handle));
    CTnode2LB_handle.insert(make_pair(start_node, handle2));
    focal.push(handle);
    int best_LB = start_node->LB;

    while (!open.empty())
    {

        int old_best_LB = best_LB;
        best_LB = LBset.top()->LB;
        if (best_LB > old_best_LB)
        {
            for (auto iter = open.ordered_begin(); iter!=open.ordered_end(); iter++)
            {
                int val = (*iter)->cost;
                if ((double)val > (double) old_best_LB * this->l_weight && (double) val <= (double) best_LB * l_weight)
                {
                    auto handle_iter = stateToHeap.find(*iter);
                    focal.push(handle_iter->second);
                }
                if ((double)val > (double) best_LB * l_weight) break;
            }
        }


        auto current_handler = focal.top();
        shared_ptr<ECBSTANode> cur_node = *current_handler;
        focal.pop();
        open.erase(current_handler);
        stateToHeap.erase(cur_node);

        auto LB_handler = CTnode2LB_handle[cur_node];
        LBset.erase(LB_handler);
        CTnode2LB_handle.erase(cur_node);
//        printf("cur_node->focal_score: %d\n", cur_node->focal_score);
        bool done = cur_node->focal_score == 0;
        if (done)
        {
            std::cout << "done; cost: " << cur_node->cost << std::endl;
            this->out_solution = cur_node->out_solution;
            if (!check_ans_valid(this->out_solution))
                std::cout << "INVALID ANS PATH!!!!" << std::endl;
            this->cost = cur_node->cost;
            this->solution_found = true;
            return true;
        }

        if (cur_node->is_root)
        {
            this->newnode_timer.reset();
            shared_ptr<ECBSTANode> new_node(new ECBSTANode(this->agent_n));
            this->newnode_timer.stop();
            this->newnode_time += this->newnode_timer.elapsedSeconds();

            new_node->is_root = true;
            tmp_TA_node = nullptr;

            this->ta_timer.reset();
            this->num_ta ++;
            bool b = this->get_next_TA(tmp_TA_node);
            this->ta_timer.stop();
            this->ta_runtime += this->ta_timer.elapsedSeconds();
            if (!b) continue;
            new_node->out_TA_solution = tmp_TA_node->out_TA_solution;
            new_node->get_path_from_TA(this->cost_matrix);

//            new_node->focal_score = this->high_focal_score(new_node->out_solution, new_node->first_conflict);
//            new_node->focal_score = high_focal_score_v2(new_node->out_solution, new_node->first_conflict);
            changed_agent = vector<bool>(this->agent_n, 1);
            new_node->focal_score = high_focal_score_v4(new_node->out_solution, new_node->first_conflict, changed_agent, new_node->conflict_matrix);

            auto handle = open.push(new_node);
            auto handle2 = LBset.push(new_node);
            stateToHeap.insert(make_pair(new_node, handle));
            CTnode2LB_handle.insert(make_pair(new_node, handle2));
            if ((double) new_node->cost <= (double) best_LB * this->l_weight) {
                focal.push(handle);
            }
        }

        unordered_map<size_t, Constraints> tmp;
        createConstraintsFromConflict(cur_node->first_conflict, tmp);

        for (unsigned short cur_i = 0; auto &[key, value]: tmp)
        {
            shared_ptr<ECBSTANode> new_node;
            this->newnode_timer.reset();
            if (cur_i == 1) new_node = cur_node;
                else new_node = shared_ptr<ECBSTANode>(new ECBSTANode(cur_node));
            this->newnode_timer.stop();
            this->newnode_time += this->newnode_timer.elapsedSeconds();

            assert(!new_node->constraint_sets[key]->overlap(value));
            new_node->constraint_sets[key] = shared_ptr<Constraints>(new Constraints(*(new_node->constraint_sets[key])));
            new_node->constraint_sets[key]->add(value);
            cur_i ++;
            new_node->is_root = false;
            bool b = this->update_ECBSTANode_solution(new_node, key);
            if (!b) continue;

            this->cbsnode_num ++;
            this->conflict_num_timer.reset();
            changed_agent = vector<bool>(this->agent_n, 0);
            changed_agent[key] = 1;
//            new_node->focal_score = this->high_focal_score(new_node->out_solution, new_node->first_conflict);
            new_node->focal_score = high_focal_score_v4(new_node->out_solution, new_node->first_conflict, changed_agent, new_node->conflict_matrix);
            this->conflict_num_timer.stop();
            this->conflict_num_time += this->conflict_num_timer.elapsedSeconds();

            auto handle = open.push(new_node);
            auto handle2 = LBset.push(new_node);
            stateToHeap.insert(make_pair(new_node, handle));
            CTnode2LB_handle.insert(make_pair(new_node, handle2));
            if ((double) new_node->cost <= (double) best_LB * this->l_weight) {
                focal.push(handle);
            }

        }
    }
    return false;
}
