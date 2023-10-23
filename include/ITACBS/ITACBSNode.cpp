//
// Created by YIMIN TANG on 3/20/23.
//
#include "ITACBSNode.hpp"
#include "ITACBS.hpp"


ITACBSNode::ITACBSNode(shared_ptr<ITACBSNode> curnode, const int& id) {
    idx = id;
    out_TA_solution = curnode->out_TA_solution;
    out_solution = curnode->out_solution;
    cost = curnode->cost;
    constraint_sets = curnode->constraint_sets;
    new_assignment = curnode->new_assignment;
    cost_matrix = curnode->cost_matrix;
}

ITACBSNode::ITACBSNode(const int&  id){
    idx = id;
}

void ITACBSNode::create_cost_matrix(ITACBS* pInstance) {
    constraint_sets.resize(pInstance->agent_n, shared_ptr<Constraints>(nullptr));
    cost_matrix.resize(pInstance->agent_n, vector<shared_ptr<Path> >(pInstance->diff_goal_n, shared_ptr<Path >(nullptr)));
    for (int i=0; i< pInstance->agent_n;i++)
    {
        constraint_sets[i] = shared_ptr<Constraints>(new Constraints);
        for (int j=0;j< pInstance->diff_goal_n; j++)
        {
            if (!pInstance->assignment_allow_map[i][j]) continue;
            pInstance->lowlevel_search_timer.reset();
            cost_matrix[i][j] = pInstance->findPath_a_star(this->constraint_sets[i], i, j);
            pInstance->lowlevel_search_timer.stop();
            pInstance->lowlevel_search_time += pInstance->lowlevel_search_timer.elapsedSeconds();
        }
    }
}

bool ITACBSNode::get_first_assignment(ITACBS* pInstance) {
    this->new_assignment.initHungarian(pInstance->agent_n, pInstance->diff_goal_n);
    pInstance->num_ta ++;
    pInstance->ta_timer.reset();
    this->cost = this->new_assignment.firstSolution(this->cost_matrix, this->out_TA_solution);
    pInstance->ta_timer.stop();
    pInstance->ta_runtime += pInstance->ta_timer.elapsedSeconds();
    if (this->cost > BAD_TA_ANS) return 0;
    return this->cost;
}


bool ITACBSNode::get_first_conflict(Conflict &result) {
    int agent_n = this->cost_matrix.size();
    this->out_solution.clear();
    this->out_solution.resize(agent_n, shared_ptr<Path>(nullptr));
    for (const auto& [key, value] : out_TA_solution) {
        this->out_solution[key] = this->cost_matrix[key][value];
    }
    int max_t = 0;
    for (const auto &sol: this->out_solution) {
        max_t = std::max<int>(max_t, sol->size());
    }

    for (int t = 0; t < max_t; ++t) {
        // check drive-drive vertex collisions
        for (size_t i = 0; i < agent_n; ++i) {
            State state1 = getState(this->out_solution[i], t);
            for (size_t j = i + 1; j < agent_n; ++j) {
                State state2 = getState(this->out_solution[j], t);
                if (state1.equalExceptTime(state2)) {
                    result.time = t;
                    result.agent1 = i;
                    result.agent2 = j;
                    result.type = Conflict::Vertex;
                    result.x1 = state1.x;
                    result.y1 = state1.y;
                    // std::cout << "VC " << t << "," << state1.x << "," << state1.y <<
                    // std::endl;
                    return true;
                }
            }
        }
        // drive-drive edge (swap)
        for (size_t i = 0; i < agent_n; ++i) {
            State state1a = getState(this->out_solution[i], t);
            State state1b = getState(this->out_solution[i], t + 1);
            for (size_t j = i + 1; j < agent_n; ++j) {
                State state2a = getState(this->out_solution[j], t);
                State state2b = getState(this->out_solution[j], t + 1);
                if (state1a.equalExceptTime(state2b) &&
                    state1b.equalExceptTime(state2a)) {
                    result.time = t;
                    result.agent1 = i;
                    result.agent2 = j;
                    result.type = Conflict::Edge;
                    result.x1 = state1a.x;
                    result.y1 = state1a.y;
                    result.x2 = state1b.x;
                    result.y2 = state1b.y;
                    return true;
                }
            }
        }
    }

    return false;
}

bool ITACBSNode::update_cost_matrix(ITACBS* pInstance, int agent_id, Constraints& new_constraint) {
    bool b = 0;
    int x = this->out_TA_solution[agent_id];
    for (int j=0;j< pInstance->diff_goal_n; j++)
    {
        if (!pInstance->assignment_allow_map[agent_id][j] || this->cost_matrix[agent_id][j] == nullptr) continue;
        int pre_cost = this->cost_matrix[agent_id][j]->back().gScore;

        bool need_replan = false;
        int cnt = 0;
        PathEntry last_path_node;
        for (auto& path_node: *(this->cost_matrix[agent_id][j]))
        {
            if (!new_constraint.stateValid(path_node.state)) {need_replan = true; break;}
            if (cnt > 0)
                if (!new_constraint.transitionValid(last_path_node.state, path_node.state))
                    {need_replan = true; break;}
            cnt++;
            last_path_node = path_node;
        }
        for (const auto &vc: new_constraint.vertexConstraints)
            if (last_path_node.state.x == vc.x && last_path_node.state.y == vc.y && last_path_node.state.time <= vc.time) need_replan = true;

        if (need_replan) {
            pInstance->lowlevel_search_timer.reset();
            this->cost_matrix[agent_id][j] = pInstance->findPath_a_star(this->constraint_sets[agent_id], agent_id, j);
            pInstance->lowlevel_search_timer.stop();
            pInstance->lowlevel_search_time += pInstance->lowlevel_search_timer.elapsedSeconds();
            int post_cost;
            if (this->cost_matrix[agent_id][j] == nullptr) post_cost = -1;
                else post_cost = this->cost_matrix[agent_id][j]->back().gScore;
            b = max(b, pre_cost != post_cost);
        }
    }
    return b;
}

bool ITACBSNode::get_next_assignment(int agent_id) {
    unordered_map<int, int> preTAsol = this->out_TA_solution;
    this->cost = this->new_assignment.incrementalSolutionX(this->cost_matrix, this->out_TA_solution, agent_id);
    int n = this->cost_matrix.size();
    bool ta_changed = 0;
    for (int i=0;i<n;i++)
        if (preTAsol[i] != this->out_TA_solution[i]) {ta_changed = 1; break;}
    return ta_changed;
}


