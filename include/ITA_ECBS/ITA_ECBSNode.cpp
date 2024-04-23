
#include "ITA_ECBSNode.hpp"
#include "ITA_ECBS.hpp"


ITA_ECBSNode::ITA_ECBSNode(shared_ptr<ITA_ECBSNode> curnode) {
    out_TA_solution = curnode->out_TA_solution;
    out_solution = curnode->out_solution;
    cost = curnode->cost;
    constraint_sets = curnode->constraint_sets;
    new_assignment = curnode->new_assignment;
    cost_matrix = curnode->cost_matrix;
    LB = curnode->LB;
    fmin_matrix = curnode->fmin_matrix;
    focal_score = curnode->focal_score;
    conflict_matrix = curnode->conflict_matrix;
}

ITA_ECBSNode::ITA_ECBSNode(){
    LB = 0;
    focal_score = 0;
}

void ITA_ECBSNode::create_cost_matrix(ITA_ECBS* pInstance) {
    constraint_sets.resize(pInstance->agent_n, shared_ptr<Constraints>(nullptr));
    cost_matrix.resize(pInstance->agent_n, vector<shared_ptr<Path> >(pInstance->diff_goal_n, shared_ptr<Path >(nullptr)));
    fmin_matrix.resize(pInstance->agent_n, vector<shared_ptr<Path> >(pInstance->diff_goal_n, shared_ptr<Path >(nullptr)));
    conflict_matrix.resize(pInstance->agent_n, vector<int>(pInstance->agent_n, 0));
    for (int i=0; i< pInstance->agent_n;i++)
    {
        constraint_sets[i] = shared_ptr<Constraints>(new Constraints);
        for (int j=0;j< pInstance->diff_goal_n; j++)
        {
            if (!pInstance->assignment_allow_map[i][j]) continue;
            pInstance->lowlevel_search_timer.reset();
            cost_matrix[i][j] = pInstance->findPath_a_star(this->constraint_sets[i], i, j);
            fmin_matrix[i][j] = cost_matrix[i][j];
            pInstance->lowlevel_search_timer.stop();
            pInstance->lowlevel_search_time += pInstance->lowlevel_search_timer.elapsedSeconds();
        }
    }
}

bool ITA_ECBSNode::get_first_assignment(ITA_ECBS* pInstance) {
    this->new_assignment.initHungarian(pInstance->agent_n, pInstance->diff_goal_n);
    pInstance->num_ta ++;
    pInstance->ta_timer.reset();
    cost = this->new_assignment.firstSolution(this->cost_matrix, this->out_TA_solution);
    pInstance->ta_timer.stop();
    pInstance->ta_runtime += pInstance->ta_timer.elapsedSeconds();
    if (cost > BAD_TA_ANS) return 0;
    LB = this->get_LB();

    this->out_solution.clear();
    this->out_solution.resize(pInstance->agent_n, shared_ptr<Path>(nullptr));
    for (const auto& [key, value] : out_TA_solution) {
        this->out_solution[key] = this->cost_matrix[key][value];
    }
    return cost;
}


bool ITA_ECBSNode::get_first_conflict(Conflict &result) {
    int agent_n = this->cost_matrix.size();

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

bool ITA_ECBSNode::update_fmin_matrix(ITA_ECBS* pInstance, int agent_id, Constraints& new_constraint) {
    for (int j=0;j< pInstance->diff_goal_n; j++)
    {
        if (!pInstance->assignment_allow_map[agent_id][j] || this->fmin_matrix[agent_id][j] == nullptr) continue;

        bool need_replan = false;
        int cnt = 0;
        PathEntry last_path_node;
        for (auto& path_node: *(this->fmin_matrix[agent_id][j]))
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
            this->fmin_matrix[agent_id][j] = pInstance->findPath_a_star(this->constraint_sets[agent_id], agent_id, j);
            pInstance->lowlevel_search_timer.stop();
            pInstance->lowlevel_search_time += pInstance->lowlevel_search_timer.elapsedSeconds();
        }
    }
    return 1;
}

bool ITA_ECBSNode::update_cost_matrix(ITA_ECBS* pInstance, Constraints& new_constraint, vector<bool>& changed_agent) {
    vector<shared_ptr<Path > > old_solution = this->out_solution;
    this->out_solution.clear();
    this->out_solution.resize(this->out_TA_solution.size(), shared_ptr<Path>(nullptr));
    for (const auto& [key, value] : out_TA_solution) {
        this->out_solution[key] = this->cost_matrix[key][value];
    }


    for (int i=0;i<pInstance->agent_n;i++)
    {
        bool need_replan = false;
        int cnt = 0;
        PathEntry last_path_node;
        for (auto& path_node: *(this->out_solution[i]))
        {
            if (!this->constraint_sets[i]->stateValid(path_node.state)) {need_replan = true; break;}
            if (cnt > 0)
                if (!this->constraint_sets[i]->transitionValid(last_path_node.state, path_node.state))
                    {need_replan = true; break;}
            cnt++;
            last_path_node = path_node;
        }
        for (const auto &vc: this->constraint_sets[i]->vertexConstraints)
            if (last_path_node.state.x == vc.x && last_path_node.state.y == vc.y && last_path_node.state.time <= vc.time) need_replan = true;

        if (need_replan) {
            pInstance->lowlevel_search_timer.reset();
            int goal_idx = this->out_TA_solution[i];
            int fmin = this->fmin_matrix[i][goal_idx]->back().gScore;
            this->cost_matrix[i][goal_idx] = pInstance->findPath_a_star_eps(this->constraint_sets[i], i,
                                                                            goal_idx, this->out_solution, fmin);
            this->out_solution[i] = this->cost_matrix[i][goal_idx];
            if (this->cost_matrix[i][goal_idx] == nullptr)
                this->out_solution[i] = this->fmin_matrix[i][goal_idx];
            pInstance->lowlevel_search_timer.stop();
            pInstance->lowlevel_search_time += pInstance->lowlevel_search_timer.elapsedSeconds();
        }
    }
    for (int i=0;i< this->out_solution.size(); i++)
    {
        changed_agent[i] = 0;
        if (this->out_solution[i] != old_solution[i]) changed_agent[i] = 1;
    }
    cost = this->get_cost();
    return 1;
}

int ITA_ECBSNode::get_LB()
{
    int fmin_val = 0;
    int n = out_solution.size();
    for (int i=0;i<n;i++)
    {
        int goal_idx = this->out_TA_solution[i];
        if (fmin_matrix[i][goal_idx] != nullptr)
            fmin_val += fmin_matrix[i][goal_idx]->back().gScore;
        else fmin_val += INF7f;
    }
    return fmin_val;
}

int ITA_ECBSNode::get_cost()
{
    int cost = 0;
    int n = out_TA_solution.size();
    for (int i=0;i<n;i++)
    {
        int goal_idx = this->out_TA_solution[i];
        if (cost_matrix[i][goal_idx] == nullptr)
            cost += INF7f;
        else cost += cost_matrix[i][goal_idx]->back().gScore;
    }
    return cost;
}

bool ITA_ECBSNode::get_next_assignment(int agent_id, double w) {
    LB = this->new_assignment.incrementalSolutionX(this->fmin_matrix, this->out_TA_solution, agent_id);
    if (LB > BAD_TA_ANS) return 0;
    return 1;
}


