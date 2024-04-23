#include "ECBSTANode.hpp"

ECBSTANode::ECBSTANode(shared_ptr<ECBSTANode> curnode) {
    out_TA_solution = curnode->out_TA_solution;
    out_solution = curnode->out_solution;
    cost = curnode->cost;
    constraint_sets = curnode->constraint_sets;
    is_root = curnode->is_root;
    fmin_matrix = curnode->fmin_matrix;
    conflict_matrix = curnode->conflict_matrix;
    LB = curnode->LB;
}

ECBSTANode::ECBSTANode(int n){
    is_root = false;
    constraint_sets.resize(n, shared_ptr<Constraints>(new Constraints));
    fmin_matrix.resize(n, 0);
    conflict_matrix.resize(n, vector<int>(n, 0));
    LB = 0;
    focal_score = 0;
}


void ECBSTANode::get_path_from_TA(vector<vector<shared_ptr<Path > > >& cost_matrix)
{
    int n = this->out_TA_solution.size();
    BOOST_ASSERT(n == cost_matrix.size());
    this->out_solution.resize(n);
    this->cost = 0;
    this->LB = 0;
    for (int i=0;i<n;i++) {
        this->out_solution[i] = cost_matrix[i][this->out_TA_solution[i]];
        this->fmin_matrix[i] = this->out_solution[i]->back().gScore;
        this->cost += this->out_solution[i]->back().gScore;
    }
    this->LB = this->cost;
    return ;
}


bool ECBSTANode::get_first_conflict(Conflict &result) {
    int agent_n = this->out_solution.size();
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


