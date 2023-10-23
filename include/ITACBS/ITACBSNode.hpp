//
// Created by YIMIN TANG on 3/20/23.
//

#ifndef ITACBS_REMAKE_ITACBSNODE_HPP
#define ITACBS_REMAKE_ITACBSNODE_HPP

#include "ITACBS.hpp"
#include "../dynamic_hungarian_assignment.hpp"

class ITACBSNode{


public:

    ITACBSNode(const int&  id);
    explicit ITACBSNode(shared_ptr<ITACBSNode> curnode, const int& id);

    void create_cost_matrix(ITACBS* pInstance);
    bool get_first_assignment(ITACBS* pInstance);
    bool get_first_conflict(Conflict& conflict);
    bool update_cost_matrix(ITACBS* pInstance, int agent_id, Constraints& new_constraint);
    bool get_next_assignment(int agent_id);

    int cost;
    unordered_map<int, int> out_TA_solution;
    vector<shared_ptr<Path > > out_solution;
    vector<vector<shared_ptr<Path > > > cost_matrix;
    vector<shared_ptr<Constraints >> constraint_sets;
    DynamicHungarianAssignment new_assignment;

    int idx;

};

struct ITA_CBSNodePtrCmp{
    bool operator()(const shared_ptr<ITACBSNode>& a, const shared_ptr<ITACBSNode>& b) const {
        if (a->cost == b->cost) return a->idx < b->idx;
        return a->cost > b->cost;
    }
};



#endif //ITACBS_REMAKE_ITACBSNODE_HPP
