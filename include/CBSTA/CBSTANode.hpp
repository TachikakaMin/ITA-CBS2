//
// Created by YIMIN TANG on 3/20/23.
//

#ifndef ITACBS_REMAKE_CBSTANode_HPP
#define ITACBS_REMAKE_CBSTANode_HPP

#include "../common.hpp"
#include "../dynamic_hungarian_assignment.hpp"

class CBSTANode{


public:

    CBSTANode(int n);
    explicit CBSTANode(shared_ptr<CBSTANode> curnode);
    bool get_first_conflict(Conflict& conflict);
    void get_path_from_TA(vector<vector<shared_ptr<Path > > >& cost_matrix);

    int cost;
    bool is_root;
    unordered_map<int, int> out_TA_solution;
    vector<shared_ptr<Path > > out_solution;
    vector<shared_ptr<Constraints >> constraint_sets;
};

struct CBSTANodePtrCmp{
    bool operator()(const shared_ptr<CBSTANode> a, const shared_ptr<CBSTANode> b) const {
        return a->cost > b->cost;
    }
};



#endif //ITACBS_REMAKE_CBSTANode_HPP
