//
// Created by YIMIN TANG on 5/23/23.
//

#ifndef ITACBS_REMAKE_TANODE_HPP
#define ITACBS_REMAKE_TANODE_HPP

#include "../common.hpp"

class TANode {


public:

    TANode(int n);
    explicit TANode(shared_ptr<TANode> curnode);
    void update_cost_matrix(vector<vector<shared_ptr<Path > > >& global_cost_matrix);

    int cost;
    unordered_map<int, int> out_TA_solution;
    vector<vector<int> > out_set;
    unordered_map<int, int> in_set;
    vector<vector<shared_ptr<Path> > > cost_matrix;
};

struct TANodePtrCmp{
    bool operator()(const shared_ptr<TANode> a, const shared_ptr<TANode> b) const {
        return a->cost > b->cost;
    }
};
#endif //ITACBS_REMAKE_TANODE_HPP
