

#ifndef ITACBS_REMAKE_ECBSTANode_HPP
#define ITACBS_REMAKE_ECBSTANode_HPP

#include "../common.hpp"
#include "../dynamic_hungarian_assignment.hpp"

class ECBSTANode{


public:

    ECBSTANode(int n);
    explicit ECBSTANode(shared_ptr<ECBSTANode> curnode);
    bool get_first_conflict(Conflict& conflict);
    void get_path_from_TA(vector<vector<shared_ptr<Path > > >& cost_matrix);

    int cost, LB, focal_score;
    bool is_root;
    Conflict first_conflict;
    unordered_map<int, int> out_TA_solution;
    vector<int > fmin_matrix;
    vector<vector<int>> conflict_matrix;
    vector<shared_ptr<Path > > out_solution;
    vector<shared_ptr<Constraints >> constraint_sets;
};

struct ECBSTANodePtrCmp{
    bool operator()(const shared_ptr<ECBSTANode>& a, const shared_ptr<ECBSTANode>& b) const {
        return a->cost > b->cost;
    }
};


typedef typename boost::heap::d_ary_heap<shared_ptr<ECBSTANode>,
        boost::heap::mutable_<true>,
        boost::heap::arity<2>,
        boost::heap::compare<ECBSTANodePtrCmp> >
        high_openSet_t;
using ECBSTANodeHandle = high_openSet_t::handle_type;

struct ECBSTANodePtrLBCmp{
    bool operator()(const shared_ptr<ECBSTANode>& a, const shared_ptr<ECBSTANode>& b) const {
        return a->LB > b->LB;
    }
};

typedef typename boost::heap::d_ary_heap<shared_ptr<ECBSTANode>,
        boost::heap::mutable_<true>,
        boost::heap::arity<2>,
        boost::heap::compare<ECBSTANodePtrLBCmp> >
        high_LBSet_t;
using ECBSTANodeLBHandle = high_LBSet_t::handle_type;

struct compareFocalHeuristic {
    bool operator()(const ECBSTANodeHandle& h1, const ECBSTANodeHandle& h2) const {
        // Our heap is a maximum heap, so we invert the comperator function here
        if ((*h1)->focal_score != (*h2)->focal_score) {
            return (*h1)->focal_score > (*h2)->focal_score;
        }
        return (*h1)->cost > (*h2)->cost;
    }
};

typedef typename boost::heap::d_ary_heap<
        ECBSTANodeHandle, boost::heap::arity<2>, boost::heap::mutable_<true>,
        boost::heap::compare<compareFocalHeuristic> >
        high_focalSet_t;



#endif //ITACBS_REMAKE_ECBSTANode_HPP
