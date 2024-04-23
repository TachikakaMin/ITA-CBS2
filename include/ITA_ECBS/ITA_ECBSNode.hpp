

#ifndef ITA_ECBS_REMAKE_ITA_ECBSNODE_HPP
#define ITA_ECBS_REMAKE_ITA_ECBSNODE_HPP

#include "ITA_ECBS.hpp"
#include "../dynamic_hungarian_assignment.hpp"



class ITA_ECBSNode{


public:

    ITA_ECBSNode();
    explicit ITA_ECBSNode(shared_ptr<ITA_ECBSNode> curnode);

    void create_cost_matrix(ITA_ECBS* pInstance);
    bool get_first_assignment(ITA_ECBS* pInstance);
    bool get_first_conflict(Conflict& conflict);
    bool update_fmin_matrix(ITA_ECBS* pInstance, int agent_id, Constraints& new_constraint);
    bool update_cost_matrix(ITA_ECBS* pInstance, Constraints& new_constraint, vector<bool>& changed_agent);
    bool get_next_assignment(int agent_id, double w);
    int get_LB();
    int get_cost();

    int cost, LB, focal_score;
    Conflict first_conflict;
    unordered_map<int, int> out_TA_solution;
    vector<shared_ptr<Path > > out_solution;
    vector<vector<shared_ptr<Path > > > cost_matrix;
    vector<vector<shared_ptr<Path > > > fmin_matrix;
    vector<vector<int > > conflict_matrix;
    vector<shared_ptr<Constraints >> constraint_sets;
    DynamicHungarianAssignment new_assignment;
};

struct ITA_ECBSNodePtrCmp{
    bool operator()(const shared_ptr<ITA_ECBSNode>& a, const shared_ptr<ITA_ECBSNode>& b) const {
        return a->cost > b->cost;
    }
};


typedef typename boost::heap::d_ary_heap<shared_ptr<ITA_ECBSNode>,
        boost::heap::mutable_<true>,
        boost::heap::arity<2>,
        boost::heap::compare<ITA_ECBSNodePtrCmp> >
        high_openSet_t;
using ITA_ECBSNodeHandle = high_openSet_t::handle_type;

struct ITA_ECBSNodePtrLBCmp{
    bool operator()(const shared_ptr<ITA_ECBSNode>& a, const shared_ptr<ITA_ECBSNode>& b) const {
        return a->LB > b->LB;
    }
};

typedef typename boost::heap::d_ary_heap<shared_ptr<ITA_ECBSNode>,
        boost::heap::mutable_<true>,
        boost::heap::arity<2>,
        boost::heap::compare<ITA_ECBSNodePtrLBCmp> >
        high_LBSet_t;
using ITA_ECBSNodeLBHandle = high_LBSet_t::handle_type;

struct compareFocalHeuristic {
    bool operator()(const ITA_ECBSNodeHandle& h1, const ITA_ECBSNodeHandle& h2) const {
        // Our heap is a maximum heap, so we invert the comperator function here
        if ((*h1)->focal_score != (*h2)->focal_score) {
            return (*h1)->focal_score > (*h2)->focal_score;
        }
        return (*h1)->cost > (*h2)->cost;
    }
};

typedef typename boost::heap::d_ary_heap<
        ITA_ECBSNodeHandle, boost::heap::arity<2>, boost::heap::mutable_<true>,
        boost::heap::compare<compareFocalHeuristic> >
        high_focalSet_t;




#endif //ITA_ECBS_REMAKE_ITA_ECBSNODE_HPP
