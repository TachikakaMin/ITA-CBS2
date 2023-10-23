#include "TANode.hpp"


TANode::TANode(shared_ptr<TANode> curnode){
    this->out_set = curnode->out_set;
    this->in_set = curnode->in_set;
    this->cost = curnode->cost;
    this->out_TA_solution = curnode->out_TA_solution;
    this->cost_matrix = curnode->cost_matrix;
}


void TANode::update_cost_matrix(vector<vector<shared_ptr<Path > > >& global_cost_matrix)
{
    int n = global_cost_matrix.size();
    int m = global_cost_matrix[0].size();
    this->cost_matrix.resize(n, vector<shared_ptr<Path> >(m, nullptr) );
    for (int i=0;i<n;i++)
    {
        if (this->in_set.find(i) != this->in_set.end())
        {
            int goal = this->in_set[i];
            this->cost_matrix[i][goal] = global_cost_matrix[i][goal];
        }
        else {
            for (int j=0;j<m;j++) this->cost_matrix[i][j] = global_cost_matrix[i][j];
        }
    }

    for (int i=0;i<n;i++)
    {
        vector<int>& out_set_list = this->out_set[i];
        for (int j=0;j<out_set_list.size(); j++)
        {
            int goal = out_set_list[j];
            this->cost_matrix[i][goal] = nullptr;
        }
    }
    return ;
}

TANode::TANode(int n){
    this->cost = 0;
    this->out_set.resize(n, vector<int>());
}