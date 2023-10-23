//
// Created by YIMIN TANG on 5/24/23.
//
#include "common.hpp"

void createConstraintsFromConflict(
        const Conflict &conflict, unordered_map<size_t, Constraints> &constraints) {
    if (conflict.type == Conflict::Vertex) {
        Constraints c1;
        c1.vertexConstraints.emplace(
                VertexConstraint(conflict.time, conflict.x1, conflict.y1, conflict.agent2));
        constraints[conflict.agent1] = c1;
        Constraints c2;
        c2.vertexConstraints.emplace(
                VertexConstraint(conflict.time, conflict.x1, conflict.y1, conflict.agent1));
        constraints[conflict.agent2] = c2;
    } else if (conflict.type == Conflict::Edge) {
        Constraints c1;
        c1.edgeConstraints.emplace(EdgeConstraint(
                conflict.time, conflict.x1, conflict.y1, conflict.x2, conflict.y2, conflict.agent2));
        constraints[conflict.agent1] = c1;
        Constraints c2;
        c2.edgeConstraints.emplace(EdgeConstraint(
                conflict.time, conflict.x2, conflict.y2, conflict.x1, conflict.y1, conflict.agent1));
        constraints[conflict.agent2] = c2;
    } else {
        printf("???? No such collision\n");
    }
}

State getState(shared_ptr<Path> sol, int t)
{
    if (t >= sol->size()) return sol->back().state;
    return (*sol)[t].state;
}

int high_focal_score_v2(const vector<shared_ptr<Path > >& out_solution, Conflict& result)
{
    int numConflicts = 0;
    int max_t = 0;
    for (const auto &sol: out_solution) {
        max_t = std::max<int>(max_t, sol->size());
    }

    for (size_t i = 0; i < out_solution.size(); ++i)
        for (size_t j = i + 1; j < out_solution.size(); ++j)
            for (int t = 0; t < max_t; ++t)
            {
                State state1a = getState(out_solution[i], t);
                State state2a = getState(out_solution[j], t);
                if (state1a.equalExceptTime(state2a)) {
                    if (numConflicts == 0 || result.time > t)
                    {
                        result.time = t;
                        result.agent1 = i;
                        result.agent2 = j;
                        result.type = Conflict::Vertex;
                        result.x1 = state1a.x;
                        result.y1 = state1a.y;
                    }
                    ++numConflicts;
                    break;
                }

                State state1b = getState(out_solution[i], t + 1);
                State state2b = getState(out_solution[j], t + 1);
                if (state1a.equalExceptTime(state2b) &&
                    state1b.equalExceptTime(state2a)) {
                    if (numConflicts == 0 || result.time > t) {
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
                    break;
                }
            }
    return numConflicts;
}

int read_map_file(std::filesystem::path map_file_path, vector<vector<bool> >& ret_map){
    std::ifstream inFile(map_file_path); // Replace with your file path
    if (!inFile) {
        std::cerr << "Unable to open file\n";
        return 1; // Return with error
    }

    std::string line;
    for (int i = 0; i < 4; ++i) { // Skip first 4 lines
        std::getline(inFile, line);
    }

    while (std::getline(inFile, line)) {
        vector<bool> row;
        for (char c : line) {
            if (c == '\n') break;
            if (c == '.') row.push_back(0);
                else row.push_back(1);
        }
        ret_map.push_back(row);
    }

    inFile.close();
    return 0;
}

