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
//                    break;
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
//                    break;
                }
            }
    return numConflicts;
}


int high_focal_score_v3(const vector<shared_ptr<Path > >& out_solution, Conflict& result, const vector<bool>& changed_agent, vector<vector<int>>& conflict_matrix)
{

    int max_t = 0;
    for (const auto &sol: out_solution) {
        max_t = std::max<int>(max_t, sol->size());
    }
    result.time = max_t;
    bool found_1st = 0;
    for (int t = 0; t < max_t; ++t) {
        // check drive-drive vertex collisions
        if (found_1st) break;
        for (size_t i = 0; i < out_solution.size(); ++i) {
            State state1 = getState(out_solution[i], t);
            if (found_1st) break;
            for (size_t j = i + 1; j < out_solution.size(); ++j) {
                State state2 = getState(out_solution[j], t);
                if (state1.equalExceptTime(state2)) {
                    result.time = t;
                    result.agent1 = i;
                    result.agent2 = j;
                    result.type = Conflict::Vertex;
                    result.x1 = state1.x;
                    result.y1 = state1.y;
                    found_1st = 1;
                    break;
                }
            }
        }
        // drive-drive edge (swap)
        for (size_t i = 0; i < out_solution.size(); ++i) {
            if (found_1st) break;
            State state1a = getState(out_solution[i], t);
            State state1b = getState(out_solution[i], t + 1);
            for (size_t j = i + 1; j < out_solution.size(); ++j) {
                State state2a = getState(out_solution[j], t);
                State state2b = getState(out_solution[j], t + 1);
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
                    found_1st = 1;
                    break;
                }
            }
        }
    }

    for (size_t i = 0; i < out_solution.size(); ++i) {
        if (!changed_agent[i]) continue;
        for (size_t j = 0; j < out_solution.size(); ++j) {
            if (changed_agent[j] && j < i) continue;
            if (i == j) continue;
            conflict_matrix[i][j] = conflict_matrix[j][i] = 0;
            for (int t = result.time; t < max_t; ++t) {
                State state1a = getState(out_solution[i], t);
                State state2a = getState(out_solution[j], t);
                if (state1a.equalExceptTime(state2a)) {
                    ++conflict_matrix[i][j];
                }

                State state1b = getState(out_solution[i], t + 1);
                State state2b = getState(out_solution[j], t + 1);
                if (state1a.equalExceptTime(state2b) &&
                    state1b.equalExceptTime(state2a)) {
                    ++conflict_matrix[i][j];
                }
            }
            conflict_matrix[j][i] = conflict_matrix[i][j];
        }
    }
    int numConflicts = 0;
    for (size_t i = 0; i < out_solution.size(); ++i)
        for (size_t j = i+1; j < out_solution.size(); ++j)
            numConflicts += conflict_matrix[i][j];
    return numConflicts;
}

void get_block_map(unordered_map<tuple<int, int, int>, int, tuple_hash, tuple_equal>& pointCheckMap,
                   unordered_map<tuple<int, int>, int, tuple_hash, tuple_equal>& lastPointCheckMap,
                   unordered_map<tuple<int, int, int, int, int>, int, tuple_hash, tuple_equal>& edgeCheckMap,
                   const vector<shared_ptr<Path > >& out_solution,
                   int agent_idx)
{
    for (int i = 0; i<out_solution.size(); i++) {
        if (i == agent_idx) continue;
        auto sol = out_solution[i];
        {
            auto pathEntry = (*sol)[0];
            auto idx = make_tuple(pathEntry.state.x, pathEntry.state.y, pathEntry.state.time);
            pointCheckMap[idx] += 1;
        }
        for (int j = 1; j < sol->size(); j++) {
            auto pathEntry = (*sol)[j];
            auto idx = make_tuple(pathEntry.state.x, pathEntry.state.y, pathEntry.state.time);
            pointCheckMap[idx] += 1;
            if (sol->size()-1 == j) {
                auto idx2 = make_tuple(pathEntry.state.x, pathEntry.state.y);
                lastPointCheckMap[idx2] = pathEntry.state.time;
            }

            auto lastEntry = (*sol)[j - 1];
            auto ky_tuple = make_tuple(pathEntry.state.x,
                                       pathEntry.state.y, lastEntry.state.time, lastEntry.state.x, lastEntry.state.y);
            edgeCheckMap[ky_tuple]++;
        }
    }
    return ;
}


int get_block_map(unordered_map<tuple<int, int, int>, int, tuple_hash, tuple_equal>& pointCheckMap,
                   unordered_map<tuple<int, int, int>, int, tuple_hash, tuple_equal>& edgeCheckMap,
                   const vector<shared_ptr<Path > >& out_solution)
{
    int max_t = 0;
    for (const auto &sol: out_solution) {
        max_t = std::max<int>(max_t, sol->size());
    }

    int first_time = max_t;

    for (int i = 0; i<out_solution.size(); i++) {
        auto sol = out_solution[i];
        for (int j = 0; j < sol->size(); j++) {
            auto pathEntry = (*sol)[j];
            auto idx = make_tuple(pathEntry.state.x, pathEntry.state.y, pathEntry.state.time);
            pointCheckMap[idx] += 1;
            if (pointCheckMap[idx] > 1 && first_time > pathEntry.state.time)
                first_time = pathEntry.state.time;
            if (sol->size()-1 == j) {
                for (int k=j+1; k<max_t; k++)
                {
                    auto idx2 = make_tuple(pathEntry.state.x, pathEntry.state.y, k);
                    pointCheckMap[idx2] += 1;
                    if (pointCheckMap[idx2] > 1 && first_time > k)
                        first_time = k;
                }
            }
        }
        for (int j = 1; j < sol->size(); j++) {
            auto pathEntry = (*sol)[j];
            auto lastEntry = (*sol)[j - 1];
            auto ky_tuple = make_tuple(lastEntry.state.x + pathEntry.state.x, lastEntry.state.y + pathEntry.state.y,
                                       lastEntry.state.time);
            edgeCheckMap[ky_tuple]++;
            if (edgeCheckMap[ky_tuple] > 1 && first_time > lastEntry.state.time)
                first_time = lastEntry.state.time;
        }
    }
    return first_time;
}

int high_focal_score_v4(const vector<shared_ptr<Path > >& out_solution, Conflict& result, const vector<bool>& changed_agent, vector<vector<int>>& conflict_matrix)
{
    unordered_map<tuple<int, int, int>, int, tuple_hash, tuple_equal> pointCheckMap;
    unordered_map<tuple<int, int, int>, int, tuple_hash, tuple_equal> edgeCheckMap;

    int first_time = get_block_map(pointCheckMap,  edgeCheckMap, out_solution);
    bool found_1st = false;
    for (size_t i = 0; i < out_solution.size(); ++i) {
        State state1 = getState(out_solution[i], first_time);
        if (found_1st) break;
        for (size_t j = i + 1; j < out_solution.size(); ++j) {
            State state2 = getState(out_solution[j], first_time);
            if (state1.equalExceptTime(state2)) {
                result.time = first_time;
                result.agent1 = i;
                result.agent2 = j;
                result.type = Conflict::Vertex;
                result.x1 = state1.x;
                result.y1 = state1.y;
                found_1st = 1;
                break;
            }
        }
    }
    // drive-drive edge (swap)
    for (size_t i = 0; i < out_solution.size(); ++i) {
        if (found_1st) break;
        State state1a = getState(out_solution[i], first_time);
        State state1b = getState(out_solution[i], first_time + 1);
        for (size_t j = i + 1; j < out_solution.size(); ++j) {
            State state2a = getState(out_solution[j], first_time);
            State state2b = getState(out_solution[j], first_time + 1);
            if (state1a.equalExceptTime(state2b) &&
                state1b.equalExceptTime(state2a)) {
                result.time = first_time;
                result.agent1 = i;
                result.agent2 = j;
                result.type = Conflict::Edge;
                result.x1 = state1a.x;
                result.y1 = state1a.y;
                result.x2 = state1b.x;
                result.y2 = state1b.y;
                found_1st = 1;
                break;
            }
        }
    }

    int numConflicts = 0;
    for (const auto& pair : pointCheckMap) {
        const tuple<int, int, int> &key = pair.first;
        int value = pair.second;
        numConflicts += value*(value-1)/2;
    }
    for (const auto& pair : edgeCheckMap) {
        const tuple<int, int, int>& key = pair.first;
        int value = pair.second;
        numConflicts += value*(value-1)/2;
    }
//    printf("found_1st: %d, numConflicts: %d\n", found_1st, numConflicts);

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

bool check_ans_valid(vector<shared_ptr<Path > > out_solution)
{
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
                    return false;
                }

                State state1b = getState(out_solution[i], t + 1);
                State state2b = getState(out_solution[j], t + 1);
                if (state1a.equalExceptTime(state2b) &&
                    state1b.equalExceptTime(state2a)) {
                    return false;
                }
            }
    return true;
}

