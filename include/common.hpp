
#ifndef ITACBS_REMAKE_COMMON_HPP
#define ITACBS_REMAKE_COMMON_HPP

#include <tuple>
#include <list>
#include <vector>
#include <set>
#include <ctime>
#include <fstream>
#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision
#include <algorithm>
#include <cmath>
#include <chrono>
#include <string>
#include <filesystem>
#include <unordered_set>
#include <queue>

#include <boost/heap/pairing_heap.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <boost/functional/hash.hpp>
#include <boost/heap/d_ary_heap.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/heap/priority_queue.hpp>
#include <boost/range/algorithm/reverse.hpp>
#include <boost/utility.hpp>
#include <boost/container/vector.hpp>
#include <boost/dynamic_bitset.hpp>
#include <boost/thread.hpp>
#include <boost/random.hpp>

using boost::dynamic_bitset;
using boost::range::reverse;
using boost::shared_ptr;
using boost::random::mt19937;
using boost::random::uniform_real_distribution;
using boost::heap::pairing_heap;
using boost::heap::compare;
using boost::unordered_map;
using boost::unordered_set;
//using std::unordered_set;
using boost::heap::priority_queue;
//using boost::container::vector;
using std::vector;
using std::list;
using std::set;
using std::get;
using boost::tuple;
using boost::make_tuple;
using std::pair;
using std::make_pair;
using std::tie;
using std::min;
using std::max;
using std::make_shared;
using std::queue;
using std::clock;
using std::cout;
using std::endl;
using std::ofstream;
using std::cerr;
using std::string;

const int MAX_NODES = 1e7;
#define MAX_TIMESTEP INT_MAX / 2
#define MAX_COST INT_MAX / 2
#define MAX_NODES INT_MAX / 2
#define MAX_NUM_STATS 4000
const int dx[5] = {0, 0, 1, -1, 0};
const int dy[5] = {1, -1, 0, 0, 0};
const int INF7f = 0x7fffff;
const int BAD_TA_ANS = 1e6;
struct State {
    State(){}
    State(int time, int x, int y) : time(time), x(x), y(y) {}

    bool operator==(const State &s) const {
        return time == s.time && x == s.x && y == s.y;
    }

    bool equalExceptTime(const State &s) const { return x == s.x && y == s.y; }

    friend std::ostream &operator<<(std::ostream &os, const State &s) {
        return os << s.time << ": (" << s.x << "," << s.y << ")";
        // return os << "(" << s.x << "," << s.y << ")";
    }

    int time;
    int x;
    int y;
};



struct Location {
    Location() = default;

    Location(int x, int y) : x(x), y(y) {}

    int x;
    int y;

    bool operator<(const Location &other) const {
        return std::tie(x, y) < std::tie(other.x, other.y);
    }

//    bool operator>(const Location &other) const {
//        return std::tie(x, y) > std::tie(other.x, other.y);
//    }

    bool operator==(const Location &other) const {
        return std::tie(x, y) == std::tie(other.x, other.y);
    }

    friend std::ostream &operator<<(std::ostream &os, const Location &c) {
        return os << "(" << c.x << "," << c.y << ")";
    }
};

namespace boost {
    template<>
    struct hash<Location> {
        size_t operator()(const Location &s) const {
            size_t seed = 0;
            boost::hash_combine(seed, s.x);
            boost::hash_combine(seed, s.y);
            return seed;
        }
    };
}

namespace boost {
    template<>
    struct hash<State> {
        size_t operator()(const State &s) const {
            size_t seed = 0;
            boost::hash_combine(seed, s.time);
            boost::hash_combine(seed, s.x);
            boost::hash_combine(seed, s.y);
            return seed;
        }
    };
}  // namespace std


struct Conflict {
    enum Type {
        Vertex,
        Edge,
    };

    int time;
    size_t agent1;
    size_t agent2;
    Type type;

    int x1;
    int y1;
    int x2;
    int y2;

    friend std::ostream &operator<<(std::ostream &os, const Conflict &c) {
        switch (c.type) {
            case Vertex:
                return os << c.time << ": Vertex(" << c.x1 << "," << c.y1 << ")";
            case Edge:
                return os << c.time << ": Edge(" << c.x1 << "," << c.y1 << "," << c.x2
                          << "," << c.y2 << ")";
        }
        return os;
    }
};


struct VertexConstraint {
    VertexConstraint(int time, int x, int y) : time(time), x(x), y(y) {}
    VertexConstraint(int time, int x, int y, int for_who) : time(time), x(x), y(y),for_who(for_who) {}
    int time;
    int x;
    int y;
    int for_who;
    bool operator<(const VertexConstraint& other) const {
        return std::tie(time, x, y) < std::tie(other.time, other.x, other.y);
    }

    bool operator==(const VertexConstraint& other) const {
        return std::tie(time, x, y) == std::tie(other.time, other.x, other.y);
    }

    friend std::ostream& operator<<(std::ostream& os, const VertexConstraint& c) {
        return os << "VC(" << c.time << "," << c.x << "," << c.y << ")";
    }
};

namespace std {
    template <>
    struct hash<VertexConstraint> {
        size_t operator()(const VertexConstraint& s) const {
            size_t seed = 0;
            boost::hash_combine(seed, s.time);
            boost::hash_combine(seed, s.x);
            boost::hash_combine(seed, s.y);
            return seed;
        }
    };
}  // namespace std

struct EdgeConstraint {
    EdgeConstraint(int time, int x1, int y1, int x2, int y2)
            : time(time), x1(x1), y1(y1), x2(x2), y2(y2) {}
    EdgeConstraint(int time, int x1, int y1, int x2, int y2, int for_who)
            : time(time), x1(x1), y1(y1), x2(x2), y2(y2), for_who(for_who) {}
    int time;
    int x1;
    int y1;
    int x2;
    int y2;
    int for_who;
    bool operator<(const EdgeConstraint& other) const {
        return std::tie(time, x1, y1, x2, y2) <
               std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
    }

    bool operator==(const EdgeConstraint& other) const {
        return std::tie(time, x1, y1, x2, y2) ==
               std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
    }

    friend std::ostream& operator<<(std::ostream& os, const EdgeConstraint& c) {
        return os << "EC(" << c.time << "," << c.x1 << "," << c.y1 << "," << c.x2
                  << "," << c.y2 << ")";
    }
};

namespace std {
    template <>
    struct hash<EdgeConstraint> {
        size_t operator()(const EdgeConstraint& s) const {
            size_t seed = 0;
            boost::hash_combine(seed, s.time);
            boost::hash_combine(seed, s.x1);
            boost::hash_combine(seed, s.y1);
            boost::hash_combine(seed, s.x2);
            boost::hash_combine(seed, s.y2);
            return seed;
        }
    };
}  // namespace std

struct Constraints {
    std::unordered_set<VertexConstraint> vertexConstraints;
    std::unordered_set<EdgeConstraint> edgeConstraints;

    void add(const Constraints &other) {
        vertexConstraints.insert(other.vertexConstraints.begin(),
                                 other.vertexConstraints.end());
        edgeConstraints.insert(other.edgeConstraints.begin(),
                               other.edgeConstraints.end());
    }

    bool overlap(const Constraints &other) const {
        for (const auto &vc: vertexConstraints) {
            if (other.vertexConstraints.count(vc) > 0) {
                return true;
            }
        }
        for (const auto &ec: edgeConstraints) {
            if (other.edgeConstraints.count(ec) > 0) {
                return true;
            }
        }
        return false;
    }

    friend std::ostream &operator<<(std::ostream &os, const Constraints &c) {
        for (const auto &vc: c.vertexConstraints) {
            os << vc << std::endl;
        }
        for (const auto &ec: c.edgeConstraints) {
            os << ec << std::endl;
        }
        return os;
    }

    bool stateValid(const State& state){
        const auto &con = this->vertexConstraints;
        return con.find(VertexConstraint(state.time, state.x, state.y)) == con.end();
    }

    bool transitionValid(const State& s1, const State& s2) {
        const auto &con = this->edgeConstraints;
        return con.find(EdgeConstraint(s1.time, s1.x, s1.y, s2.x, s2.y)) == con.end();
    }
};


void createConstraintsFromConflict(
        const Conflict &conflict, unordered_map<size_t, Constraints> &constraints);
int read_map_file(std::filesystem::path map_file_path, vector<vector<bool> >& ret_map);


struct PathEntry {
    State state;
    int fScore;
    int gScore;
    int focalScore;
    shared_ptr<PathEntry> parent;

    PathEntry() {}
    PathEntry(State state, int fScore, int gScore, shared_ptr<PathEntry> parent)
            : state(state), fScore(fScore), gScore(gScore), parent(parent) {}

    PathEntry(State state, int fScore, int gScore, int focalScore, shared_ptr<PathEntry> parent)
            : state(state), fScore(fScore), gScore(gScore), focalScore(focalScore), parent(parent) {}
};

struct PathEntryCompare {
    bool operator()(const shared_ptr<PathEntry>& a, const shared_ptr<PathEntry>& b) const {
        if (a->fScore != b->fScore) {
            return a->fScore > b->fScore;
        } else {
            return a->gScore < b->gScore;
        }
    }
};

struct PathEntryCompare2 {
    bool operator()(const shared_ptr<PathEntry>& a, const shared_ptr<PathEntry>& b) const {
        if (a->focalScore != b->focalScore) return a->focalScore > b->focalScore;
        else {
            if (a->fScore != b->fScore) return a->fScore > b->fScore;
        }
        return a->gScore < b->gScore;
    }
};


typedef vector<PathEntry> Path;


class Timer {
public:
    Timer()
            : start_(std::chrono::high_resolution_clock::now()),
              end_(std::chrono::high_resolution_clock::now()) {}

    void reset() { start_ = std::chrono::high_resolution_clock::now(); }

    void stop() { end_ = std::chrono::high_resolution_clock::now(); }

    double elapsedSeconds() const {
        auto timeSpan = std::chrono::duration_cast<std::chrono::duration<double>>(
                end_ - start_);
        return timeSpan.count();
    }

private:
    std::chrono::high_resolution_clock::time_point start_;
    std::chrono::high_resolution_clock::time_point end_;
};

class ScopedTimer : public Timer {
public:
    ScopedTimer() {}

    ~ScopedTimer() {
        stop();
        std::cout << "Elapsed: " << elapsedSeconds() << " s" << std::endl;
    }
};

class BoolMatrix3D {
private:
    boost::dynamic_bitset<> bits;
    size_t rows, cols, depths;

public:
    BoolMatrix3D(size_t rows, size_t cols, size_t depths)
            : rows(rows), cols(cols), depths(depths), bits(rows * cols * depths) {}

    void set(size_t row, size_t col, size_t depth, bool value) {
        if (row >= rows || col >= cols || depth >= depths) {
            throw std::out_of_range("Index out of bounds");
        }
        bits[row * cols * depths + col * depths + depth] = value;
    }

    bool get(size_t row, size_t col, size_t depth) const {
        if (row >= rows || col >= cols || depth >= depths) {
            throw std::out_of_range("Index out of bounds");
        }
        return bits[row * cols * depths + col * depths + depth];
    }
};

struct tuple_hash
{
    template <typename T1, typename T2, typename T3, typename T4, typename T5>
    std::size_t operator()(const boost::tuples::tuple<T1, T2, T3, T4, T5>& k) const
    {
        std::size_t seed = 114514;
        boost::hash_combine(seed, k.template get<0>());
        boost::hash_combine(seed, k.template get<1>());
        boost::hash_combine(seed, k.template get<2>());
        boost::hash_combine(seed, k.template get<3>());
        boost::hash_combine(seed, k.template get<4>());
        return seed;
    }

    template <typename T1, typename T2, typename T3>
    std::size_t operator()(const boost::tuples::tuple<T1, T2, T3>& k) const
    {
        std::size_t seed = 114514;
        boost::hash_combine(seed, k.template get<0>());
        boost::hash_combine(seed, k.template get<1>());
        boost::hash_combine(seed, k.template get<2>());
        return seed;
    }

    template <typename T1, typename T2>
    std::size_t operator()(const boost::tuples::tuple<T1, T2>& k) const
    {
        std::size_t seed = 114514;
        boost::hash_combine(seed, k.template get<0>());
        boost::hash_combine(seed, k.template get<1>());
        return seed;
    }
};

struct tuple_equal {
    template <typename T1, typename T2, typename T3, typename T4, typename T5>
    bool operator()(const boost::tuples::tuple<T1, T2, T3, T4, T5>& lhs, const boost::tuples::tuple<T1, T2, T3, T4, T5>& rhs) const {
        return lhs.template get<0>() == rhs.template get<0>() &&
               lhs.template get<1>() == rhs.template get<1>() &&
               lhs.template get<2>() == rhs.template get<2>() &&
               lhs.template get<3>() == rhs.template get<3>() &&
               lhs.template get<4>() == rhs.template get<4>();
    }

    template <typename T1, typename T2, typename T3>
    bool operator()(const boost::tuples::tuple<T1, T2, T3>& lhs, const boost::tuples::tuple<T1, T2, T3>& rhs) const {
        return lhs.template get<0>() == rhs.template get<0>() &&
               lhs.template get<1>() == rhs.template get<1>() &&
               lhs.template get<2>() == rhs.template get<2>();
    }

    template <typename T1, typename T2>
    bool operator()(const boost::tuples::tuple<T1, T2>& lhs, const boost::tuples::tuple<T1, T2>& rhs) const {
        return lhs.template get<0>() == rhs.template get<0>() &&
               lhs.template get<1>() == rhs.template get<1>();
    }
};

class Edge{

public:
    int from, to, cap;
    Edge(int from, int to, int cap)
    {
        this->from = from;
        this->to = to;
        this->cap = cap;
    }
    bool operator < (const Edge& other) const {
        return cap < other.cap;
    }
};


State getState(shared_ptr<Path> sol, int t);

void get_block_map(unordered_map<tuple<int, int, int>, int, tuple_hash, tuple_equal>& pointCheckMap,
                   unordered_map<tuple<int, int>, int, tuple_hash, tuple_equal>& lastPointCheckMap,
                   unordered_map<tuple<int, int, int, int, int>, int, tuple_hash, tuple_equal>& edgeCheckMap,
                   const vector<shared_ptr<Path > >& out_solution,
                   int agent_idx);
int get_block_map(unordered_map<tuple<int, int, int>, int, tuple_hash, tuple_equal>& pointCheckMap,
                   unordered_map<tuple<int, int, int>, int, tuple_hash, tuple_equal>& edgeCheckMap,
                   const vector<shared_ptr<Path > >& out_solution);
int high_focal_score_v2(const vector<shared_ptr<Path > >& out_solution, Conflict& result);
int high_focal_score_v3(const vector<shared_ptr<Path > >& out_solution, Conflict& result, const vector<bool>& changed_agent, vector<vector<int>>& conflict_matrix);
int high_focal_score_v4(const vector<shared_ptr<Path > >& out_solution, Conflict& result, const vector<bool>& changed_agent, vector<vector<int>>& conflict_matrix);
bool check_ans_valid(vector<shared_ptr<Path > > out_solution);
#endif //ITACBS_REMAKE_COMMON_HPP
