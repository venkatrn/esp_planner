#pragma once
/**
 * @file edge_selector_ssp.h
 * @author Venkatraman Narayanan
 * @brief An Stochastic Shortest Path (SSP) formulation for determining what edges to evaluate next, given
 * a set of paths (containing unevaluated, stochastic edges that are expensive to evaluate) between a start 
 * and goal state. We compute the optimal policy that maps from (current evaluation status of all stochastic edges) 
 * to the next edge that needs to be evaluated.
 * Carnegie Mellon University, 2016
 */

#define BOOST_DYNAMIC_BITSET_DONT_USE_FRIENDS

#include <sbpl_utils/hash_manager/hash_manager.h>
#include <unordered_set>
#include <unordered_map>
#include <vector>

#include <boost/dynamic_bitset.hpp>

using dynamic_bitset = boost::dynamic_bitset<>;

namespace std {
template <class T>
inline void hash_combine(std::size_t &seed, const T &v) {
  std::hash<T> hasher;
  seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}
template <>
struct hash<dynamic_bitset> {
  size_t operator()(const dynamic_bitset &bitset) const {
    size_t res = 0;

    for (auto s : bitset.m_bits) {
      hash_combine(res, s);
    }

    return res;
  }
};
}  // namespace std

namespace sbpl {

// Represent a path as a collection of edge IDs.
typedef std::vector<int> SimplifiedPath;
// A mapping from edge ID to edge status (valid: 1, unevaluated: 0, invalid:
// -1)
typedef std::unordered_map<int, int> EdgeStatusMap;
typedef dynamic_bitset BitVector;

struct Path {
  // Path includes start and goal state IDs.
  std::vector<int> state_ids;
  // We require edge_probabilities.size() = state_ids.size() - 1 (the number of
  // edges is one less than the number of vertices on the path).
  std::vector<double> edge_probabilities;
  // Time to evaluate each edge. We require edge_eval_times.size() =
  // edge_probabilities.size(). Note that the eval_times for deterministic
  // edges don't matter.
  std::vector<double> edge_eval_times;
  // Cost of the path. Type is int rather than double, for traditional SBPL
  // reasons.
  int cost = 0;
};

struct Edge {
  int first = 0;
  int second = 0;
  double probability = 0.0;
  double evaluation_time = 0.0;
  Edge() = delete;
  Edge(int first, int second, double probability, double evaluation_time);
  Edge(int first, int second, double probability);
  Edge(int first, int second);
  bool operator==(const Edge &other) const;
  bool operator!=(const Edge &other) const;
  size_t GetHash() const;
};

struct SSPState {
  BitVector valid_bits;
  BitVector invalid_bits;
  double suboptimality_bound = 0;
  SSPState() = delete;
  explicit SSPState(int num_edges);
  SSPState(const SSPState& other);
  bool operator==(const SSPState &other) const;
  bool operator!=(const SSPState &other) const;
  size_t GetHash() const;
  size_t size() const;
};


class EdgeSelectorSSP {
 public:
  EdgeSelectorSSP();
  // The provided set of paths is preprocessed to remove non-stochastic edges
  // (since we don't care about these for the optimal policy), and convenience
  // data structures are set up.
  void SetPaths(const std::vector<Path> &paths);
  void GetSuccs(int state_id, std::vector<std::vector<int>> *succ_state_ids_map,
                std::vector<std::vector<double>> *succ_state_probabilities_map,
                std::vector<int> *action_ids,
                std::vector<std::vector<double>> *action_costs_map);
  bool IsGoalState(int state_id) const;
  int GetGoalHeuristic(int state_id) const;
 private:
  std::vector<Path> paths_;
  std::vector<SimplifiedPath> simplified_paths_;
  std::vector<BitVector> path_bit_vectors_;
  sbpl_utils::HashManager<Edge> edge_hasher_;
  sbpl_utils::HashManager<SSPState> state_hasher_;

 public:
  // Returns
  // -1 if path is invalid.
  // 0 if path status could be valid or invalid (i.e, path has some unevaluated
  // edges).
  // 1 if path is valid.
  int PathStatus(const BitVector &path,
                 const SSPState &ssp_state) const;
  int PathStatus(const SimplifiedPath &path,
                 const EdgeStatusMap &edge_status_map) const;
  int PathStatus(const SimplifiedPath &path,
                 const std::unordered_set<int> &evaluated_edges,
                 const std::unordered_set<int> &valid_edges) const;
  void ComputeBounds(const EdgeStatusMap &edge_status_map, int *lower_bound,
                     int *upper_bound) const;
  void ComputeBounds(const SSPState &ssp_state, int *lower_bound,
                     int *upper_bound) const;
  int GetSuboptimalityBound(const SSPState &ssp_state) const;
  int ComputeTransitionCost(const SSPState &parent_state,
                            const SSPState &child_state, int action_id) const;
};
} // namspace sbpl