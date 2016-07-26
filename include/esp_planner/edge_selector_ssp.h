#pragma once

#include <sbpl_utils/hash_manager/hash_manager.h>
#include <unordered_set>
#include <unordered_map>
#include <vector>

namespace sbpl {
struct Path {
  // Path includes start and goal state IDs.
  std::vector<int> state_ids;
  // We require edge_probabilities.size() = state_ids.size() - 1 (the number of
  // edges is one less than the number of vertices on the path).
  std::vector<double> edge_probabilities;
  // Cost of the path. Type is int rather than double, for traditional SBPL
  // reasons.
  int cost = 0;
};
struct Edge {
  int first = 0;
  int second = 0;
  double probability;
  Edge() {
  };
  Edge(int first, int second) {
    this->first = first;
    this->second = second;
  }
  Edge(int first, int second, double probability) {
    Edge(first, second);
    this->probability = probability;
  }
  bool operator==(const Edge &other) const {
    return first == other.first && second == other.second;
  }
  size_t GetHash() const {
    return std::hash<int>()(first) ^ std::hash<int>()(second);
  }
};
typedef std::vector<int> SimplifiedPath;
class EdgeSelectorSSP {
 public:
  EdgeSelectorSSP();
  void SetPaths(const std::vector<Path> &paths);
 private:
  std::vector<Path> paths_;
  std::vector<SimplifiedPath> simplified_paths_;
  sbpl_utils::HashManager<Edge> edge_hasher_;
  int PathStatus(const SimplifiedPath &path,
                 const std::unordered_set<int> &evaluated_edges,
                 const std::unordered_set<int> &valid_edges) const;
  void ComputeBounds(const std::vector<SimplifiedPath> &paths,
                     const std::unordered_set<int> &evaluated_edges,
                     const std::unordered_set<int> &valid_edges, int *lower_bound,
                     int *upper_bound) const;
};
} // namspace
