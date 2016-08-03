#pragma once

#include <iostream>
#include <vector>

#include <cstdio>

namespace sbpl {
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

std::ostream &operator<< (std::ostream &stream, const Edge &edge);

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
} // namespace sbpl
