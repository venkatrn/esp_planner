/**
 * @file edge_selector_ssp.h
 * @author Venkatraman Narayanan
 * @brief An Stochastic Shortest Path (SSP) formulation for determining what edges to evaluate next, given
 * a set of paths (containing unevaluated, stochastic edges that are expensive to evaluate) between a start
 * and goal state. We compute the optimal policy that maps from (current evaluation status of all stochastic edges)
 * to the next edge that needs to be evaluated.
 * Carnegie Mellon University, 2016
 */

#include <esp_planner/edge_selector_ssp.h>

#include <limits>

#include <cassert>

using namespace std;

namespace {
constexpr double kFloatingPointTolerance = 1e-3;
constexpr int kEdgeEvalTimeMultiplier = 10;
}

namespace sbpl {

Edge::Edge(int first, int second, double probability, double evaluation_time) {
  this->first = first;
  this->second = second;
  this->probability = probability;
  this->evaluation_time = evaluation_time;

  if (probability < 0 || probability > 1.0) {
    printf("ERROR: Illegal probability value %f for edge: (%d %d)\n", probability,
           first, second);
  }

  if (evaluation_time < 0) {
    printf("ERROR: Illegal evaluation time %f for edge: (%d %d)\n",
           evaluation_time, first, second);
  }
}
Edge::Edge(int first, int second, double probability) : Edge(first, second,
                                                               probability, 0.0) {
}
Edge::Edge(int first, int second) : Edge(first, second, 0.0, 0.0) {
}
bool Edge::operator==(const Edge &other) const {
  return (first == other.first) && (second == other.second);
}
bool Edge::operator!=(const Edge &other) const {
  return !(*this == other);
}
size_t Edge::GetHash() const {
  size_t hash_value = std::hash<int>()(first);
  std::hash_combine(hash_value, std::hash<int>()(second));
  return hash_value;
}


SSPState::SSPState(int num_edges) {
  valid_bits.resize(num_edges, false);
  invalid_bits.resize(num_edges, false);
}
SSPState::SSPState(const SSPState& other) {
  this->valid_bits = other.valid_bits;
  this->invalid_bits = other.invalid_bits;
}
bool SSPState::operator==(const SSPState &other) const {
  return valid_bits == other.valid_bits && invalid_bits == other.invalid_bits;
}
bool SSPState::operator!=(const SSPState &other) const {
  return !(*this == other);
}
size_t SSPState::GetHash() const {
  size_t hash_value = std::hash<dynamic_bitset>()(valid_bits);
  std::hash_combine(hash_value, std::hash<dynamic_bitset>()(invalid_bits));
  return hash_value;
}
size_t SSPState::size() const {
  return valid_bits.size();
}



EdgeSelectorSSP::EdgeSelectorSSP() {

}
void EdgeSelectorSSP::SetPaths(const std::vector<Path> &paths) {
  paths_ = paths;
  simplified_paths_.resize(paths.size());
  path_bit_vectors_.resize(paths.size());

  for (size_t ii = 0; ii < paths.size(); ++ii) {
    const auto &state_ids = paths[ii].state_ids;
    simplified_paths_[ii].reserve(state_ids.size() - 1);

    for (size_t jj = 0; jj < state_ids.size() - 1; ++jj) {
      const double edge_probability = paths[ii].edge_probabilities[jj];
      const Edge edge(state_ids[jj], state_ids[jj + 1], edge_probability);

      // cout << edge.first << " " << edge.second << endl;
      // cout << state_ids[jj] << " " << state_ids[jj+1] << endl;
      // cout << edge_probability << endl;

      if (edge_probability >= 1.0 - kFloatingPointTolerance) {
        continue;
      }

      const int edge_id = edge_hasher_.GetStateIDForceful(edge);
      // cout << edge_id << endl;
      simplified_paths_[ii].push_back(edge_id);
    }
  }

  const int num_edges = edge_hasher_.Size();
  cout << "Number of stochastic edges across all paths: " << num_edges << endl;

  // Create the path_bit_vectors.
  for (size_t ii = 0; ii < paths.size(); ++ii) {
    auto &path_bit_vector = path_bit_vectors_[ii];
    path_bit_vector.resize(num_edges);
    const auto &simplified_paths = simplified_paths_[ii];

    for (size_t jj = 0; jj < simplified_paths.size(); ++jj) {
      /////////////////////////////// WARNING ////////////////////////////////
      // This assumes edge_ids are generated sequentially by the edge_hasher!!!
      /////////////////////////////// WARNING ////////////////////////////////
      const int edge_id = simplified_paths[jj];
      assert(edge_id < num_edges);
      path_bit_vector.set(edge_id);
    }

    cout << "Path " << ii << ": " << path_bit_vector << endl;
  }
}

int EdgeSelectorSSP::PathStatus(const BitVector &path,
                                const SSPState &ssp_state) const {
  if (path.size() != ssp_state.size()) {
    printf("ERROR: Number of bits in the path and ssp_state are not equal\n");
  }

  const auto &unevaluated_edges_in_path = path - ssp_state.valid_bits;
  const auto &invalid_edges_in_path = path & ssp_state.invalid_bits;

  if (invalid_edges_in_path.any()) {
    return -1;
  } else if (unevaluated_edges_in_path.any()) {
    return 0;
  } else {
    return 1;
  }
}

int EdgeSelectorSSP::PathStatus(const SimplifiedPath &path,
                                const EdgeStatusMap &edge_status_map) const {
  // Assume valid until proven otherwise.
  int path_status = 1;

  for (int edge_id : path) {
    auto it = edge_status_map.find(edge_id);

    if (edge_status_map.find(edge_id) == edge_status_map.end()) {
      printf("ERROR: Edge Status Map does not contain edge %d\n", edge_id);
      return -1;
    }

    const int edge_status = it->second;

    if (edge_status == -1) {
      return -1;
    } else if (edge_status == 0) {
      path_status = 0;
    }
  }

  return path_status;
}

int EdgeSelectorSSP::PathStatus(const SimplifiedPath &path,
                                const std::unordered_set<int> &evaluated_edges,
                                const std::unordered_set<int> &valid_edges) const {

  // Assume valid until proven otherwise.
  int path_status = 1;

  for (int edge_id : path) {
    if (evaluated_edges.find(edge_id) != evaluated_edges.end()) {
      if (valid_edges.find(edge_id) == valid_edges.end()) {
        // Evaluated and not valid.
        return -1;
      }
    } else {
      // Not evaluated. Need to keep going to check if we can get an invalid
      // edge.
      path_status = 0;
    }
  }

  return path_status;
}

void EdgeSelectorSSP::ComputeBounds(const EdgeStatusMap &edge_status_map,
                                    int *lower_bound,
                                    int *upper_bound) const {
  *lower_bound = std::numeric_limits<int>::max();
  *upper_bound = std::numeric_limits<int>::max();

  for (size_t ii = 0; ii < simplified_paths_.size(); ++ii) {
    const auto &path = simplified_paths_[ii];
    const int cost = paths_[ii].cost;
    const int path_status = PathStatus(path, edge_status_map);

    if (path_status == 0 || path_status == 1) {
      *lower_bound = std::min(*lower_bound, cost);
    }

    if (path_status == 1) {
      *upper_bound = std::min(*upper_bound, cost);
    }
  }

  // There should be at least one deterministic path.
  assert(*upper_bound != std::numeric_limits<int>::max());
}

void EdgeSelectorSSP::ComputeBounds(const SSPState &ssp_state,
                                    int *lower_bound,
                                    int *upper_bound) const {
  *lower_bound = std::numeric_limits<int>::max();
  *upper_bound = std::numeric_limits<int>::max();

  for (size_t ii = 0; ii < path_bit_vectors_.size(); ++ii) {
    const auto &path_bit_vector = path_bit_vectors_[ii];
    const int cost = paths_[ii].cost;
    const int path_status = PathStatus(path_bit_vector, ssp_state);

    if (path_status == 0 || path_status == 1) {
      *lower_bound = std::min(*lower_bound, cost);
    }

    if (path_status == 1) {
      *upper_bound = std::min(*upper_bound, cost);
    }
  }

  // There should be at least one deterministic path.
  assert(*upper_bound != std::numeric_limits<int>::max());

}

int EdgeSelectorSSP::GetSuboptimalityBound(const SSPState &ssp_state) const {
  int lower_bound = 0;
  int upper_bound = 0;
  ComputeBounds(ssp_state, &lower_bound, &upper_bound);
  return (upper_bound - lower_bound);
}

int EdgeSelectorSSP::ComputeTransitionCost(const SSPState &parent_state,
                                           const SSPState &child_state, int action_id) const {
  const int parent_subopt_bound = GetSuboptimalityBound(parent_state);
  const int child_subopt_bound = GetSuboptimalityBound(child_state);

  const Edge &edge = edge_hasher_.GetState(action_id);
  // Compute the area of the trapezoid formed by the parallel sides
  // with length parent_subopt_bound and child_subopt_bound, and height
  // edge.evaluation_time.
  const int cost = static_cast<int>(0.5 * kEdgeEvalTimeMultiplier *
                                    edge.evaluation_time * static_cast<double>(parent_subopt_bound +
                                                                               child_subopt_bound));
  return cost;
}

bool EdgeSelectorSSP::IsGoalState(int state_id) const {
  auto &ssp_state = state_hasher_.GetState(state_id);
  const int suboptimality_bound = GetSuboptimalityBound(ssp_state);
  return (suboptimality_bound == 0);
}

int EdgeSelectorSSP::GetGoalHeuristic(int state_id) const {
  // TODO: implement.
  return 0;
}

void EdgeSelectorSSP::GetSuccs(int state_id,
                               std::vector<std::vector<int>> *succ_state_ids_map,
                               std::vector<std::vector<double>> *succ_state_probabilities_map,
                               std::vector<int> *action_ids,
                               std::vector<std::vector<double>> *action_costs_map) {
  auto &parent_state = state_hasher_.GetState(state_id);
  succ_state_ids_map->clear();
  succ_state_probabilities_map->clear();
  action_ids->clear();
  action_costs_map->clear();

  BitVector evaluated_edges = parent_state.valid_bits |
                              parent_state.invalid_bits;
  const int num_edges = evaluated_edges.size();
  const int num_unevaluated_edges = num_edges - static_cast<int>
                                    (evaluated_edges.count());
  assert(num_unevaluated_edges >= 0);

  action_ids->reserve(num_unevaluated_edges);
  action_costs_map->reserve(num_unevaluated_edges);
  succ_state_ids_map->reserve(num_unevaluated_edges);
  succ_state_probabilities_map->reserve(num_unevaluated_edges);

  for (size_t ii = 0; ii < num_edges; ++ii) {
    // Ignore edge if already evaluated.
    if (evaluated_edges.test(ii)) {
      continue;
    }

    action_ids->push_back(static_cast<int>(ii));
    const Edge &edge = edge_hasher_.GetState(ii);

    SSPState succ_optimistic = parent_state;
    succ_optimistic.valid_bits.set(ii, true);
    const double prob_optimistic = edge.probability;
    const double cost_optimistic = ComputeTransitionCost(parent_state,
                                                         succ_optimistic, ii);
    const int succ_optimistic_id = state_hasher_.GetStateIDForceful(
                                     succ_optimistic);

    SSPState succ_pessimistic = parent_state;
    succ_pessimistic.invalid_bits.set(ii, true);
    const double prob_pessimistic = 1 - edge.probability;
    const double cost_pessimistic = ComputeTransitionCost(parent_state,
                                                          succ_pessimistic, ii);
    const int succ_pessimistic_id = state_hasher_.GetStateIDForceful(
                                      succ_pessimistic);

    vector<int> succ_states = {succ_optimistic_id, succ_optimistic_id};
    vector<double> succ_probabilities = {prob_optimistic, prob_pessimistic};
    vector<double> succ_costs = {cost_optimistic, cost_pessimistic};

    succ_state_ids_map->push_back(succ_states);
    succ_state_probabilities_map->push_back(succ_probabilities);
    action_costs_map->push_back(succ_costs);
  }
}
} // namespace
