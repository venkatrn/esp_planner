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
#include <iostream>
#include <fstream>

using namespace std;

namespace {
constexpr double kFloatingPointTolerance = 1e-3;
constexpr double kEdgeEvalTimeMultiplier = 10;
}

namespace sbpl {

SSPState::SSPState(int num_edges) {
  valid_bits.resize(num_edges, false);
  invalid_bits.resize(num_edges, false);
}
SSPState::SSPState(const SSPState &other) :
  valid_bits{other.valid_bits},
  invalid_bits{other.invalid_bits},
  suboptimality_bound(other.suboptimality_bound) {}

bool SSPState::operator==(const SSPState &other) const {
  return valid_bits == other.valid_bits && invalid_bits == other.invalid_bits;
}
bool SSPState::operator!=(const SSPState &other) const {
  return !(*this == other);
}
size_t SSPState::GetHash() const {
  size_t hash_value = std::hash<dynamic_bitset>()(valid_bits);
  boost::hash_combine(hash_value, std::hash<dynamic_bitset>()(invalid_bits));
  return hash_value;
}
size_t SSPState::size() const {
  return valid_bits.size();
}

string SSPState::to_string() const {
  string edge_statuses(valid_bits.size(), '-');

  for (size_t ii = 0; ii < valid_bits.size(); ++ii) {
    if (valid_bits.test(ii)) {
      edge_statuses[ii] = '1';
    } else if (invalid_bits.test(ii)) {
      edge_statuses[ii] = '0';
    }
  }

  return edge_statuses;
}

vector<int> SSPState::GetUnevaluatedEdges() const {
  vector<int> unevaluated_edges;
  unevaluated_edges.reserve(size());
  const BitVector evaluated_edges = valid_bits | invalid_bits;

  for (size_t ii = 0; ii < size(); ++ii) {
    if (!evaluated_edges.test(ii)) {
      unevaluated_edges.push_back(static_cast<int>(ii));
    }
  }

  return unevaluated_edges;
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
      const double edge_eval_time = paths[ii].edge_eval_times[jj];
      const Edge edge(state_ids[jj], state_ids[jj + 1], edge_probability,
                      edge_eval_time);

      // cout << edge.first << " " << edge.second << endl;
      // cout << state_ids[jj] << " " << state_ids[jj+1] << endl;
      // cout << edge_probability << endl;

      if (edge_probability >= 1.0 - kFloatingPointTolerance) {
        continue;
      }

      const int edge_id = edge_hasher_.GetStateIDForceful(edge);
      // TODO: validate that common edges across paths have same probabilities
      // and evaluation times, rather than just assuming.
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
  int lower_bound_idx = -1;
  int upper_bound_idx = -1;
  ComputeBounds(ssp_state, lower_bound, upper_bound, &lower_bound_idx,
                &upper_bound_idx);
}

void EdgeSelectorSSP::ComputeBounds(const SSPState &ssp_state,
                                    int *lower_bound,
                                    int *upper_bound,
                                    int *lower_bound_idx,
                                    int *upper_bound_idx) const {
  *lower_bound = std::numeric_limits<int>::max();
  *upper_bound = std::numeric_limits<int>::max();

  *lower_bound_idx = -1;
  *upper_bound_idx = -1;

  for (size_t ii = 0; ii < path_bit_vectors_.size(); ++ii) {
    const auto &path_bit_vector = path_bit_vectors_[ii];
    const int cost = paths_[ii].cost;
    const int path_status = PathStatus(path_bit_vector, ssp_state);

    if (path_status == 0 || path_status == 1) {
      if (cost < *lower_bound) {
        *lower_bound = cost;
        *lower_bound_idx = static_cast<int>(ii);
      }
    }

    if (path_status == 1) {
      if (cost < *upper_bound) {
        *upper_bound = cost;
        *upper_bound_idx = static_cast<int>(ii);
      }
    }
  }

  // There should be at least one deterministic path.
  assert(*upper_bound != std::numeric_limits<int>::max());

}

double EdgeSelectorSSP::GetSuboptimalityBound(const SSPState &ssp_state) const {
  int lower_bound = 0;
  int upper_bound = 0;
  ComputeBounds(ssp_state, &lower_bound, &upper_bound);
  if (lower_bound < kFloatingPointTolerance && upper_bound < kFloatingPointTolerance) {
    return 1.0;
  } else if (upper_bound == std::numeric_limits<double>::max() && lower_bound == std::numeric_limits<double>::max()) {
    return 1.0;
  } else if (lower_bound < kFloatingPointTolerance) {
    return std::numeric_limits<double>::max();
  } else if (upper_bound == std::numeric_limits<double>::max() || lower_bound == std::numeric_limits<double>::max()) {
    return std::numeric_limits<double>::max();
  }
  return static_cast<double>(upper_bound) / static_cast<double>(lower_bound);
}

double EdgeSelectorSSP::ComputeTransitionCost(const SSPState &parent_state,
                                              const SSPState &child_state, int edge_id) const {

  const Edge &edge = edge_hasher_.GetState(edge_id);
  // Compute the area of the trapezoid formed by the parallel sides
  // with length parent_subopt_bound and child_subopt_bound, and height
  // edge.evaluation_time.
  const double cost = 0.5 * edge.evaluation_time * static_cast<double>
                      (parent_state.suboptimality_bound +
                       child_state.suboptimality_bound);
  return cost;
}

bool EdgeSelectorSSP::IsGoalState(int state_id) const {
  auto &ssp_state = state_hasher_.GetState(state_id);
  return (ssp_state.suboptimality_bound < 1.0 + kFloatingPointTolerance);
}

double EdgeSelectorSSP::GetGoalHeuristic(int state_id) const {
  // return 0;
  auto &ssp_state = state_hasher_.GetState(state_id);
  const vector<int> unevaluated_edges = ssp_state.GetUnevaluatedEdges();
  double min_eval_time = std::numeric_limits<double>::max();

  for (int edge_id : unevaluated_edges) {
    const Edge &edge = edge_hasher_.GetState(edge_id);
    min_eval_time = std::min(min_eval_time, edge.evaluation_time);
  }

  // TODO: implement more informed heuristic.
  return 0.5 * min_eval_time * ssp_state.suboptimality_bound;
}

Edge EdgeSelectorSSP::EdgeIDToEdge(int edge_id) const {
  return edge_hasher_.GetState(edge_id);
}

SSPState EdgeSelectorSSP::SSPStateIDToSSPState(int ssp_state_id) const {
  return state_hasher_.GetState(ssp_state_id);
}

int EdgeSelectorSSP::GetBestValidPathIdx(const SSPState &ssp_state) const {
  int lower_bound = -1;
  int lower_bound_idx = -1;
  int upper_bound = -1;
  int upper_bound_idx = -1;
  ComputeBounds(ssp_state, &lower_bound, &upper_bound, &lower_bound_idx,
                &upper_bound_idx);
  // The path corresponding to the upper bound is the valid executable path
  // (i.e, all the edges on this path have been evaluated as valid).
  return upper_bound_idx;
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
    succ_optimistic.suboptimality_bound = GetSuboptimalityBound(succ_optimistic);
    const double prob_optimistic = edge.probability;
    const double cost_optimistic = ComputeTransitionCost(parent_state,
                                                         succ_optimistic, ii);
    const int succ_optimistic_id = state_hasher_.GetStateIDForceful(
                                     succ_optimistic);

    SSPState succ_pessimistic = parent_state;
    succ_pessimistic.invalid_bits.set(ii, true);
    succ_pessimistic.suboptimality_bound = GetSuboptimalityBound(succ_pessimistic);
    const double prob_pessimistic = 1 - edge.probability;
    const double cost_pessimistic = ComputeTransitionCost(parent_state,
                                                          succ_pessimistic, ii);
    const int succ_pessimistic_id = state_hasher_.GetStateIDForceful(
                                      succ_pessimistic);

    const vector<int> succ_states{succ_optimistic_id, succ_pessimistic_id};
    const vector<double> succ_probabilities{prob_optimistic, prob_pessimistic};
    const vector<double> succ_costs{cost_optimistic, cost_pessimistic};

    succ_state_ids_map->push_back(succ_states);
    succ_state_probabilities_map->push_back(succ_probabilities);
    action_costs_map->push_back(succ_costs);
    // cout << "Succs for " << parent_state.to_string() << endl
    //  << succ_optimistic.to_string() << endl
    //  << succ_pessimistic.to_string() << endl;
  }
}

void EdgeSelectorSSP::PrintPathsAsDOTGraph(const string &filename) const {
  std::ofstream dot_file(filename.c_str());
  dot_file << "digraph D {\n"
           << "  rankdir=LR\n"
           << "  size=\"4,3\"\n"
           << "  ratio=\"fill\"\n"
           << "  edge[style=\"bold\", labelfontcolor=\"black\"]\n"
           << "  node[shape=\"circle\", style=\"filled\", fillcolor=\"green\"]\n";

  int start_id = paths_[0].state_ids[0];
  int goal_id = paths_[0].state_ids.back();

  for (size_t ii = 0; ii < simplified_paths_.size(); ++ii) {
    const auto &path = simplified_paths_[ii];
    const int path_cost = paths_[ii].cost;
    bool first_edge = true;
    string parent_string("S"), succ_string;

    for (int edge_id : path) {
      const Edge edge = edge_hasher_.GetState(edge_id);

      succ_string = std::to_string(edge_id);

      std::stringstream edge_prob;
      edge_prob << fixed << setprecision(2) << edge.probability;
      dot_file << succ_string
               << "[xlabel=<<font color=\"red\">" << edge_prob.str() << " </font>>]\n";

      dot_file << parent_string << " -> " << succ_string
               << "[label=\"" << " "  << "\"";

      if (parent_string.compare("S") != 0) {
        dot_file << ", style=\"dashed\"";
      } else {
        dot_file << ", label=\"" << std::to_string(path_cost) << "\"";
      }

      dot_file << "]\n";
      parent_string = succ_string;
    }

    // Goal node.
    if (!succ_string.empty()) {
      dot_file << succ_string << " -> " << "G"
               << "[label=\"" << "" << "\""
               // << ",color=\"" << "red" << "\""
               << "]\n";
    } else {
      dot_file << "S->G"
               << "[label=\"" << std::to_string(path_cost) << "\"]\n";
    }
  }

  dot_file << "S[fillcolor=\"red\"]\n";
  dot_file << "G[fillcolor=\"red\"]\n";

  dot_file << "}";

}
} // namespace

std::ostream &operator<< (std::ostream &stream,
                          const sbpl::SSPState &ssp_state) {
  // stream << "Valid Bits:   " << ssp_state.valid_bits << endl;
  // stream << "Invalid Bits: " << ssp_state.invalid_bits << endl;
  stream << "Edge Status:   " << ssp_state.to_string() << endl;
  stream << "Subopt Bound: " << ssp_state.suboptimality_bound;
  return stream;
}
