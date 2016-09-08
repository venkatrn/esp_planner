#include <esp_planner/esp_environment.h>

using namespace std;

namespace {
// Tolerance for comparing double numbers.
constexpr double kDblTolerance = 1e-4;
// We will cache successors from the first time so that we get the same set of
// successors during path reconstruction as well---which might not happen if
// the environment caches edge evaluations and returns a different successor
// set at a later time.
constexpr bool kUseCaching = true;
} // namespace

EnvWrapper::EnvWrapper(EnvironmentESP *env_esp) {
  environment_esp_ = env_esp;
}

bool EnvWrapper::EvaluateOriginalEdge(int parent_id, int child_id) {
  return environment_esp_->EvaluateEdge(parent_id, child_id);
}

void EnvWrapper::GetSuccs(int parent_id, std::vector<int> *succ_ids,
                          std::vector<int> *costs, std::vector<double> *edge_probabilities,
                          std::vector<double> *edge_eval_times, std::vector<int> *edge_groups) {

  if (kUseCaching) {
    if (succs_cache_.find(parent_id) != succs_cache_.end()) {
      *succ_ids = succs_cache_[parent_id];
      *costs = costs_cache_[parent_id];
      *edge_probabilities = probs_cache_[parent_id];
      *edge_eval_times = time_cache_[parent_id];
      auto it = edge_groups_cache_.find(parent_id);

      if (it != edge_groups_cache_.end()) {
        *edge_groups = it->second;
      }

      return;
    }
  }

  const auto &parent_wrapper = wrapper_state_hasher_.GetState(parent_id);
  const set<int> &parent_lazy_edges = parent_wrapper.lazy_edges;
  const set<int> &parent_all_stochastic_edges =
    parent_wrapper.all_stochastic_edges;
  const int orig_parent_id = WrapperToStateID(parent_id);
  vector<int> orig_succ_ids;
  vector<int> orig_costs;
  vector<double> orig_edge_probabilities;
  vector<double> orig_edge_eval_times;
  vector<int> orig_edge_groups;

  environment_esp_->GetSuccs(orig_parent_id, &orig_succ_ids, &orig_costs,
                             &orig_edge_probabilities, &orig_edge_eval_times, &orig_edge_groups);

  succ_ids->clear();
  costs->clear();
  edge_probabilities->clear();
  edge_eval_times->clear();
  edge_groups->clear();

  succ_ids->reserve(orig_succ_ids.size());
  costs->reserve(orig_succ_ids.size());
  edge_probabilities->reserve(orig_succ_ids.size());
  edge_eval_times->reserve(orig_succ_ids.size());
  edge_groups->reserve(orig_succ_ids.size());

  for (size_t ii = 0; ii < orig_succ_ids.size(); ++ii) {
    if (orig_edge_probabilities[ii] < kDblTolerance) {
      // Skip edges that have zero probability.
      continue;
    }

    auto succ_lazy_edges = parent_lazy_edges;
    auto succ_all_stochastic_edges = parent_all_stochastic_edges;

    // TODO: we could hash only stochastic edges. Would need to update
    // ConvertWrapperIDsToSBPLPath accordingly.
    Edge edge(orig_parent_id, orig_succ_ids[ii], orig_edge_probabilities[ii],
              orig_edge_eval_times[ii]);
    const int edge_id = edge_hasher_.GetStateIDForceful(edge);
    // We have a stochastic edge.
    double wrapper_state_log_prob = parent_wrapper.log_prob;

    if (orig_edge_probabilities[ii] < 1.0 - kDblTolerance) {
      succ_all_stochastic_edges.insert(edge_id);

      // TODO: check self-consistency.
      // TODO: use edge_group info here if available.
      if (orig_edge_groups.empty()) {
        succ_lazy_edges.insert(edge_id);
        wrapper_state_log_prob += log(orig_edge_probabilities[ii]);
      } else {
        auto insert_code = succ_lazy_edges.insert(orig_edge_groups[ii]);
        edge_groups->push_back(orig_edge_groups[ii]);

        // Multiply probability only if this is the first time we pass through
        // this edge group.
        if (insert_code.second) {
          wrapper_state_log_prob += log(orig_edge_probabilities[ii]);
        }
      }

      edge_probabilities->push_back(orig_edge_probabilities[ii]);
    } else {
      edge_probabilities->push_back(1.0);
    }

    edge_eval_times->push_back(orig_edge_eval_times[ii]);
    costs->push_back(orig_costs[ii]);

    if (edge_groups != nullptr && !edge_groups->empty()) {
      edge_to_group_id_mapping_[edge] = orig_edge_groups[ii];
    }


    const int wrapper_succ_id = GetWrapperStateID(orig_succ_ids[ii],
                                                  wrapper_state_log_prob,
                                                  succ_lazy_edges,
                                                  succ_all_stochastic_edges);
    succ_ids->push_back(wrapper_succ_id);

    orig_state_to_wrapper_ids_mapping_[orig_succ_ids[ii]].insert(
      wrapper_succ_id);

    if (orig_succ_ids[ii] == original_goal_id_) {
      goal_wrapper_ids_.insert(wrapper_succ_id);
    }
  }

  // DEBUG
  // if (succs_cache_.find(parent_id) != succs_cache_.end()) {
  //   const auto& cached_succ_ids = succs_cache_[parent_id];
  // printf("Cached succs for %d\n", parent_id);
  // for (int ii = 0; ii < cached_succ_ids.size(); ++ii) {
  //   printf("%d, ", cached_succ_ids[ii]);
  // }
  // printf("\n");
  // printf("Regenerated succs for %d\n", parent_id);
  // for (int ii = 0; ii < succ_ids->size(); ++ii) {
  //   printf("%d, ", succ_ids->at(ii));
  // }
  // printf("\n");

  // for (int ii = 0; ii < cached_succ_ids.size(); ++ii) {
  //     if (cached_succ_ids[ii] != succ_ids->at(ii)) {
  //       printf("No Match: \n");
  //       cout << wrapper_state_hasher_.GetState(cached_succ_ids[ii]) << endl;
  //       cout << wrapper_state_hasher_.GetState(succ_ids->at(ii)) << endl;
  //       cout << orig_succ_ids[ii] << endl;
  //       cout << (wrapper_state_hasher_.GetState(cached_succ_ids[ii]) == wrapper_state_hasher_.GetState(succ_ids->at(ii))) << endl;
  //     }
  // }

  // const auto& cached_costs = costs_cache_[parent_id];
  // printf("Cached costs for %d\n", parent_id);
  // for (int ii = 0; ii < cached_costs.size(); ++ii) {
  //   printf("%d, ", cached_costs[ii]);
  // }
  // printf("\n");
  // printf("Regenerated costs for %d\n", parent_id);
  // for (int ii = 0; ii < costs->size(); ++ii) {
  //   printf("%d, ", costs->at(ii));
  // }
  // printf("\n");
  // }

  if (kUseCaching) {
    succs_cache_[parent_id] = *succ_ids;
    costs_cache_[parent_id] = *costs;
    probs_cache_[parent_id] = *edge_probabilities;
    time_cache_[parent_id] = *edge_eval_times;

    if (edge_groups != nullptr && !edge_groups->empty()) {
      edge_groups_cache_[parent_id] = *edge_groups;
    }
  }
}

void EnvWrapper::GetPreds(int parent_id, std::vector<int> *succ_ids,
                          std::vector<int> *costs, std::vector<double> *edge_probabilities,
                          std::vector<double> *edge_eval_times,
                          std::vector<int> *edge_groups /*= nullptr*/) {
  throw std::runtime_error("GetPreds has not been implemented for this environment");
}

int EnvWrapper::GetGoalHeuristic(int state_id) {
  const int orig_id = WrapperToStateID(state_id);
  return environment_esp_->GetGoalHeuristic(orig_id);
}

int EnvWrapper::GetGoalHeuristic(int heuristic_id, int state_id) {
  const int orig_id = WrapperToStateID(state_id);
  return environment_esp_->GetGoalHeuristic(heuristic_id, orig_id);
}

double EnvWrapper::GetStateProbability(int state_id) {
  const auto &wrapper_state = wrapper_state_hasher_.GetState(state_id);
  return exp(wrapper_state.log_prob);
}

double EnvWrapper::GetStateNegLogProbability(int state_id) {
  const auto &wrapper_state = wrapper_state_hasher_.GetState(state_id);
  return -wrapper_state.log_prob;
}

int EnvWrapper::GetStartHeuristic(int state_id) {
  const int orig_id = WrapperToStateID(state_id);
  return environment_esp_->GetStartHeuristic(orig_id);
}

void EnvWrapper::EnsureHeuristicsUpdated(bool forward_search) {
  environment_esp_->EnsureHeuristicsUpdated(forward_search);
}

vector<int> EnvWrapper::StateToWrapperIDs(int env_state_id) const {
  // const auto &states = wrapper_state_hasher_.GetStateMappings();
  //
  // vector<int> wrapper_ids;
  //
  // for (const auto &state : states) {
  //   if (state.first.env_state_id == env_state_id) {
  //     wrapper_ids.push_back(state.second);
  //   }
  // }
  //
  // return wrapper_ids;
  auto mappings_it = orig_state_to_wrapper_ids_mapping_.find(env_state_id);

  if (mappings_it == orig_state_to_wrapper_ids_mapping_.end()) {
    return std::vector<int>();
  }

  return std::vector<int>(mappings_it->second.begin(),
                          mappings_it->second.end());
}

int EnvWrapper::WrapperToStateID(int wrapper_id) const {
  // This is guaranteed to exist.
  auto it = wrapper_to_state_id_.find(wrapper_id);

  if (it == wrapper_to_state_id_.end()) {
    std::stringstream ss;
    ss << "Wrapper ID " << wrapper_id << " not found in mapping" << endl;
    throw std::runtime_error(ss.str());
  }

  return it->second;
}

int EnvWrapper::GetWrapperStateID(int state_id, double prob,
                                  const std::set<int> &lazy_edges,
                                  const std::set<int> &all_stochastic_edges) {
  WrapperState wrapper(state_id, prob, lazy_edges, all_stochastic_edges);
  int wrapper_state_id = 0;

  if (wrapper_state_hasher_.Exists(wrapper)) {
    wrapper_state_id = wrapper_state_hasher_.GetStateID(wrapper);
    wrapper_state_hasher_.UpdateState(wrapper);
  } else {
    wrapper_state_id = wrapper_state_hasher_.GetStateIDForceful(wrapper);
    wrapper_to_state_id_[wrapper_state_id] = state_id;
  }

  return wrapper_state_id;
}

std::vector<int> EnvWrapper::GetAllGoalWrapperIDs(int goal_wrapper_id) const {
  // return AllEquivalentWrapperIDs(goal_wrapper_id);
  return std::vector<int>(goal_wrapper_ids_.begin(), goal_wrapper_ids_.end());
}

std::vector<int> EnvWrapper::AllEquivalentWrapperIDs(int wrapper_state_id)
const {
  int orig_state_id = WrapperToStateID(wrapper_state_id);
  vector<int> state_wrapper_ids = StateToWrapperIDs(orig_state_id);
  return state_wrapper_ids;
}

int EnvWrapper::GetUpdatedWrapperStateID(int wrapper_state_id, const std::set<int>& invalid_edge_groups, const std::set<int>& valid_edge_groups) {
  int orig_state_id = WrapperToStateID(wrapper_state_id);
  const auto &wrapper_state = wrapper_state_hasher_.GetState(
                                       wrapper_state_id);

    std::set<int> unevaluated_edges_in_equiv_temp;
    std::set<int> unevaluated_edges_in_equiv;
    std::set_difference(wrapper_state.lazy_edges.begin(), 
                        wrapper_state.lazy_edges.end(),
                        valid_edge_groups.begin(), 
                        valid_edge_groups.end(),
                        std::inserter(unevaluated_edges_in_equiv_temp, unevaluated_edges_in_equiv_temp.begin()));
    std::set_difference(unevaluated_edges_in_equiv_temp.begin(), 
                        unevaluated_edges_in_equiv_temp.end(),
                        invalid_edge_groups.begin(), 
                        invalid_edge_groups.end(),
                        std::inserter(unevaluated_edges_in_equiv, unevaluated_edges_in_equiv.begin()));
    if (unevaluated_edges_in_equiv == wrapper_state.lazy_edges) {
      return wrapper_state_id;
    } 
    // TODO: we need to remove evaluated edges from "all stochastic edges" as well.
    const int new_wrapper_state_id = GetWrapperStateID(wrapper_state.env_state_id,wrapper_state.log_prob, unevaluated_edges_in_equiv, wrapper_state.all_stochastic_edges);
    return new_wrapper_state_id;
}

std::vector<int> EnvWrapper::AllSubsetWrapperIDs(int wrapper_state_id, const std::set<int>& valid_edge_groups) const {
  int orig_state_id = WrapperToStateID(wrapper_state_id);
  const auto &source_wrapper_state = wrapper_state_hasher_.GetState(
                                       wrapper_state_id);
  vector<int> state_wrapper_ids = StateToWrapperIDs(orig_state_id);
  vector<int> subset_wrapper_ids;
  subset_wrapper_ids.reserve(state_wrapper_ids.size());

  for (size_t ii = 0; ii < state_wrapper_ids.size(); ++ii) {
    if (state_wrapper_ids[ii] == wrapper_state_id) {
      continue;
    }

    const auto &wrapper_state = wrapper_state_hasher_.GetState(
                                  state_wrapper_ids[ii]);
    std::set<int> unevaluated_edges_in_equiv;
    std::set_difference(wrapper_state.lazy_edges.begin(), 
                        wrapper_state.lazy_edges.end(),
                        valid_edge_groups.begin(), 
                        valid_edge_groups.end(),
                        std::inserter(unevaluated_edges_in_equiv, unevaluated_edges_in_equiv.begin()));

    const bool is_subset = std::includes(source_wrapper_state.lazy_edges.begin(),
                                         source_wrapper_state.lazy_edges.end(), unevaluated_edges_in_equiv.begin(),
                                         unevaluated_edges_in_equiv.end());

    // const bool is_subset = IsSubset(source_wrapper_state, wrapper_state);

    if (is_subset) {
      subset_wrapper_ids.push_back(state_wrapper_ids[ii]);
    }
  }

  return subset_wrapper_ids;
}

std::vector<int> EnvWrapper::AllSubsetWrapperIDs(int wrapper_state_id) const {
  return AllSubsetWrapperIDs(wrapper_state_id, std::set<int>());
}

bool EnvWrapper::IsSubset(const WrapperState &source_wrapper_state,
                          const WrapperState &wrapper_state) const {

  if (wrapper_state.lazy_edges.empty() &&
      source_wrapper_state.lazy_edges.empty()) {
    return true;
  }

  if (wrapper_state.lazy_edges.empty() &&
      !source_wrapper_state.lazy_edges.empty()) {
    return true;
  }

  if (!wrapper_state.lazy_edges.empty() &&
      source_wrapper_state.lazy_edges.empty()) {
    return false;
  }

  for (const auto &child_lazy_edge : wrapper_state.lazy_edges) {
    const auto &child_edge = edge_hasher_.GetState(child_lazy_edge);
    const std::pair<int, int> child_edge_pair = std::make_pair(child_edge.first,
                                                               child_edge.second);
    bool child_contained_in_source = false;

    for (const auto &source_lazy_edge : source_wrapper_state.lazy_edges) {
      const auto &source_edge = edge_hasher_.GetState(source_lazy_edge);
      const std::pair<int, int> source_edge_pair = std::make_pair(source_edge.first,
                                                                  source_edge.second);
      child_contained_in_source = environment_esp_->EdgesInSameGroup(child_edge_pair,
                                                                     source_edge_pair);

      if (child_contained_in_source) {
        break;
      }
    }

    if (!child_contained_in_source) {
      return false;
    }
  }

  return true;
}

bool EnvWrapper::WrapperContainsOriginalEdge(int wrapper_state_id,
                                             const sbpl::Edge &edge) {
  const auto &wrapper_state = wrapper_state_hasher_.GetState(wrapper_state_id);
  const int edge_id = edge_hasher_.GetStateID(edge);
  return (wrapper_state.all_stochastic_edges.find(edge_id) !=
          wrapper_state.all_stochastic_edges.end());
}

bool EnvWrapper::WrapperContainsInvalidEdge(int wrapper_state_id,
                                            const std::unordered_set<sbpl::Edge> &edges) {
  const auto &wrapper_state = wrapper_state_hasher_.GetState(wrapper_state_id);
  bool contains_invalid_edge = false;

  std::unordered_set<int> invalid_edge_groups;

  bool using_edge_groups = true;
  for (const auto &invalid_edge : edges) {
    auto it = edge_to_group_id_mapping_.find(invalid_edge);
    if (it != edge_to_group_id_mapping_.end()) {
      invalid_edge_groups.insert(edge_to_group_id_mapping_[invalid_edge]);
    } else {
      using_edge_groups = false;
      break;
    }
  }

  // Make this clear when defining edge groups.

  if (!using_edge_groups) {
    for (const auto &edge_id : wrapper_state.all_stochastic_edges) {
      const auto &edge = edge_hasher_.GetState(edge_id);

      if (edges.find(edge) != edges.end()) {
        contains_invalid_edge = true;
        break;
      }
    }
  } else {
    for (const auto &edge_group_id : wrapper_state.lazy_edges) {
      if (invalid_edge_groups.find(edge_group_id) != invalid_edge_groups.end()) {
        contains_invalid_edge = true;
        break;
      }
    }
  }

  return contains_invalid_edge;
}

bool EnvWrapper::WrapperContainsEdgeGroup(int wrapper_state_id,
                                            int invalid_edge_group_id) {
  const auto &wrapper_state = wrapper_state_hasher_.GetState(wrapper_state_id);
  return (wrapper_state.lazy_edges.find(invalid_edge_group_id) !=
          wrapper_state.lazy_edges.end());
}

sbpl::Path EnvWrapper::ConvertWrapperIDsPathToSBPLPath(const std::vector<int>
                                                       &wrapper_ids_path) {
  sbpl::Path path;
  const size_t num_states = wrapper_ids_path.size();
  path.state_ids.resize(num_states);
  path.edge_probabilities.resize(num_states - 1);
  path.edge_eval_times.resize(num_states - 1);

  std::unordered_set<int> edge_groups_in_path;

  for (size_t ii = 0; ii < num_states - 1; ++ii) {
    int s1_id = WrapperToStateID(wrapper_ids_path[ii]);
    int s2_id = WrapperToStateID(wrapper_ids_path[ii + 1]);
    sbpl::Edge temp_edge(s1_id, s2_id);
    const auto &edge = edge_hasher_.GetState(edge_hasher_.GetStateID(temp_edge));
    auto it = edge_to_group_id_mapping_.find(edge);
    const bool using_edge_groups = it != edge_to_group_id_mapping_.end();
    int edge_group_id = -1;
    bool group_already_in_path = false;

    if (using_edge_groups) {
      edge_group_id = it->second;
      group_already_in_path = edge_groups_in_path.find(edge_group_id) !=
                              edge_groups_in_path.end();
    }

    if (group_already_in_path) {
      path.edge_probabilities[ii] = 1.0;
    } else {
      path.edge_probabilities[ii] = edge.probability;

      if (using_edge_groups) {
        edge_groups_in_path.insert(edge_group_id);
      }
    }

    path.state_ids[ii] = s1_id;
    path.edge_eval_times[ii] = edge.evaluation_time;
  }

  // Add the final state to the path.
  path.state_ids[num_states - 1] = WrapperToStateID(wrapper_ids_path.back());
  return path;
}

std::ostream &operator<< (std::ostream &stream, const WrapperState &state) {
  stream << "(" << state.env_state_id << ", "  << exp(state.log_prob) << ")";
  stream << "\n";

  for (auto it = state.lazy_edges.begin(); it != state.lazy_edges.end(); ++it) {
    stream << *it << " ";
  }

  return stream;
}

void EnvWrapper::SetOriginalGoalID(int original_goal_id) {
  original_goal_id_ = original_goal_id;
  const int wrapper_goal_id = GetWrapperStateID(original_goal_id,
                                                std::numeric_limits<double>::lowest(), std::set<int>(), std::set<int>());
  goal_wrapper_ids_.insert(wrapper_goal_id);
  orig_state_to_wrapper_ids_mapping_[original_goal_id_].insert(wrapper_goal_id);
}
