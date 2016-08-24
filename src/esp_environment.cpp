#include <esp_planner/esp_environment.h>

using namespace std;

namespace {
// Tolerance for comparing double numbers.
constexpr double kDblTolerance = 1e-4;
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

  if (succs_cache_.find(parent_id) != succs_cache_.end()) {
    *succ_ids = succs_cache_[parent_id];
    *costs = costs_cache_[parent_id];
    *edge_probabilities = probs_cache_[parent_id];
    *edge_eval_times = time_cache_[parent_id];
    edge_groups->clear();
    return;
  }

  const auto &parent_wrapper = wrapper_state_hasher_.GetState(parent_id);
  set<int> parent_lazy_edges = parent_wrapper.lazy_edges;
  const int orig_parent_id = WrapperToStateID(parent_id);
  vector<int> orig_succ_ids;
  vector<int> orig_costs;

  environment_esp_->GetSuccs(orig_parent_id, &orig_succ_ids, costs,
                             edge_probabilities, edge_eval_times, edge_groups);

  succ_ids->resize(orig_succ_ids.size());

  for (size_t ii = 0; ii < orig_succ_ids.size(); ++ii) {
    auto succ_lazy_edges = parent_lazy_edges;

    // TODO: we could hash only stochastic edges. Would need to update
    // ConvertWrapperIDsToSBPLPath accordingly.
    Edge edge(orig_parent_id, orig_succ_ids[ii], edge_probabilities->at(ii),
              edge_eval_times->at(ii));
    const int edge_id = edge_hasher_.GetStateIDForceful(edge);
    // We have a stochastic edge.
    double wrapper_state_log_prob = parent_wrapper.log_prob;

    if (edge_probabilities->at(ii) < 1.0 - kDblTolerance) {
      // TODO: check self-consistency.
      // TODO: use edge_group info here if available.
      succ_lazy_edges.insert(edge_id);
      wrapper_state_log_prob += log(edge_probabilities->at(ii));
    } else {
      edge_probabilities->at(ii) = 1.0;
    }

    const int wrapper_succ_id = GetWrapperStateID(orig_succ_ids[ii],
                                                  wrapper_state_log_prob,
                                                  succ_lazy_edges);
    succ_ids->at(ii) = wrapper_succ_id;
  }

  succs_cache_[parent_id] = *succ_ids;
  costs_cache_[parent_id] = *costs;
  probs_cache_[parent_id] = *edge_probabilities;
  time_cache_[parent_id] = *edge_eval_times;
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

double EnvWrapper::GetStateProbability(int state_id) {
  const auto &wrapper_state = wrapper_state_hasher_.GetState(state_id);
  return exp(wrapper_state.log_prob);
}

int EnvWrapper::GetStartHeuristic(int state_id) {
  const int orig_id = WrapperToStateID(state_id);
  return environment_esp_->GetStartHeuristic(orig_id);
}

void EnvWrapper::EnsureHeuristicsUpdated(bool forward_search) {
  environment_esp_->EnsureHeuristicsUpdated(forward_search);
}

vector<int> EnvWrapper::StateToWrapperIDs(int env_state_id) const {
  const auto &states = wrapper_state_hasher_.GetStateMappings();

  vector<int> wrapper_ids;

  for (const auto &state : states) {
    if (state.first.env_state_id == env_state_id) {
      wrapper_ids.push_back(state.second);
    }
  }

  return wrapper_ids;
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
                                  const std::set<int> &lazy_edges /*= empty set*/) {
  WrapperState wrapper(state_id, prob, lazy_edges);
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
  return AllEquivalentWrapperIDs(goal_wrapper_id);
}

std::vector<int> EnvWrapper::AllEquivalentWrapperIDs(int wrapper_state_id)
const {
  int orig_state_id = WrapperToStateID(wrapper_state_id);
  vector<int> state_wrapper_ids = StateToWrapperIDs(orig_state_id);
  return state_wrapper_ids;
}

std::vector<int> EnvWrapper::AllSubsetWrapperIDs(int wrapper_state_id) const {
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
    const bool is_subset = std::includes(source_wrapper_state.lazy_edges.begin(),
                                         source_wrapper_state.lazy_edges.end(), wrapper_state.lazy_edges.begin(),
                                         wrapper_state.lazy_edges.end());

    if (is_subset) {
      subset_wrapper_ids.push_back(state_wrapper_ids[ii]);
    }
  }

  return subset_wrapper_ids;
}

bool EnvWrapper::WrapperContainsOriginalEdge(int wrapper_state_id, const sbpl::Edge& edge) {
  const auto &wrapper_state = wrapper_state_hasher_.GetState(wrapper_state_id);
  const int edge_id = edge_hasher_.GetStateID(edge);
  return (wrapper_state.lazy_edges.find(edge_id) != wrapper_state.lazy_edges.end());
}

sbpl::Path EnvWrapper::ConvertWrapperIDsPathToSBPLPath(const std::vector<int>
                                                       &wrapper_ids_path) {
  sbpl::Path path;
  const size_t num_states = wrapper_ids_path.size();
  path.state_ids.resize(num_states);
  path.edge_probabilities.resize(num_states - 1);
  path.edge_eval_times.resize(num_states - 1);

  for (size_t ii = 0; ii < num_states - 1; ++ii) {
    int s1_id = WrapperToStateID(wrapper_ids_path[ii]);
    int s2_id = WrapperToStateID(wrapper_ids_path[ii + 1]);
    sbpl::Edge temp_edge(s1_id, s2_id);
    const auto &edge = edge_hasher_.GetState(edge_hasher_.GetStateID(temp_edge));
    path.state_ids[ii] = s1_id;
    path.edge_probabilities[ii] = edge.probability;
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
