#include <esp_planner/esp_environment.h>

using namespace std;

namespace {
  // Tolerance for comparing double numbers.
  constexpr double kDblTolerance = 1e-4;
} // namespace

EnvWrapper::EnvWrapper(EnvironmentESP* env_esp) {
  environment_esp_ = env_esp;
}

void EnvWrapper::GetSuccs(int parent_id, std::vector<int>* succ_ids, std::vector<int> *costs, std::vector<double>* edge_probabilities, std::vector<double>* edge_eval_times, std::vector<int>* edge_groups) {

  const auto &parent_wrapper = wrapper_state_hasher_.GetState(parent_id);
  set<int> parent_lazy_edges = parent_wrapper.lazy_edges;
  const int orig_parent_id = WrapperToStateID(parent_id);
  vector<int> orig_succ_ids;
  vector<int> orig_costs;

  environment_esp_->GetSuccs(orig_parent_id, &orig_succ_ids, costs, edge_probabilities, edge_eval_times, edge_groups);

  succ_ids->resize(orig_succ_ids.size());

  for (size_t ii = 0; ii < orig_succ_ids.size(); ++ii) {
    auto succ_lazy_edges = parent_lazy_edges;

    // We have a stochastic edge.
    if (edge_probabilities->at(ii) < 1.0 - kDblTolerance) {
      Edge edge(orig_parent_id, orig_succ_ids[ii], edge_probabilities->at(ii), edge_eval_times->at(ii));
      // TODO: check self-consistency.
      // TODO: use edge_group info here if available.
      const int edge_id = edge_hasher_.GetStateIDForceful(edge);
      succ_lazy_edges.insert(edge_id);
    } else {
      edge_probabilities->at(ii) = 1.0;
    }

    const int wrapper_succ_id = GetWrapperStateID(orig_succ_ids[ii], succ_lazy_edges);
    succ_ids->at(ii) = wrapper_succ_id;
  }
}

void EnvWrapper::GetPreds(int parent_id, std::vector<int>* succ_ids, std::vector<int> *costs, std::vector<double>* edge_probabilities, std::vector<double> *edge_eval_times, std::vector<int>* edge_groups /*= nullptr*/) {
      throw std::runtime_error("GetPreds has not been implemented for this environment");
}

int EnvWrapper::GetGoalHeuristic(int state_id) {
  const int orig_id = WrapperToStateID(state_id);
  return environment_esp_->GetGoalHeuristic(orig_id);
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

int EnvWrapper::GetWrapperStateID(int state_id, const std::set<int> &lazy_edges /*= empty set*/) {
  WrapperState wrapper(state_id, lazy_edges);
  int wrapper_state_id = 0;
  if (wrapper_state_hasher_.Exists(wrapper_state_id)) {
    wrapper_state_id = wrapper_state_hasher_.GetStateID(wrapper);
  } else {
    wrapper_state_id = wrapper_state_hasher_.GetStateIDForceful(wrapper);
    wrapper_to_state_id_[wrapper_state_id] = state_id;
  }
  return wrapper_state_id;
}
