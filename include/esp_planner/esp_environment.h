#pragma once

#include <esp_planner/esp_structs.h>
#include <sbpl/headers.h>
#include <sbpl_utils/hash_manager/hash_manager.h>
#include <stdexcept>
#include <set>
#include <unordered_map>
#include <vector>

using sbpl::Edge;

class EnvironmentESP : public DiscreteSpaceInformation {
  public:
    virtual void GetSuccs(int parent_id, std::vector<int>* succ_ids, std::vector<int> *costs, std::vector<double>* edge_probabilites, std::vector<double>* edge_eval_times, std::vector<int>* edge_groups = nullptr) = 0;
    virtual void GetPreds(int parent_id, std::vector<int>* succ_ids, std::vector<int> *costs, std::vector<double>* edge_probabilites, std::vector<double>* edge_eval_times, std::vector<int>* edge_groups = nullptr) {
      throw std::runtime_error("GetPreds has not been implemented for this environment");
    }
};

class WrapperState {
  public:
    explicit WrapperState(int state_id, const std::set<int> &lazy_edges) : env_state_id(state_id), lazy_edges(lazy_edges) {}
    // NOTE: This is the environment state ID, not the ID of the wrapped state!
    int env_state_id = -1;
    // A collection of edges (or edge sets) that have been traversed by this
    // path.
    std::set<int> lazy_edges;
    // The probability of this path.
    // TODO: use log probability.
    double p = 0.0;

    size_t GetHash() const {
      return env_state_id;
    }
    bool operator==(const WrapperState& other) const {
      if(env_state_id != other.env_state_id) {
        return false;
      }
      // TODO: implement
      return true;
    }
    bool operator!=(const WrapperState& other) const {
      return !(*this == other);
    }
};

// TODO: could derive from DiscreteSpaceInformation, but that inherits a ton of
// unused code.
class EnvWrapper {
  public:
    EnvWrapper(EnvironmentESP* env_esp);
    // Wrapper for environment methods.
    void GetSuccs(int parent_id, std::vector<int>* succ_ids, std::vector<int> *costs, std::vector<double>* edge_probabilities, std::vector<double>* edge_eval_times, std::vector<int>* edge_groups = nullptr);
    void GetPreds(int parent_id, std::vector<int>* succ_ids, std::vector<int> *costs, std::vector<double>* edge_probabilities, std::vector<double>* edge_eval_times, std::vector<int>* edge_groups = nullptr);
    int GetGoalHeuristic(int state_id);
    int GetStartHeuristic(int state_id);
    void EnsureHeuristicsUpdated(bool forward_search);

    // ID conversion utils.
    // The mapping from state IDs to wrapper IDs is one-to-many and injective (i.e, every wrapper ID corresponds to a unique state ID but not
    // vice versa).
    std::vector<int> StateToWrapperIDs(int env_state_id) const;
    int WrapperToStateID(int wrapper_id) const;
    // Returns ID of the newly created state, or the existing state.
    int GetWrapperStateID(int state_id, const std::set<int>& lazy_edges = std::set<int>());

  private:
    EnvironmentESP* environment_esp_;
    sbpl_utils::HashManager<WrapperState> wrapper_state_hasher_;
    sbpl_utils::HashManager<Edge> edge_hasher_;
    std::unordered_map<int, int> wrapper_to_state_id_;
};
