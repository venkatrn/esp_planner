#pragma once

#include <esp_planner/esp_structs.h>
#include <sbpl/headers.h>
#include <sbpl_utils/hash_manager/hash_manager.h>

#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <set>
#include <unordered_map>
#include <vector>

using sbpl::Edge;

class EnvironmentESP : public virtual DiscreteSpaceInformation {
 public:
  virtual void GetSuccs(int parent_id, std::vector<int> *succ_ids,
                        std::vector<int> *costs, std::vector<double> *edge_probabilites,
                        std::vector<double> *edge_eval_times,
                        std::vector<int> *edge_groups = nullptr) = 0;
  virtual int GetGoalHeuristic(int state_id) override = 0;
  virtual bool EvaluateEdge(int parent_id, int child_id) = 0;
  virtual void GetPreds(int parent_id, std::vector<int> *succ_ids,
                        std::vector<int> *costs, std::vector<double> *edge_probabilites,
                        std::vector<double> *edge_eval_times,
                        std::vector<int> *edge_groups = nullptr) {
    throw std::runtime_error("GetPreds has not been implemented for this environment");
  }

  // Unused methods.
  virtual bool InitializeEnv(const char *) override {return true;};
  virtual bool InitializeMDPCfg(MDPConfig *) override {return true;};
  virtual int GetFromToHeuristic(int, int) override {
    return 0;
  }
  virtual int GetStartHeuristic(int) override {
    return 0;
  }
  virtual void GetPreds(int, std::vector<int> *, std::vector<int> *) override {}
  virtual void SetAllActionsandAllOutcomes(CMDPSTATE *) override {}
  virtual void SetAllPreds(CMDPSTATE *) override {}
  virtual int SizeofCreatedEnv() override {
    return 0;
  }
  virtual void PrintState(int, bool, FILE *) override {};
  virtual void PrintEnv_Config(FILE *) override {};
};

class WrapperState;
std::ostream &operator<< (std::ostream &stream, const WrapperState &state);

class WrapperState {
 public:
  explicit WrapperState(int state_id, double log_prob,
                        const std::set<int> &lazy_edges) : env_state_id(state_id), log_prob(log_prob),
    lazy_edges(lazy_edges) {}

  // NOTE: This is the environment state ID, not the ID of the wrapped state!
  int env_state_id = -1;
  // A collection of edges (or edge sets) that have been traversed by this
  // path.
  std::set<int> lazy_edges;
  // The log probability of this path.
  double log_prob = std::numeric_limits<double>::max();

  size_t GetHash() const {
    return env_state_id;
  }
  bool operator==(const WrapperState &other) const {
    // Two wrapper states are definitely different if they have different
    // underlying original states.
    if (env_state_id != other.env_state_id) {
      return false;
    }

    return lazy_edges == other.lazy_edges;
  }
  bool operator!=(const WrapperState &other) const {
    return !(*this == other);
  }
};

// TODO: could derive from DiscreteSpaceInformation, but that inherits a ton of
// unused code.
class EnvWrapper {
 public:
  EnvWrapper(EnvironmentESP *env_esp);
  // Wrapper for environment methods.
  void GetSuccs(int parent_id, std::vector<int> *succ_ids,
                std::vector<int> *costs, std::vector<double> *edge_probabilities,
                std::vector<double> *edge_eval_times, std::vector<int> *edge_groups = nullptr);
  void GetPreds(int parent_id, std::vector<int> *succ_ids,
                std::vector<int> *costs, std::vector<double> *edge_probabilities,
                std::vector<double> *edge_eval_times, std::vector<int> *edge_groups = nullptr);
  virtual bool EvaluateOriginalEdge(int parent_id, int child_id);
  int GetGoalHeuristic(int state_id);
  int GetStartHeuristic(int state_id);
  double GetStateProbability(int state_id);
  void EnsureHeuristicsUpdated(bool forward_search);
  std::vector<int> GetAllGoalWrapperIDs(int goal_wrapper_id) const;
  sbpl::Path ConvertWrapperIDsPathToSBPLPath(const std::vector<int>
                                             &wrapper_ids_path);

  // ID conversion utils.
  // The mapping from state IDs to wrapper IDs is one-to-many and injective (i.e, every wrapper ID corresponds to a unique state ID but not
  // vice versa).
  std::vector<int> StateToWrapperIDs(int env_state_id) const;
  // Return all the wrapper IDs that all have the same underlying env_state_id as
  // that of the state with wrapper_id.
  std::vector<int> AllEquivalentWrapperIDs(int wrapper_state_id) const;
  std::vector<int> AllSubsetWrapperIDs(int wrapper_state_id) const;
  int WrapperToStateID(int wrapper_id) const;
  // Returns ID of the newly created state, or the existing state.
  int GetWrapperStateID(int state_id, double prob,
                        const std::set<int> &lazy_edges = std::set<int>());
  friend class ESPPlanner;

 private:
  EnvironmentESP *environment_esp_;
  sbpl_utils::HashManager<WrapperState> wrapper_state_hasher_;
  sbpl_utils::HashManager<Edge> edge_hasher_;
  std::unordered_map<int, int> wrapper_to_state_id_;


  // Caches.
  std::unordered_map<int, std::vector<int>> succs_cache_;
  std::unordered_map<int, std::vector<int>> costs_cache_;
  std::unordered_map<int, std::vector<double>> probs_cache_;
  std::unordered_map<int, std::vector<double>> time_cache_;
};
