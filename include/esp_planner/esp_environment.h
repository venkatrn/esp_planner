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
  virtual int GetTrueCost(int parent_id, int child_id) override = 0;
  virtual void GetPreds(int parent_id, std::vector<int> *succ_ids,
                        std::vector<int> *costs, std::vector<double> *edge_probabilites,
                        std::vector<double> *edge_eval_times,
                        std::vector<int> *edge_groups = nullptr) {
    throw std::runtime_error("GetPreds has not been implemented for this environment");
  }

  // Unused methods.
  virtual bool InitializeEnv(const char *) override {};
  virtual bool InitializeMDPCfg(MDPConfig *) override {};
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
  explicit WrapperState(int state_id, double prob,
                        const std::set<int> &lazy_edges) : env_state_id(state_id), prob(prob),
    lazy_edges(lazy_edges) {}
  // NOTE: This is the environment state ID, not the ID of the wrapped state!
  int env_state_id = -1;
  // A collection of edges (or edge sets) that have been traversed by this
  // path.
  std::set<int> lazy_edges;
  // The probability of this path.
  // TODO: use log probability.
  double prob = -1.0;

  // Timestamp at which this state was created.
  double creation_time = -1.0;

  size_t GetHash() const {
    return env_state_id;
  }
  bool operator==(const WrapperState &other) const {
    // Two wrapper states are definitely different if they have different
    // underlying original states.
    if (env_state_id != other.env_state_id) {
      return false;
    }
    // bool print = false;
    // if (env_state_id == 5 && lazy_edges.empty()) {
    //   std::cout << *this << std::endl;
    //   std::cout << "Other\n";
    //   std::cout << other << std::endl << std::endl;
    //   print = true;
    // }
    // if (other.env_state_id == 5 && other.lazy_edges.empty()) {
    //   std::cout << other << std::endl;
    //   std::cout << "Other\n";
    //   std::cout << *this << std::endl << std::endl;
    //   print = true;
    // }
    
    // We assume that duplicates of states get generated in order of increasing
    // g-values, i.e, if s2 = (x, [abc]) is generated after s1 = (x,[ab]), then
    // s1 and s2 are the same. Note that this handles the empty lazy_edges set
    // (deterministic) case correctly as well.
    // TODO: delegate the pruning part to GetSuccs and just check if two sets
    // are equal here.
    bool equal = true;
    if (creation_time < other.creation_time) {
      const bool subset = std::includes(other.lazy_edges.begin(), other.lazy_edges.end(), lazy_edges.begin(), lazy_edges.end());
      if (subset && !other.lazy_edges.empty() && lazy_edges.empty() && prob < 0) {
        equal = false;
      } else {
      equal = subset;
      }
    } else {
      const bool subset = std::includes(lazy_edges.begin(), lazy_edges.end(), other.lazy_edges.begin(), other.lazy_edges.end());
      if (subset && !lazy_edges.empty() && other.lazy_edges.empty() && other.prob < 0) {
        equal = false;
      } else {
        equal = subset;
      }
    }
    // if (print) {
    //   std::cout << "EQUAL: " << equal << std::endl;
    //   const bool subset = std::includes(other.lazy_edges.begin(), other.lazy_edges.end(), lazy_edges.begin(), lazy_edges.end());
    //   std::cout << subset << std::endl;
    //   std::cout << !other.lazy_edges.empty() << std::endl;
    //   std::cout << lazy_edges.empty() << std::endl;
    //   std::cout << (prob < 1e-3) << std::endl;
    // }
    return equal;
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
  int GetGoalHeuristic(int state_id);
  int GetStartHeuristic(int state_id);
  void EnsureHeuristicsUpdated(bool forward_search);
  std::vector<int> GetAllGoalWrapperIDs(int goal_wrapper_id);
  sbpl::Path ConvertWrapperIDsPathToSBPLPath(const std::vector<int>& wrapper_ids_path);

  // ID conversion utils.
  // The mapping from state IDs to wrapper IDs is one-to-many and injective (i.e, every wrapper ID corresponds to a unique state ID but not
  // vice versa).
  std::vector<int> StateToWrapperIDs(int env_state_id) const;
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
