#pragma once

#include <string>
#include <unordered_map>

#include <esp_planner/esp_environment.h>
#include <sbpl_utils/environments/boost_graph_environment.h>

using sbpl_utils::StochasticGraph;
using sbpl_utils::BGEnvironment;

namespace sbpl {
class EdgeSelectorSSP;

void PrintPolicyAndPathsAsDOTGraph(const std::string &name,
                                   const std::string &dir,
                                   const std::unordered_map<int, int> &policy_map, const EdgeSelectorSSP &ssp);

// A boost graph environment for testing the ESP planner.
class EnvBGStochastic final : public EnvironmentESP,
  public BGEnvironment<StochasticGraph> {
 public:
  EnvBGStochastic(const StochasticGraph &graph);
  void GetSuccs(int parent_id, std::vector<int> *succ_ids,
                std::vector<int> *costs, std::vector<double> *edge_probabilities,
                std::vector<double> *edge_eval_times,
                std::vector<int> *edge_groups = nullptr) override;

  void GetSuccs(int parent_id, std::vector<int> *succ_ids,
                std::vector<int> *costs) override {
    std::vector<double> edge_probabilities, edge_eval_times;
    GetSuccs(parent_id, succ_ids, costs, &edge_probabilities, &edge_eval_times);
  }

  int GetTrueCost(int parent_id, int child_id) override;
  int GetGoalHeuristic(int state_id) override;

  // Unused methods. These need to be here again, to avoid the ambiguity in
  // overrides.
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
 private:
  decltype(get(bo::edge_bundle, graph_)) edge_bundle_map_;
};
}
