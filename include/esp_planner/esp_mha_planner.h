
/*
 * Copyright (c) 2016, Venkatraman Narayanan and Maxim Likhachev
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <esp_planner/esp_environment.h>
#include <esp_planner/esp_structs.h>

// #include <sbpl/headers.h>
#include <sbpl/planners/mha_planner.h>
#include <queue>
#include <memory>
#include <unordered_map>
#include <set>

#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>

#include <boost/functional/hash.hpp>

#include <chrono>

using high_res_clock = std::chrono::high_resolution_clock;

// Need this for the edge_set_heaps_, where indexing for the map is done using
// the set of lazy edges.
namespace std {
template <>
struct hash<std::set<int>> {
  std::size_t operator()(const std::set<int>& int_set) const {
    return boost::hash_value(int_set);
  }
};
} // namespace std

class ESPState: public AbstractSearchState {
 public:
  int id;
  int q_id;
  unsigned int v;
  unsigned int g;
  int h;
  short unsigned int iteration_closed;
  short unsigned int replan_number;
  ESPState *best_parent;
  ESPState *expanded_best_parent;
  bool in_incons;
};

class EdgeSetState: public AbstractSearchState {
 public:
  int id;
};

class ESPPlanner : public SBPLPlanner {

 public:
  virtual int replan(double allocated_time_secs,
                     std::vector<int> *solution_stateIDs_V) {
    printf("Not supported. Use ReplanParams");
    return -1;
  };
  virtual int replan(double allocated_time_sec,
                     std::vector<int> *solution_stateIDs_V, int *solcost) {
    printf("Not supported. Use ReplanParams");
    return -1;
  };

  virtual int replan(int start, int goal, std::vector<int> *solution_stateIDs_V,
                     ReplanParams params, int *solcost);
  virtual int replan(std::vector<int> *solution_stateIDs_V, ReplanParams params);
  virtual int replan(std::vector<int> *solution_stateIDs_V, ReplanParams params,
                     int *solcost);
  virtual int replan(std::vector<sbpl::Path> *paths, ReplanParams params,
                     int *solcost);

  void interrupt();

  virtual int set_goal(int goal_stateID);
  virtual int set_start(int start_stateID);

  virtual void costs_changed(StateChangeQuery const &stateChange) {
    return;
  };
  virtual void costs_changed() {
    return;
  };

  virtual int force_planning_from_scratch() {
    return 1;
  };
  virtual int force_planning_from_scratch_and_free_memory() {
    return 1;
  };

  virtual int set_search_mode(bool bSearchUntilFirstSolution) {
    printf("Not supported. Use ReplanParams");
    return -1;
  };

  virtual void set_initialsolution_eps(double initialsolution_eps) {
    printf("Not supported. Use ReplanParams");
  };

  ESPPlanner(EnvironmentESP *environment, int num_heuristics, bool bforwardsearch);
  ~ESPPlanner();

  virtual void get_search_stats(std::vector<PlannerStats> *s);
  virtual void get_ee_stats(std::vector<PlannerStats> *s);

  // Planner for computing optimal edge evaluation policy. If partial paths is
  // true, we won't record any valid paths found as solutions. 
  virtual int GetTruePathIdx(const std::vector<sbpl::Path> &paths, bool partial_paths);
  // Overload assuming we are given full paths to goal. This is stateful and
  // will modify solution_paths_ and ee_stats whenever new paths or better
  // suboptimality bounds are discovered.
  virtual int GetTruePathIdx(const std::vector<sbpl::Path> &paths);
  // Evaluate all lazy edges in all paths, and return the idx of the best valid
  // path.
  virtual int GetTruePathIdxAllEdges(const std::vector<sbpl::Path> &paths);
  

  // Helper for computing and executing edge evaluation policy. Returns -1 if
  // no valid path was found, otherwise returns the wrapper state ID
  // corresponding to the valid path. Output goal_wrapper_id is the wrapper
  // id/path id that was evaluated to be true, and -1 if none of the paths were valid.
  // The partial_paths bool serves the same purpose as in GetTruePathIdx.
  int RunEvaluation(int* goal_wrapper_id, bool partial_paths);
  int RunEvaluation(int* goal_wrapper_id);
  std::vector<sbpl::Path> GetCurrentSolutionPaths(std::vector<int>* path_ids, bool partial_paths);
  std::vector<sbpl::Path> GetCurrentSolutionPaths(std::vector<int>* path_ids);
  void AddNewSolution(const sbpl::Path& path);

  // Return the best frontier states for each unique edge_set collection we
  // have on the complete frontier. E.g., if we have many states in OPEN that
  // have traversed edge sets a, b, ab, abc, etc, this will return the states
  // with the lowest f-value for each distinct edge set collection. This is
  // efficient since we have a separate heap for every distinct collection.
  std::vector<int> GetBestDistinctFrontierStateIDs();

  std::unordered_set<int> evaluated_goal_wrapper_ids_;
  std::unordered_set<sbpl::Edge> invalid_edges_;
  std::unordered_set<int> invalid_edge_groups_;
  std::unordered_set<sbpl::Edge> valid_edges_;
  std::unordered_set<int> valid_edge_groups_;

 protected:
  //data structures (open and incons lists)
  std::vector<CHeap> heaps;
  std::vector<std::vector<ESPState*>> incons;
  std::vector<std::vector<ESPState*>> states;
  std::vector<std::vector<BestHState*>> best_h_states;
  std::unordered_map<std::set<int>, std::vector<EdgeSetState*>> edge_set_states_;

  // Separate heap for every unique edge set.
  // NOTE: this won't contain any goal wrapper IDs.
  std::unordered_map<std::set<int>, CHeap> edge_set_heaps_;

  //params
  ReplanParams params;
  bool bforwardsearch; //if true, then search proceeds forward, otherwise backward
  ESPState *goal_state;
  ESPState *start_state;
  int goal_state_id;
  int start_state_id;

  std::vector<int> goal_wrapper_ids_;
  std::vector<std::set<int>> goal_subsets_;
  std::vector<sbpl::Path> solution_paths_;
  long int incumbent_minkey;

  // Environment.
  std::unique_ptr<EnvWrapper> environment_wrapper_; 

  //mha params
  int num_heuristics;
  int expands;
  double inflation_eps, anchor_eps;
  bool use_anchor;
  bool doing_optimal_search;
  mha_planner::PlannerType planner_type;
  mha_planner::MetaSearchType meta_search_type;
  mha_planner::MHAType mha_type;

  // meta-A* variables
  std::vector<int> queue_expands;
  std::vector<int> queue_best_h_dts;
  std::vector<int> queue_stuck_in_minima;
  std::vector<CHeap> queue_best_h_meta_heaps;
  int max_edge_cost;
  std::vector<int> max_heur_dec;

  // Dynamic Thompson Sampling variables
  std::vector<double> alpha;
  std::vector<double> beta;
  double betaC;
  const gsl_rng_type* gsl_rand_T;
  gsl_rng* gsl_rand;

  //search member variables
  double eps;
  double eps_satisfied;
  int search_expands;
  high_res_clock::time_point TimeStarted;
  high_res_clock::time_point TimeLastSolutionFound;
  high_res_clock::time_point TimeExpandsResumed;
  double total_expands_time_ = 0.0;
  double total_eval_time_ = 0.0;
  short unsigned int search_iteration;
  short unsigned int replan_number;
  bool use_repair_time;

  //stats
  std::vector<PlannerStats> stats;
  std::vector<PlannerStats> ee_stats;
  unsigned int totalExpands;
  double totalTime;
  double totalPlanTime;
  double reconstructTime;

  bool interruptFlag;

  virtual ESPState *GetState(int q_id, int id);
  virtual BestHState* GetBestHState(int q_id, int id);
  virtual EdgeSetState* GetEdgeSetState(ESPState* esp_state);
  virtual void ExpandState(int q_id, ESPState *parent);
  bool putStateInHeap(int q_id, ESPState *state);

  virtual int ImprovePath();
  void checkHeaps(std::string msg);

  virtual std::vector<int> GetSearchPath(ESPState* end_state, int &solcost);

  virtual bool outOfTime();
  virtual void initializeSearch();
  virtual void prepareNextSearchIteration();
  virtual bool Search(std::vector<int> &pathIds, int &PathCost);

  // MHA-specific
  virtual int GetBestHeuristicID();
  virtual bool UpdateGoal(ESPState* state);
};
