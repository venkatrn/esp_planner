
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

#include <sbpl/headers.h>
#include <sbpl/planners/mha_planner.h>
#include <queue>
#include <memory>

#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>


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

  // Planner for computing optimal edge evaluation policy.
  virtual int GetTruePathIdx(const std::vector<sbpl::Path> &paths);
  // Evaluate all lazy edges in all paths, and return the idx of the best valid
  // path.
  virtual int GetTruePathIdxAllEdges(const std::vector<sbpl::Path> &paths);
  

  // Helper for computing and executing edge evaluation policy. Returns -1 if
  // no valid path was found, otherwise returns the wrapper state ID
  // corresponding to the valid path. Output goal_wrapper_id is the wrapper
  // id/path id that was evaluated to be true, and -1 if none of the paths were valid.
  int RunEvaluation(int* goal_wrapper_id);
  std::vector<sbpl::Path> GetCurrentSolutionPaths(std::vector<int>* path_ids);
  std::unordered_set<int> evaluated_goal_wrapper_ids_;
  std::unordered_set<sbpl::Edge> invalid_edges_;
  std::unordered_set<sbpl::Edge> valid_edges_;

 protected:
  //data structures (open and incons lists)
  std::vector<CHeap> heaps;
  std::vector<std::vector<ESPState*>> incons;
  std::vector<std::vector<ESPState*>> states;
  std::vector<std::vector<BestHState*>> best_h_states;

  //params
  ReplanParams params;
  bool bforwardsearch; //if true, then search proceeds forward, otherwise backward
  ESPState *goal_state;
  ESPState *start_state;
  int goal_state_id;
  int start_state_id;

  std::vector<int> goal_wrapper_ids_;
  std::vector<std::set<int>> goal_subsets_;

  // Environment.
  std::unique_ptr<EnvWrapper> environment_wrapper_; 

  //mha params
  int num_heuristics;
  int expands;
  double inflation_eps, anchor_eps;
  bool use_anchor;
  mha_planner::PlannerType planner_type;
  mha_planner::MetaSearchType meta_search_type;
  mha_planner::MHAType mha_type;

  // meta-A* variables
  std::vector<int> queue_expands;
  std::vector<int> queue_best_h_dts;
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
  clock_t TimeStarted;
  short unsigned int search_iteration;
  short unsigned int replan_number;
  bool use_repair_time;

  //stats
  std::vector<PlannerStats> stats;
  unsigned int totalExpands;
  double totalTime;
  double totalPlanTime;
  double reconstructTime;

  bool interruptFlag;

  virtual ESPState *GetState(int q_id, int id);
  virtual BestHState* GetBestHState(int q_id, int id);
  virtual void ExpandState(int q_id, ESPState *parent);
  void putStateInHeap(int q_id, ESPState *state);

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
