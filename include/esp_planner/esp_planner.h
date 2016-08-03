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
#include <queue>
#include <memory>

class ESPState: public AbstractSearchState {
 public:
  int id;
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

  ESPPlanner(EnvironmentESP *environment, bool bforwardsearch);
  ~ESPPlanner() {};

  virtual void get_search_stats(std::vector<PlannerStats> *s);

 protected:
  //data structures (open and incons lists)
  CHeap heap;
  std::vector<ESPState *> incons;
  std::vector<ESPState *> states;

  //params
  ReplanParams params;
  bool bforwardsearch; //if true, then search proceeds forward, otherwise backward
  ESPState *goal_state;
  ESPState *start_state;
  int goal_state_id;
  int start_state_id;

  // Environment.
  std::unique_ptr<EnvWrapper> environment_wrapper_; 

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

  virtual ESPState *GetState(int id);
  virtual void ExpandState(ESPState *parent);
  void putStateInHeap(ESPState *state);

  virtual int ImprovePath();

  virtual std::vector<int> GetSearchPath(int state_id, int &solcost);

  virtual bool outOfTime();
  virtual void initializeSearch();
  virtual void prepareNextSearchIteration();
  virtual bool Search(std::vector<int> &pathIds, int &PathCost);

};
