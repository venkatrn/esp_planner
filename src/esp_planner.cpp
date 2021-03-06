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
 *     * Neither the name of the Carnegie Mellon University nor the names of its
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
#include <esp_planner/esp_planner.h>
#include <esp_planner/edge_selector_ssp.h>
#include <esp_planner/lao_planner.h>

namespace {
// Maximum time allowed for computing an LAO policy. Note that this time could
// be exceeded if it takes longer to compute an action for the start state.
constexpr double kMaxLAOPolicyPlanningTime = 0.01; // s
}

using namespace std;

ESPPlanner::ESPPlanner(EnvironmentESP *environment,
                       bool bSearchForward) :
  params(0.0) {
  bforwardsearch = bSearchForward;
  environment_wrapper_.reset(new EnvWrapper(environment));
  replan_number = 0;

  goal_state_id = -1;
  start_state_id = -1;
}

ESPState *ESPPlanner::GetState(int id) {
  //if this stateID is out of bounds of our state vector then grow the list
  if (id >= int(states.size())) {
    for (int i = states.size(); i <= id; i++) {
      states.push_back(NULL);
    }
  }

  //if we have never seen this state then create one
  if (states[id] == NULL) {
    states[id] = new ESPState();
    states[id]->id = id;
    states[id]->replan_number = -1;
  }

  //initialize the state if it hasn't been for this call to replan
  ESPState *s = states[id];

  if (s->replan_number != replan_number) {
    s->g = INFINITECOST;
    s->v = INFINITECOST;
    s->iteration_closed = -1;
    s->replan_number = replan_number;
    s->best_parent = NULL;
    s->expanded_best_parent = NULL;
    s->heapindex = 0;
    s->in_incons = false;

    //compute heuristics
    if (bforwardsearch) {
      // const double prob = environment_wrapper_->GetStateProbability(s->id);
      // s->h = static_cast<int>((2.0 - 1.0 * prob)  * 0.5 *
      //                         static_cast<double>(environment_wrapper_->GetGoalHeuristic(s->id)));
      s->h = environment_wrapper_->GetGoalHeuristic(s->id);
    } else {
      s->h = environment_wrapper_->GetStartHeuristic(s->id);
    }
  }

  return s;
}

void ESPPlanner::ExpandState(ESPState *parent) {
  bool print = false; //parent->id == 285566;

  if (print) {
    printf("expand %d\n", parent->id);
  }

  vector<int> children;
  vector<int> costs;
  vector<double> probabilities;
  vector<double> times;
  vector<int> edge_groups;

  if (bforwardsearch) {
    environment_wrapper_->GetSuccs(parent->id, &children, &costs, &probabilities,
                                   &times, &edge_groups);
  } else {
    environment_wrapper_->GetPreds(parent->id, &children, &costs, &probabilities,
                                   &times, &edge_groups);
  }


  //printf("expand %d\n",parent->id);
  //iterate through children of the parent
  for (int i = 0; i < (int)children.size(); i++) {
    //printf("  succ %d\n",children[i]);
    // Decide if this state should be pruned (happens when we the present path
    // traverses the same set of lazy edges (possibly more as well) as a
    // previously expanded path to this state which traversed those exact set
    // (and nothing more).
    vector<int> equiv_wrapper_ids = environment_wrapper_->AllSubsetWrapperIDs(
                                      children[i]);
    bool prune = false;

    for (const int equiv_wrapper_id : equiv_wrapper_ids) {
      ESPState *child = GetState(equiv_wrapper_id);

      if (child->iteration_closed == search_iteration) {
        prune = true;
        break;
      }
    }

    if (prune) {
      continue;
    }

    ESPState *child = GetState(children[i]);

    bool print = false; //state->id == 285566 || parent->id == 285566;

    if (print) {
      printf("child->id=%d child->g=%d parent->v=%d edgeCost=%d\n",
             child->id, child->g, parent->v, costs[i]);
    }

    if (child->v <= parent->v + costs[i]) {
      continue;
    } else if (child->g <= parent->v + costs[i]) {
      continue;
    } else {
      //the new guy is the cheapest
      child->g = parent->v + costs[i];
      child->best_parent = parent;
      //this function puts the state into the heap (or updates the position) if we haven't expanded
      //if we have expanded, it will put the state in the incons list (if we haven't already)
      putStateInHeap(child);
    }
  }
}


void ESPPlanner::putStateInHeap(ESPState *state) {
  bool print = false; //state->id == 285566;

  //we only allow one expansion per search iteration
  //so insert into heap if not closed yet
  if (state->iteration_closed != search_iteration) {
    if (print) {
      printf("put state in open\n");
    }

    CKey key;
    key.key[0] = state->g + int(eps * state->h);

    //if the state is already in the heap, just update its priority
    if (state->heapindex != 0) {
      heap.updateheap(state, key);
    } else { //otherwise add it to the heap
      heap.insertheap(state, key);
    }
  }
  //if the state has already been expanded once for this iteration
  //then add it to the incons list so we can keep track of states
  //that we know we have better costs for
  else if (!state->in_incons) {
    if (print) {
      printf("put state in incons\n");
    }

    incons.push_back(state);
    state->in_incons = true;
  }
}

//returns 1 if the solution is found, 0 if the solution does not exist and 2 if it ran out of time
int ESPPlanner::ImprovePath() {

  //expand states until done
  int expands = 0;
  CKey min_key = heap.getminkeyheap();

  int valid_path_idx = -1;

  while (!heap.emptyheap() &&
         min_key.key[0] < INFINITECOST &&
         (goal_state->g > min_key.key[0]) &&
         !outOfTime()) {

    //get the state
    ESPState *state = (ESPState *)heap.deleteminheap();

    if (state->v == state->g) {
      printf("ERROR: consistent state is being expanded\n");
      printf("id=%d v=%d g=%d\n",
             state->id, state->v, state->g);
      throw new SBPL_Exception();
    }

    //mark the state as expanded
    state->v = state->g;
    state->expanded_best_parent = state->best_parent;
    state->iteration_closed = search_iteration;
    //expand the state
    expands++;

    const auto &wrapper_state =
      environment_wrapper_->wrapper_state_hasher_.GetState(state->id);

    if (wrapper_state.env_state_id == goal_state_id) {
      goal_wrapper_ids_.push_back(state->id);
      goal_subsets_.push_back(wrapper_state.lazy_edges);
      // Trigger a policy execution.
      // TODO: this is experimental. We want to do this only when we have a at
      // least a few paths to work with.
      valid_path_idx = RunEvaluation();

      if (valid_path_idx != -1) {
        break;
      }
    } else {
      bool is_subset = false;

      // If the current state about to be expanded has a lazy edge set X,
      // and there is already a goal state (closed) whose lazy edge set is a
      // subset of X, then we can safely avoid expanding this state since it
      // will lead only to a worse path.
      for (const auto &goal_subset : goal_subsets_) {
        is_subset = std::includes(wrapper_state.lazy_edges.begin(),
                                  wrapper_state.lazy_edges.end(),
                                  goal_subset.begin(),
                                  goal_subset.end());

        if (is_subset) {
          break;
        }
      }

      bool is_feasible = true;

      // TODO: this is horrible. Either swap the check direction, or even
      // better remove states from OPEN that are known to be infeasible.
      for (const auto &invalid_edge : invalid_edges_) {
        if (environment_wrapper_->WrapperContainsOriginalEdge(state->id,
                                                              invalid_edge)) {
          is_feasible = false;
          // printf("Wrapper contains invalid edge!\n");
          break;

        }
      }

      if (!is_subset && is_feasible) {
        ExpandState(state);
      }
    }

    if (expands % 100000 == 0) {
      printf("expands so far=%u\n", expands);
    }

    //get the min key for the next iteration
    min_key = heap.getminkeyheap();
    //printf("min_key =%d\n",min_key.key[0]);
  }

  if (valid_path_idx != -1) {
    // Hack.
    auto temp_goal_state = GetState(valid_path_idx);
    *goal_state = *temp_goal_state;
    goal_state->id = goal_state_id;
  }

  search_expands += expands;

  if (goal_state->g == INFINITECOST && (heap.emptyheap() ||
                                        min_key.key[0] >= INFINITECOST)) {
    return 0;  //solution does not exists
  }

  if (!heap.emptyheap() && goal_state->g > min_key.key[0]) {
    return 2;  //search exited because it ran out of time
  }

  printf("search exited with a solution for eps=%.3f\n", eps);

  if (goal_state->g < goal_state->v) {
    goal_state->expanded_best_parent = goal_state->best_parent;
    goal_state->v = goal_state->g;
  }

  return 1;
}

vector<int> ESPPlanner::GetSearchPath(int state_id, int &solcost) {
  vector<int> SuccIDV;
  vector<int> CostV;
  vector<int> wholePathIds;
  vector<double> ProbV;
  vector<double> TimesV;
  vector<int> EdgeGroupsV;

  ESPState *state;
  ESPState *final_state;

  if (bforwardsearch) {
    state = GetState(state_id);
    final_state = start_state;
  } else {
    state = start_state;
    final_state = GetState(state_id);
  }

  wholePathIds.push_back(state->id);
  solcost = 0;

  while (state->id != final_state->id) {
    if (state->expanded_best_parent == NULL) {
      printf("a state along the path has no parent!\n");
      break;
    }

    if (state->v == INFINITECOST) {
      printf("a state along the path has an infinite g-value!\n");
      printf("inf state = %d\n", state->id);
      break;
    }

    if (bforwardsearch) {
      environment_wrapper_->GetSuccs(state->expanded_best_parent->id, &SuccIDV,
                                     &CostV, &ProbV, &TimesV, &EdgeGroupsV);
    } else {
      environment_wrapper_->GetPreds(state->expanded_best_parent->id, &SuccIDV,
                                     &CostV, &ProbV, &TimesV, &EdgeGroupsV);
    }

    int actioncost = INFINITECOST;

    //printf("reconstruct expand %d\n",state->expanded_best_parent->id);
    for (unsigned int i = 0; i < SuccIDV.size(); i++) {
      //printf("  succ %d\n",SuccIDV[i]);
      if (SuccIDV[i] == state->id && CostV[i] < actioncost) {
        actioncost = CostV[i];
      }
    }

    if (actioncost == INFINITECOST) {
      printf("WARNING: actioncost = %d\n", actioncost);
    }

    solcost += actioncost;

    state = state->expanded_best_parent;
    wholePathIds.push_back(state->id);
  }

  //if we searched forward then the path reconstruction
  //worked backward from the final state, so we have to reverse the path
  if (bforwardsearch) {
    //in place reverse
    for (unsigned int i = 0; i < wholePathIds.size() / 2; i++) {
      int other_idx = wholePathIds.size() - i - 1;
      int temp = wholePathIds[i];
      wholePathIds[i] = wholePathIds[other_idx];
      wholePathIds[other_idx] = temp;
    }
  }

  return wholePathIds;
}

int ESPPlanner::GetTruePathIdx(const std::vector<sbpl::Path> &paths) {
  // GetTruePathIdxAllEdges(paths);
  using namespace sbpl;

  if (paths.empty()) {
    printf("WARNING: There are no paths to goal!\n");
    return -1;
  }

  std::shared_ptr<EdgeSelectorSSP> ssp(new EdgeSelectorSSP());
  ssp->SetPaths(paths);

  const int num_edges = ssp->NumStochasticEdges();
  SSPState start_state(num_edges);
  start_state.suboptimality_bound = ssp->GetSuboptimalityBound(start_state);
  int start_id = ssp->state_hasher_.GetStateIDForceful(start_state);

  LAOPlanner<EdgeSelectorSSP> lao_planner(ssp);
  lao_planner.SetStart(start_id);
  // lao_planner.Plan(LAOPlannerParams::ParamsForOptimalPolicy());
  lao_planner.Plan(LAOPlannerParams::ParamsForOnlinePolicy(
                     kMaxLAOPolicyPlanningTime));

  auto policy_map = lao_planner.GetPolicyMap();
  printf("Policy for Edge Evaluation\n");

  for (const auto &element : policy_map) {
    const int state_id = element.first;
    const SSPState state = ssp->state_hasher_.GetState(state_id);
    cout << state << endl << "Action: " << element.second << endl << endl;
  }

  // Actually run the policy.
  SSPState current_state = start_state;
  const double kBoundForTermination = 1e-3;
  int best_valid_path_idx = 0;

  while (ssp->GetSuboptimalityBound(current_state) > kBoundForTermination) {
    // Best action to execute.
    const int current_state_id = ssp->state_hasher_.GetStateID(current_state);


    auto it = policy_map.find(current_state_id);

    if (it == policy_map.end()) {
      cout << "ERROR: We do not have a policy for state " << current_state_id << endl
           <<
           current_state << endl;

      start_id = ssp->state_hasher_.GetStateIDForceful(current_state);
      lao_planner = LAOPlanner<EdgeSelectorSSP>(ssp);
      lao_planner.SetStart(start_id);
      lao_planner.Plan(LAOPlannerParams::ParamsForOnlinePolicy(0.01));
      policy_map = lao_planner.GetPolicyMap();
      continue;
      // return -1;
    }

    const int edge_id = it->second;
    const Edge &edge_to_evaluate = ssp->EdgeIDToEdge(edge_id);
    printf("Evaluating edge %d:  (%d  %d) \n", edge_id, edge_to_evaluate.first,
           edge_to_evaluate.second);
    bool valid = environment_wrapper_->EvaluateOriginalEdge(edge_to_evaluate.first,
                                                            edge_to_evaluate.second);

    if (valid) {
      current_state.valid_bits.set(edge_id, true);
      valid_edges_.push_back(edge_to_evaluate);
    } else {
      current_state.invalid_bits.set(edge_id, true);
      invalid_edges_.push_back(edge_to_evaluate);
    }

    best_valid_path_idx = ssp->GetBestValidPathIdx(current_state);
    printf("Current best path idx is %d\n", best_valid_path_idx);
    cout << "Current state " << current_state.to_string() << endl;
    cout << "Current bound " << ssp->GetSuboptimalityBound(current_state) << endl;
  }

  return best_valid_path_idx;
}

int ESPPlanner::GetTruePathIdxAllEdges(const std::vector<sbpl::Path> &paths) {
  using namespace sbpl;

  if (paths.empty()) {
    printf("WARNING: There are no paths to goal!\n");
    return -1;
  }

  std::shared_ptr<EdgeSelectorSSP> ssp(new EdgeSelectorSSP());
  ssp->SetPaths(paths);

  const int num_edges = ssp->NumStochasticEdges();
  SSPState start_state(num_edges);
  start_state.suboptimality_bound = ssp->GetSuboptimalityBound(start_state);
  int start_id = ssp->state_hasher_.GetStateIDForceful(start_state);

  // Actually run the policy.
  SSPState current_state = start_state;
  const double kBoundForTermination = 1e-3;
  int best_valid_path_idx = 0;

  for (size_t ii = 0; ii < num_edges; ++ii) {
    const int edge_id = static_cast<int>(ii);
    const Edge &edge_to_evaluate = ssp->EdgeIDToEdge(edge_id);
    printf("Evaluating edge %d:  (%d  %d) \n", edge_id, edge_to_evaluate.first,
           edge_to_evaluate.second);
    bool valid = environment_wrapper_->EvaluateOriginalEdge(edge_to_evaluate.first,
                                                            edge_to_evaluate.second);
    if (valid) {
      current_state.valid_bits.set(edge_id, true);
      valid_edges_.push_back(edge_to_evaluate);
    } else {
      current_state.invalid_bits.set(edge_id, true);
      invalid_edges_.push_back(edge_to_evaluate);
    }

    best_valid_path_idx = ssp->GetBestValidPathIdx(current_state);
    printf("Current best path idx is %d\n", best_valid_path_idx);
    cout << "Current state " << current_state.to_string() << endl;
    cout << "Current bound " << ssp->GetSuboptimalityBound(current_state) << endl;
  }


  return best_valid_path_idx;
}

int ESPPlanner::RunEvaluation() {
  vector<int> path_ids;
  auto possible_paths = GetCurrentSolutionPaths(&path_ids);
  int valid_path_idx = GetTruePathIdx(possible_paths) ;
  // Mark these paths/goals as evaluated, so that they are considered again in
  // the future.
  int goal_wrapper_id = path_ids[valid_path_idx];
  // evaluated_goal_wrapper_ids_.insert(path_ids.begin(), path_ids.end());

  if (valid_path_idx != -1) {
    printf("A feasible solution (%d) has been found!!\n", goal_wrapper_id);
    // Exclude the valid path ID so that it can be obtained later.
    evaluated_goal_wrapper_ids_.erase(goal_wrapper_id);
  }

  return valid_path_idx;
}

bool ESPPlanner::outOfTime() {
  //if the user has sent an interrupt signal we stop
  if (interruptFlag) {
    return true;
  }

  //if we are supposed to run until the first solution, then we are never out of time
  if (params.return_first_solution) {
    return false;
  }

  double time_used = double(clock() - TimeStarted) / CLOCKS_PER_SEC;

  if (time_used >= params.max_time) {
    printf("out of max time\n");
  }

  if (use_repair_time && eps_satisfied != INFINITECOST &&
      time_used >= params.repair_time) {
    printf("used all repair time...\n");
  }

  //we are out of time if:
  //we used up the max time limit OR
  //we found some solution and used up the minimum time limit
  return time_used >= params.max_time ||
         (use_repair_time && eps_satisfied != INFINITECOST &&
          time_used >= params.repair_time);
}

void ESPPlanner::initializeSearch() {
  //it's a new search, so increment replan_number and reset the search_iteration
  replan_number++;
  search_iteration = 0;
  search_expands = 0;
  totalPlanTime = 0;
  totalExpands = 0;

  //clear open list, incons list, and stats list
  heap.makeemptyheap();
  incons.clear();
  stats.clear();

  //initialize epsilon variable
  eps = params.initial_eps;
  eps_satisfied = INFINITECOST;

  //call get state to initialize the start and goal states
  goal_state = GetState(goal_state_id);
  start_state = GetState(start_state_id);

  //put start state in the heap
  start_state->g = 0;
  CKey key;
  key.key[0] = eps * start_state->h;
  heap.insertheap(start_state, key);

  //ensure heuristics are up-to-date
  environment_wrapper_->EnsureHeuristicsUpdated((bforwardsearch == true));
}

bool ESPPlanner::Search(vector<int> &pathIds, int &PathCost) {
  CKey key;
  TimeStarted = clock();

  initializeSearch();

  //the main loop of ARA*
  while (eps_satisfied > params.final_eps && !outOfTime()) {

    //run weighted A*
    clock_t before_time = clock();
    int before_expands = search_expands;
    //ImprovePath returns:
    //1 if the solution is found
    //0 if the solution does not exist
    //2 if it ran out of time
    int ret = ImprovePath();

    if (ret == 1) { //solution found for this iteration
      eps_satisfied = eps;
    }

    int delta_expands = search_expands - before_expands;
    double delta_time = double(clock() - before_time) / CLOCKS_PER_SEC;

    //print the bound, expands, and time for that iteration
    printf("bound=%f expands=%d cost=%d time=%.3f\n",
           eps_satisfied, delta_expands, goal_state->g, delta_time);

    //update stats
    totalPlanTime += delta_time;
    totalExpands += delta_expands;
    PlannerStats tempStat;
    tempStat.eps = eps_satisfied;
    tempStat.expands = delta_expands;
    tempStat.time = delta_time;
    tempStat.cost = goal_state->g;
    stats.push_back(tempStat);

    //no solution exists
    if (ret == 0) {
      printf("Solution does not exist\n");
      return false;
    }

    //if we're just supposed to find the first solution
    //or if we ran out of time, we're done
    if (params.return_first_solution || ret == 2) {
      break;
    }

    prepareNextSearchIteration();
  }

  if (goal_state->g == INFINITECOST) {
    printf("could not find a solution (ran out of time)\n");
    return false;
  }

  if (eps_satisfied == INFINITECOST) {
    printf("WARNING: a solution was found but we don't have quality bound for it!\n");
  }

  printf("solution found\n");
  clock_t before_reconstruct = clock();
  pathIds = GetSearchPath(goal_state_id, PathCost);
  reconstructTime = double(clock() - before_reconstruct) / CLOCKS_PER_SEC;
  totalTime = totalPlanTime + reconstructTime;

  return true;
}

void ESPPlanner::prepareNextSearchIteration() {
  //decrease epsilon
  eps -= params.dec_eps;

  if (eps < params.final_eps) {
    eps = params.final_eps;
  }

  //dump the inconsistent states into the open list
  CKey key;

  while (!incons.empty()) {
    ESPState *s = incons.back();
    incons.pop_back();
    s->in_incons = false;
    key.key[0] = s->g + int(eps * s->h);
    heap.insertheap(s, key);
  }

  //recompute priorities for states in OPEN and reorder it
  for (int i = 1; i <= heap.currentsize; ++i) {
    ESPState *state = (ESPState *)heap.heap[i].heapstate;
    heap.heap[i].key.key[0] = state->g + int(eps * state->h);
  }

  heap.makeheap();

  search_iteration++;
}


//-----------------------------Interface function-----------------------------------------------------

void ESPPlanner::interrupt() {
  interruptFlag = true;
}

int ESPPlanner::replan(vector<int> *solution_stateIDs_V, ReplanParams p) {
  int solcost;
  return replan(solution_stateIDs_V, p, &solcost);
}

int ESPPlanner::replan(int start, int goal, vector<int> *solution_stateIDs_V,
                       ReplanParams p, int *solcost) {
  set_start(start);
  set_goal(goal);
  return replan(solution_stateIDs_V, p, solcost);
}

int ESPPlanner::replan(vector<int> *solution_stateIDs_V, ReplanParams p,
                       int *solcost) {
  printf("planner: replan called\n");
  params = p;
  use_repair_time = params.repair_time >= 0;
  interruptFlag = false;

  if (goal_state_id < 0) {
    printf("ERROR searching: no goal state set\n");
    return 0;
  }

  if (start_state_id < 0) {
    printf("ERROR searching: no start state set\n");
    return 0;
  }

  //plan
  vector<int> pathIds;
  int PathCost;
  bool solnFound = Search(pathIds, PathCost);
  printf("total expands=%d planning time=%.3f reconstruct path time=%.3f total time=%.3f solution cost=%d\n",
         totalExpands, totalPlanTime, reconstructTime, totalTime, goal_state->g);

  //copy the solution
  *solution_stateIDs_V = pathIds;
  *solcost = PathCost;

  start_state_id = -1;
  goal_state_id = -1;

  return (int)solnFound;
}

int ESPPlanner::replan(vector<sbpl::Path> *solution_paths, ReplanParams p,
                       int *solcost) {
  printf("planner: replan called\n");
  params = p;
  use_repair_time = params.repair_time >= 0;
  interruptFlag = false;

  if (goal_state_id < 0) {
    printf("ERROR searching: no goal state set\n");
    return 0;
  }

  if (start_state_id < 0) {
    printf("ERROR searching: no start state set\n");
    return 0;
  }

  //plan
  vector<int> pathIds;
  int PathCost;
  bool solnFound = Search(pathIds, PathCost);
  printf("total expands=%d planning time=%.3f reconstruct path time=%.3f total time=%.3f solution cost=%d\n",
         totalExpands, totalPlanTime, reconstructTime, totalTime, goal_state->g);

  //copy the solution
  // *solution_stateIDs_V = pathIds;
  *solcost = PathCost;

  vector<int> path_ids;
  *solution_paths = GetCurrentSolutionPaths(&path_ids);

  printf("There are %d paths to the goal\n",
         static_cast<int>(solution_paths->size()));
  // environment_wrapper_->wrapper_state_hasher_.Print();

  start_state_id = -1;
  goal_state_id = -1;

  return (int)solnFound;
}

vector<sbpl::Path> ESPPlanner::GetCurrentSolutionPaths(vector<int> *path_ids) {
  path_ids->clear();
  // Get all paths.
  vector<sbpl::Path> solution_paths;
  vector<int> all_goal_wrapper_ids = environment_wrapper_->GetAllGoalWrapperIDs(
                                       goal_state_id);
  solution_paths.reserve(all_goal_wrapper_ids.size());
  path_ids->reserve(all_goal_wrapper_ids.size());

  for (size_t ii = 0; ii < all_goal_wrapper_ids.size(); ++ii) {
    // Ignore a path to goal if it has not been marked as expanded.
    auto search_state = GetState(all_goal_wrapper_ids[ii]);

    //  Skip any wrapper goal states that were never marked as closed during
    //  the search.
    if (search_state->expanded_best_parent == NULL) {
      continue;
    }

    // Skip any wrapper goal states that have already been evaluated (either as
    // valid or invalid).
    if (evaluated_goal_wrapper_ids_.find(all_goal_wrapper_ids[ii]) !=
        evaluated_goal_wrapper_ids_.end()) {
      continue;
    }

    int path_cost = 0;
    vector<int> wrapper_ids_path = GetSearchPath(all_goal_wrapper_ids[ii],
                                                 path_cost);
    auto solution_path = environment_wrapper_->ConvertWrapperIDsPathToSBPLPath(
                           wrapper_ids_path);
    solution_path.cost = path_cost;
    solution_paths.push_back(solution_path);
    path_ids->push_back(all_goal_wrapper_ids[ii]);
  }

  return solution_paths;
}

int ESPPlanner::set_goal(int id) {
  const int wrapper_goal_id = environment_wrapper_->GetWrapperStateID(id,
                                                                      std::numeric_limits<double>::lowest());
  printf("planner: setting env goal to %d and wrapper goal to %d\n", id,
         wrapper_goal_id);

  if (bforwardsearch) {
    goal_state_id = wrapper_goal_id;
  } else {
    start_state_id = wrapper_goal_id;
  }

  return 1;
}

int ESPPlanner::set_start(int id) {
  const int wrapper_start_id = environment_wrapper_->GetWrapperStateID(id, 0.0);
  printf("planner: setting env start to %d and wrapper start to %d\n", id,
         wrapper_start_id);

  if (bforwardsearch) {
    start_state_id = wrapper_start_id;
  } else {
    goal_state_id = wrapper_start_id;
  }

  return 1;
}


//---------------------------------------------------------------------------------------------------------


void ESPPlanner::get_search_stats(vector<PlannerStats> *s) {
  s->clear();
  s->reserve(stats.size());

  for (unsigned int i = 0; i < stats.size(); i++) {
    s->push_back(stats[i]);
  }
}

