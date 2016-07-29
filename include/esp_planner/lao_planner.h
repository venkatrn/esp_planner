#pragma once

/**
 * @file lao_planner.h
 * @brief Implements the algorithm in https://www.ics.uci.edu/~dechter/papers/paginated_binders/PART%25201.pdf
 * @author Venkatraman Narayanan
 * Carnegie Mellon University, 2014
 * Change Log:
 * Jul 2016: Templatized on AbstractMDP, added most likely path reconstruction,
 * plus several other improvements.
 */

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <ctime>
#include <limits>
#include <memory>
#include <stack>
#include <unordered_map>
#include <vector>

namespace {
  const int kMaxPlannerExpansions = 10000;
  // Tolerance for comparing double numbers.
  const double kDblTolerance = 1e-4;
} // namespace

namespace sbpl {

struct PlannerState {
  int state_id;
  double v;
  bool expanded;

  // Successors indexed by action id
  std::vector<int> action_ids;
  std::vector<std::vector<int>> succ_state_ids_map;
  std::vector<std::vector<double>> succ_state_probabilities_map;
  std::vector<std::vector<double>> action_costs_map;
  int parent_state_id;
  int best_action_id;
  int best_vec_idx; //Cached for convenience -- action_ids[best_vec_idx] = best_action_id

  PlannerState() {
    state_id = -1;
    v = 0;
    expanded = false;
    parent_state_id = -1;
    best_action_id = -1;
    best_vec_idx = -1;
  }

  PlannerState(int s_id, int v_val, int parent_s_id) {
    state_id = s_id;
    v = v_val;
    expanded = false;
    parent_state_id = parent_s_id;
    best_action_id = -1;
    best_vec_idx = -1;
  }

};

// TODO: Use sbpl_utils::hash_manager
struct StateHasher {
  int operator() (const PlannerState &s) const {
    return s.state_id;
  }
};

struct StateEqual {
  bool operator() (const PlannerState &s1, const PlannerState &s2) const {
    return (s1.state_id == s2.state_id);
  }
};

struct PlannerStats {
  int expansions;
  double time;
  int cost;

  PlannerStats() : expansions(-1),
    time(-1.0),
    cost(-1) {
  }
};

template<class AbstractMDP>
class LAOPlanner {
 public:
  LAOPlanner(std::shared_ptr<AbstractMDP> abstract_mdp);
  ~LAOPlanner();
  void SetStart(int start_state_id);
  bool Plan(std::vector<int> *state_ids, std::vector<int> *action_ids = nullptr);
  PlannerStats GetPlannerStats();
  const std::unordered_map<int, int> &GetPolicyMap() {
    return optimal_policy_map_;
  }
  

 private:
  std::shared_ptr<AbstractMDP> abstract_mdp_;
  int start_state_id_;
  PlannerStats planner_stats_;

  // policy_map_ is the map for all states (some of which may not be reachable
  // from start), and optimal_policy_map_ is the final solution graph.
  // Mapping from state ID to action ID.
  std::unordered_map<int, int> policy_map_; 
  std::unordered_map<int, int> optimal_policy_map_; 

  /**@brief Return a postorder DFS traversal (state ids) of the best solution graph**/
  void DFSTraversal(std::vector<int> *traversal);
  /**@brief Reconstruct optimistic path (actions lead to successor with smallest V-value)**/
  void ReconstructOptimisticPath(std::vector<int> *state_ids,
                                 std::vector<int> *action_ids);
  /**@brief Reconstruct most likely path (actions lead to successor with highest transition probability)**/
  void ReconstructMostLikelyPath(std::vector<int> *state_ids,
                                 std::vector<int> *action_ids);
  /**@brief Do value iteration on the best solution graph**/
  void SolutionValueIteration();

  /**@brief Planner hash table**/
  std::unordered_map<int, PlannerState> PlannerStateMap;
  PlannerState StateIDToState(int state_id);
  PlannerState *StateIDToMutableState(int state_id);

  /**@Debug**/
  void PrintPlannerStateMap();
};

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Template Implementation
/////////////////////////////////////////////////////////////////////////////////////////////////////


template <class AbstractMDP>
LAOPlanner<AbstractMDP>::LAOPlanner(std::shared_ptr<AbstractMDP> abstract_mdp) {
  start_state_id_ = -1;
  abstract_mdp_ = abstract_mdp;
}

template <class AbstractMDP>
LAOPlanner<AbstractMDP>::~LAOPlanner() {
}

template <class AbstractMDP>
void LAOPlanner<AbstractMDP>::SetStart(int start_state_id) {
  start_state_id_ = start_state_id;
  return;
}

template <class AbstractMDP>
void LAOPlanner<AbstractMDP>::DFSTraversal(std::vector<int> *traversal) {
  traversal->clear();
  std::stack<int> dfs_stack;
  std::vector<int> visited_states;
  dfs_stack.push(start_state_id_);
  visited_states.push_back(start_state_id_);
  bool terminal_state = false;

  while (dfs_stack.size() != 0) {
    while (!terminal_state) {
      PlannerState s = StateIDToState(dfs_stack.top());
      //printf("Top state id: %d", s.state_id);
      int best_vec_idx = s.best_vec_idx;

      //printf("Best action id: %d", s.best_action_id);
      if (best_vec_idx == -1 || abstract_mdp_->IsGoalState(dfs_stack.top())) {
        terminal_state = true;
        break;
      }

      std::vector<int> best_action_succs = s.succ_state_ids_map[best_vec_idx];

      if (best_action_succs.size() == 0) {
        terminal_state = true;
        break;
      }

      // Declare terminal state if no successors are inserted
      terminal_state = true;

      for (size_t ii = 0; ii < best_action_succs.size(); ++ii) {
        // Avoid inserting duplicates (possible in a graph)
        auto it = find(visited_states.begin(), visited_states.end(),
                       best_action_succs[ii]);

        if (it != visited_states.end()) {
          continue;
        }

        dfs_stack.push(best_action_succs[ii]);
        visited_states.push_back(best_action_succs[ii]);
        terminal_state = false;
      }
    }

    traversal->push_back(dfs_stack.top());
    dfs_stack.pop();
    terminal_state = false;
  }
}

// The planner will return all waypoints, and force prims for each waypoint except the last one.
// So state_ids.size() = action_ids.size() + 1
template <class AbstractMDP>
void LAOPlanner<AbstractMDP>::ReconstructOptimisticPath(std::vector<int> *state_ids,
                                           std::vector<int> *action_ids) {
  printf("[LAO Planner]: Reconstructing optimistic path\n");
  assert(state_ids != nullptr);
  assert(action_ids != nullptr);
  state_ids->clear();
  action_ids->clear();

  PlannerState current_state = PlannerStateMap[start_state_id_];
  assert(start_state_id_ == current_state.state_id);
  state_ids->push_back(current_state.state_id);
  action_ids->push_back(current_state.best_action_id);

  // Until terminal state is reached
  while (current_state.best_vec_idx != -1 &&
         !abstract_mdp_->IsGoalState(current_state.state_id)) {
    //printf("State: %d, Val: %f", current_state.state_id, current_state.v);
    int best_vec_idx = current_state.best_vec_idx;
    std::vector<int> succ_state_ids = current_state.succ_state_ids_map[best_vec_idx];
    double min_val = std::numeric_limits<double>::max();
    int optimistic_succ_id = -1;

    for (size_t ii = 0; ii < succ_state_ids.size(); ++ii) {
      PlannerState s = PlannerStateMap[succ_state_ids[ii]];

      if (s.v < min_val) {
        min_val = s.v;
        optimistic_succ_id = succ_state_ids[ii];
      }
    }

    assert(optimistic_succ_id != -1);

    if (current_state.state_id == optimistic_succ_id) {
      printf("[LAO Planner]: Error in path reconstruction. Optimistic successor has same ID has current state.\n");
      return;
    }

    current_state = PlannerStateMap[optimistic_succ_id];
    //TODO: Check v-values are non-increasing
    state_ids->push_back(current_state.state_id);

    //Don't push in the action for the goal state
    if (current_state.best_vec_idx != -1 &&
        !abstract_mdp_->IsGoalState(current_state.state_id)) {
      action_ids->push_back(current_state.best_action_id);
    }
  }

  printf("[LAO Planner]: Path reconstruction successful\n");
  return;
}

template <class AbstractMDP>
void LAOPlanner<AbstractMDP>::ReconstructMostLikelyPath(std::vector<int> *state_ids,
                                           std::vector<int> *action_ids) {
  printf("[LAO Planner]: Reconstructing most likely path\n");
  assert(state_ids != nullptr);
  assert(action_ids != nullptr);
  state_ids->clear();
  action_ids->clear();

  PlannerState current_state = PlannerStateMap[start_state_id_];
  assert(start_state_id_ == current_state.state_id);
  state_ids->push_back(current_state.state_id);
  action_ids->push_back(current_state.best_action_id);

  // Until terminal state is reached
  while (current_state.best_vec_idx != -1 &&
         !abstract_mdp_->IsGoalState(current_state.state_id)) {
    //printf("State: %d, Val: %f", current_state.state_id, current_state.v);
    int best_vec_idx = current_state.best_vec_idx;
    std::vector<int> succ_state_ids = current_state.succ_state_ids_map[best_vec_idx];
    double max_prob = 0;
    int most_likely_succ_id = -1;

    const auto &probs = current_state.succ_state_probabilities_map[best_vec_idx];
    for (size_t ii = 0; ii < probs.size(); ++ii) {
      if(probs[ii] > max_prob) {
        max_prob = probs[ii];
        most_likely_succ_id = current_state.succ_state_ids_map[best_vec_idx][ii];
      }
    }

    assert(optimistic_succ_id != -1);

    if (current_state.state_id == most_likely_succ_id) {
      printf("[LAO Planner]: Error in path reconstruction. Most likely successor has same ID has current state ");
      return;
    }

    current_state = PlannerStateMap[most_likely_succ_id];
    //TODO: Check v-values are non-increasing
    state_ids->push_back(current_state.state_id);

    //Don't push in the action for the goal state
    if (current_state.best_vec_idx != -1 &&
        !abstract_mdp_->IsGoalState(current_state.state_id)) {
      action_ids->push_back(current_state.best_action_id);
    }
  }

  printf("[LAO Planner]: Path reconstruction successful\n");
  return;
}

template <class AbstractMDP>
bool LAOPlanner<AbstractMDP>::Plan(std::vector<int> *state_ids, std::vector<int> *action_ids) {
  // Plan from scratch (assume goal changed)
  PlannerStateMap.clear();

  if (!abstract_mdp_) {
    printf("[LAO Planner]: Environment is not defined\n");
    return false;
  }

  PlannerState start_state(start_state_id_,
                           abstract_mdp_->GetGoalHeuristic(start_state_id_), -1);
  PlannerStateMap[start_state_id_] = start_state;

  bool exists_non_terminal_states = true;
  planner_stats_.expansions = 0;
  clock_t begin_time = clock();

  while (exists_non_terminal_states) {
    if (planner_stats_.expansions > kMaxPlannerExpansions) {
      printf("[LAO Planner]: Exceeded max expansion. Returning optimistic path anyway.\n");
      break;
      //return false;
    }

    // Get the DFS traversal of the best partial solution graph
    std::vector<int> dfs_traversal;
    DFSTraversal(&dfs_traversal);
    // Declare no non-terminal states when we don't expand anything from the traversal
    exists_non_terminal_states = false;

    // DEBUG
    /*
       printf("DFS Traversal");
       for (int ii = 0; ii < dfs_traversal.size(); ++ii)
       {
       printf("%d", dfs_traversal[ii]);
       }
       */
    // Iterate through and expand state and/or update V-values
    for (size_t ii = 0; ii < dfs_traversal.size(); ++ii) {
      PlannerState s = StateIDToState(dfs_traversal[ii]);

      // Ignore goal states
      // TODO: Update this. Search ends if state being expanded is a goal state
      if (abstract_mdp_->IsGoalState(s.state_id)) {
        //printf("[LAO Planner]: Goal state has been found\n");
        // exists_non_terminal_states = false;
        // break;
        continue;
      }

      // Expand the state if not already expanded
      if (!s.expanded) {
        exists_non_terminal_states = true;
        planner_stats_.expansions++;
        //printf("Expanding state %d", s.state_id);
        std::vector<std::vector<int>> succ_state_ids_map;
        std::vector<std::vector<double>> succ_state_probabilities_map;
        std::vector<int> action_ids;
        std::vector<std::vector<double>> action_costs_map;
        abstract_mdp_->GetSuccs(s.state_id, &succ_state_ids_map,
                                &succ_state_probabilities_map,
                                &action_ids, &action_costs_map);

        s.action_ids = action_ids;
        s.action_costs_map = action_costs_map;
        s.succ_state_ids_map = succ_state_ids_map;
        s.succ_state_probabilities_map = succ_state_probabilities_map;
        s.expanded = true;

        /*
           printf("Succs:");
           for (int ii = 0; ii < succ_state_ids_map.size(); ++ii)
           {
           printf("Edge: %d: %f", action_ids[ii], action_costs[ii]);
           for (int jj = 0; jj < succ_state_ids_map[ii].size(); ++jj)
           {
           printf("State: %d: %f", succ_state_ids_map[ii][jj], succ_state_probabilities_map[ii][jj]);
           }
           }
           */

        for (size_t ii = 0; ii < action_ids.size(); ++ii) {
          for (size_t jj = 0; jj < succ_state_ids_map[ii].size(); ++jj) {
            const int succ_id = succ_state_ids_map[ii][jj];
            // Skip successors that have already been generated
            auto it = PlannerStateMap.find(succ_id);

            if (it != PlannerStateMap.end()) {
              continue;
            }

            PlannerState succ_state(succ_state_ids_map[ii][jj],
                                    abstract_mdp_->GetGoalHeuristic(succ_state_ids_map[ii][jj]), s.state_id);
            PlannerStateMap[succ_id] = succ_state;
          }
        }
      }

      // Update V-values: V(s) = min_{a\in A} \sum_{s'} P(s'|s,a)*(c(s,a,s') + V(s'))
      const int num_actions = s.succ_state_ids_map.size();
      assert(num_actions == int(s.succ_state_probabilities_map.size()));
      assert(num_actions == int(s.action_ids.size()));
      double min_expected_cost = std::numeric_limits<double>::max();
      int best_idx = -1;

      for (int jj = 0; jj < num_actions; ++jj) {
        const int num_succs = s.succ_state_ids_map[jj].size();
        assert(num_succs == int(s.succ_state_probabilities_map[jj].size()));
        double expected_cost = 0;

        for (int kk = 0; kk < num_succs; ++kk) {
          PlannerState succ_state = StateIDToState(s.succ_state_ids_map[jj][kk]);
          expected_cost += ((succ_state.v + s.action_costs_map[jj][kk]) *
                            s.succ_state_probabilities_map[jj][kk]);
        }

        if (expected_cost < min_expected_cost) {
          min_expected_cost = expected_cost;
          best_idx = jj;
        }
      }

      assert(best_idx != -1);
      s.best_action_id = s.action_ids[best_idx];
      s.best_vec_idx = best_idx;
      s.v = min_expected_cost;

      // Update policy map.
      policy_map_[s.state_id] = s.best_action_id;

      // Update the state in PlannerStateMap
      PlannerStateMap[s.state_id] = s;
    }
  }

  clock_t end_time = clock();
  planner_stats_.time = double(end_time - begin_time) / CLOCKS_PER_SEC;
  planner_stats_.cost = PlannerStateMap[start_state.state_id].v;

  // Reconstruct path
  printf("[LAO Planner]: Finished planning\n");
  SolutionValueIteration();
  //printf("[LAO Planner]: Finished value iteration");
  // ReconstructOptimisticPath(state_ids, action_ids);
  ReconstructMostLikelyPath(state_ids, action_ids);
  // PrintPlannerStateMap();
  return true;
}

template <class AbstractMDP>
void LAOPlanner<AbstractMDP>::SolutionValueIteration() {
  // Get the DFS traversal of the best partial solution graph
  std::vector<int> dfs_traversal;
  DFSTraversal(&dfs_traversal);
  const int max_iter = 100;
  const double error_tol = 1e-3;
  double error = std::numeric_limits<double>::max();
  int iter = 0;

  while (error > error_tol && iter < max_iter) {
    iter++;
    error = 0.0;

    for (size_t ii = 0; ii < dfs_traversal.size(); ++ii) {
      PlannerState s = StateIDToState(dfs_traversal[ii]);
      // Update V-values: V(s) = min_{a\in A} c(s,a) + \sum_{s'} P(s'|s,a)*V(s')
      const int num_actions = s.succ_state_ids_map.size();
      assert(num_actions == int(s.succ_state_probabilities_map.size()));
      assert(num_actions == int(s.action_ids.size()));
      double min_expected_cost = std::numeric_limits<double>::max();
      int best_idx = -1;

      for (int jj = 0; jj < num_actions; ++jj) {
        const int num_succs = s.succ_state_ids_map[jj].size();
        assert(num_succs == int(s.succ_state_probabilities_map[jj].size()));
        double expected_cost = 0;

        for (int kk = 0; kk < num_succs; ++kk) {
          PlannerState succ_state = StateIDToState(s.succ_state_ids_map[jj][kk]);
          expected_cost += ((succ_state.v + s.action_costs_map[jj][kk]) *
                            s.succ_state_probabilities_map[jj][kk]);
        }

        if (expected_cost < min_expected_cost) {
          min_expected_cost = expected_cost;
          best_idx = jj;
        }
      }

      if (best_idx == -1) {
        continue;  //Ignore terminal states
      }

      //assert(best_idx != -1);
      s.best_action_id = s.action_ids[best_idx];
      s.best_vec_idx = best_idx;
      error += fabs(s.v - min_expected_cost);
      s.v = min_expected_cost;
      optimal_policy_map_[s.state_id] = s.best_action_id;

      // Update the state in PlannerStateMap
      PlannerStateMap[s.state_id] = s;
    }

    //printf("[LAO Planner]: VI Iteration %d, Error: %f", iter, error);
  }

  if (iter == max_iter) {
    printf("[LAO Planner]: Value iteration exceeded max_iter limit. Error in V-values is %f\n",
             error);
  }
}

template <class AbstractMDP>
PlannerStats LAOPlanner<AbstractMDP>::GetPlannerStats() {
  return planner_stats_;
}


template <class AbstractMDP>
PlannerState LAOPlanner<AbstractMDP>::StateIDToState(int state_id) {
  auto it = PlannerStateMap.find(state_id);

  if (it != PlannerStateMap.end()) {
    return it->second;
  } else {
    printf("LAO Planner: Error. Requested State ID does not exist. Will return empty state.\n");
  }

  PlannerState empty_state;
  return empty_state;
}

template <class AbstractMDP>
PlannerState *LAOPlanner<AbstractMDP>::StateIDToMutableState(int state_id) {
  auto it = PlannerStateMap.find(state_id);

  if (it != PlannerStateMap.end()) {
    return &(it->second);
  } else {
    printf("LAO Planner: Error. Requested State ID does not exist. Will return empty state.\n");
  }

  return nullptr;
}

template <class AbstractMDP>
void LAOPlanner<AbstractMDP>::PrintPlannerStateMap() {
  for (auto it = PlannerStateMap.begin(); it != PlannerStateMap.end(); ++it) {
    printf("ID: %d | V: %f | E: %d\n", it->first, it->second.v,
             it->second.expanded);
  }
}
} // namespace sbpl
