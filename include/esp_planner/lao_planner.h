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

#include <sbpl_utils/hash_manager/hash_manager.h>

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <chrono>
#include <limits>
#include <memory>
#include <stack>
#include <unordered_map>
#include <vector>

using high_res_clock = std::chrono::high_resolution_clock;

namespace {
constexpr int kMaxPlannerExpansions = 100000;
// Tolerance for comparing double numbers.
constexpr double kDblTolerance = 1e-4;
// LAO* will terminate when total residual of the best solution graph is less
// than this value (and when no non-terminal states exist).
constexpr double kMaxResidualForTermination = 1e-3;
} // namespace

namespace sbpl {

struct PlannerState {
  int state_id = -1;
  double v = 0;
  bool expanded = false;

  // Successors indexed by action id
  std::vector<int> action_ids;
  std::vector<std::vector<int>> succ_state_ids_map;
  std::vector<std::vector<double>> succ_state_probabilities_map;
  std::vector<std::vector<double>> action_costs_map;
  int parent_state_id = -1;
  int best_action_id = -1;
  int best_vec_idx =
    -1; //Cached for convenience -- action_ids[best_vec_idx] = best_action_id

  PlannerState() = default;
  PlannerState(const PlannerState &other) = default;

  PlannerState(int s_id, double v_val, int parent_s_id) {
    state_id = s_id;
    v = v_val;
    expanded = false;
    parent_state_id = parent_s_id;
    best_action_id = -1;
    best_vec_idx = -1;
  }
  bool operator==(const PlannerState &other) const {
    return state_id == other.state_id;
  }
  bool operator!=(const PlannerState &other) const {
    return !(*this == other);
  }
  size_t GetHash() const {
    return static_cast<size_t>(state_id);
  }
};

struct PlannerStats {
  int expansions = -1;
  double time = -1.0;
  int cost = -1;
  int num_backups = -1;
  PlannerStats() = default;
  PlannerStats(const PlannerStats &other) = default;
};

// The LAO planner will terminate if planning time exceeds max_plan_time or
// number of expansions (calls to GetSuccs) exceeds max_expansions or when the
// Bellman residual drops below max_allowed_bellman_residual. In the last case,
// the returned policy (which can be obtained by GetPolicyMap) is guaranteed to
// be a proper policy and an optimal one, while in the first two, there is no
// guarantee that the policy is a proper one (i.e, you might end up in a state
// for which no action has been computed). The first two are appropriate in an
// online setting, where you can plan for some time, execute the best action
// for current state, update current state, replan, iterate until you arrive at
// a terminal state.
struct LAOPlannerParams {
  double max_plan_time = 10.0; // seconds
  int max_expansions = 100000;
  double max_allowed_bellman_residual = 1e-3;
  // If true, this will throw away our existing solution graph and start over.
  // Otherwise, planning will use the existing solution graph. Use the latter
  // option when running the planner in an online setting.
  bool plan_from_scratch = true;

  // Provide some commonly used parameter settings.
  static LAOPlannerParams Default() {
    return LAOPlannerParams();
  }
  static LAOPlannerParams ParamsForOptimalPolicy() {
    LAOPlannerParams params;
    params.max_plan_time = std::numeric_limits<double>::max();
    params.max_expansions = std::numeric_limits<int>::max();
    params.max_allowed_bellman_residual = 1e-10;
    params.plan_from_scratch = true;
    return params;
  }
  static LAOPlannerParams ParamsForOnlinePolicy(double episode_time) {
    LAOPlannerParams params;
    params.max_plan_time = episode_time;
    params.max_expansions = std::numeric_limits<int>::max();
    params.max_allowed_bellman_residual = 1e-10;
    params.plan_from_scratch = false;
    return params;
  }
};

// The LAOPlanner is an MDP solver that finds an optimal policy given a start
// state. This is templated on an AbstractMDP class which must implement these
// two methods:

// void GetSuccs(int source_state_id,
//               vector<vector<int>* succ_ids,
//               vector<vector<double>* succ_probabilities,
//               vector<vector<double>>* action_costs_map);
//
// If "k" actions are applicable at the source_state s, then
// succ_ids.size() = succ_probabilities.size() = action_costs_map.size() = k.
// If 'm' there are possible successor states for applying action a (0 <= a <
// k), then succ_ids[a].size() = succ_probabilities[a].size() =
// action_costs_map[a].size() = m.  These terms represent the resulting
// successor state ID, transition probability and corresponding cost (note that
// the cost is defined as a function of <source state, action, resulting
// state> to keep it as general as possible).
//
// bool IsGoalState(int state_id);
//
//
// This should return true if the state corresponding to state_id is a
// terminal/absorbing state.
//
// double GetGoalHeuristic(int state_id);
//
// This should return a non-overestimating cost-to-go value for the state
// corresponding to state_id, under the optimal policy. This could be something
// like Euclidean distance in a stochastic shortest path problem (SSP) on grids. If
// unsure, simply return 0.
template<class AbstractMDP>
class LAOPlanner {
 public:
  LAOPlanner(std::shared_ptr<AbstractMDP> abstract_mdp);
  ~LAOPlanner();
  void SetStart(int start_state_id);
  bool Plan(const LAOPlannerParams &planner_params);
  PlannerStats GetPlannerStats();
  const std::unordered_map<int, int> &GetPolicyMap() {
    return optimal_policy_map_;
  }
  /**@brief Reconstruct optimistic path (actions lead to successor with smallest V-value)**/
  void ReconstructOptimisticPath(std::vector<int> *state_ids,
                                 std::vector<int> *action_ids);
  /**@brief Reconstruct most likely path (actions lead to successor with highest transition probability)**/
  void ReconstructMostLikelyPath(std::vector<int> *state_ids,
                                 std::vector<int> *action_ids);

 private:
  std::shared_ptr<AbstractMDP> abstract_mdp_;
  int start_state_id_;
  PlannerStats planner_stats_;
  sbpl_utils::HashManager<PlannerState> state_hasher_;
  LAOPlannerParams planner_params_;

  // policy_map_ is the map for all states (some of which may not be reachable
  // from start), and optimal_policy_map_ is the final solution graph.
  // Mapping from state ID to action ID.
  std::unordered_map<int, int> policy_map_;
  std::unordered_map<int, int> optimal_policy_map_;

  /**@brief Return a postorder DFS traversal (state ids) of the best solution graph**/
  void DFSTraversal(std::vector<int> *traversal);
  /**@brief Do value iteration on the best solution graph. Not used currently.**/
  void SolutionValueIteration();
};

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Template Implementation
/////////////////////////////////////////////////////////////////////////////////////////////////////


template <class AbstractMDP>
LAOPlanner<AbstractMDP>::LAOPlanner(std::shared_ptr<AbstractMDP>
                                    abstract_mdp) {
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
      PlannerState s = state_hasher_.GetState(dfs_stack.top());
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
void LAOPlanner<AbstractMDP>::ReconstructOptimisticPath(
  std::vector<int> *state_ids,
  std::vector<int> *action_ids) {
  printf("[LAO Planner]: Reconstructing optimistic path\n");
  assert(state_ids != nullptr);
  assert(action_ids != nullptr);
  state_ids->clear();
  action_ids->clear();

  PlannerState current_state = state_hasher_.GetState(start_state_id_);
  assert(start_state_id_ == current_state.state_id);
  state_ids->push_back(current_state.state_id);
  action_ids->push_back(current_state.best_action_id);

  // Until terminal state is reached
  while (current_state.best_vec_idx != -1 &&
         !abstract_mdp_->IsGoalState(current_state.state_id)) {
    //printf("State: %d, Val: %f", current_state.state_id, current_state.v);
    int best_vec_idx = current_state.best_vec_idx;
    std::vector<int> succ_state_ids =
      current_state.succ_state_ids_map[best_vec_idx];
    double min_val = std::numeric_limits<double>::max();
    int optimistic_succ_id = -1;

    for (size_t ii = 0; ii < succ_state_ids.size(); ++ii) {
      PlannerState s = state_hasher_.GetState(succ_state_ids[ii]);

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

    current_state = state_hasher_.GetState(optimistic_succ_id);
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
void LAOPlanner<AbstractMDP>::ReconstructMostLikelyPath(
  std::vector<int> *state_ids,
  std::vector<int> *action_ids) {
  printf("[LAO Planner]: Reconstructing most likely path\n");
  assert(state_ids != nullptr);
  assert(action_ids != nullptr);
  state_ids->clear();
  action_ids->clear();

  PlannerState current_state = state_hasher_.GetState(start_state_id_);
  assert(start_state_id_ == current_state.state_id);
  state_ids->push_back(current_state.state_id);
  action_ids->push_back(current_state.best_action_id);

  // Until terminal state is reached
  while (current_state.best_vec_idx != -1 &&
         !abstract_mdp_->IsGoalState(current_state.state_id)) {
    //printf("State: %d, Val: %f", current_state.state_id, current_state.v);
    int best_vec_idx = current_state.best_vec_idx;
    std::vector<int> succ_state_ids =
      current_state.succ_state_ids_map[best_vec_idx];
    double max_prob = 0;
    int most_likely_succ_id = -1;

    const auto &probs = current_state.succ_state_probabilities_map[best_vec_idx];

    for (size_t ii = 0; ii < probs.size(); ++ii) {
      if (probs[ii] > max_prob) {
        max_prob = probs[ii];
        most_likely_succ_id = current_state.succ_state_ids_map[best_vec_idx][ii];
      }
    }

    assert(most_likely_succ_id != -1);

    if (current_state.state_id == most_likely_succ_id) {
      printf("[LAO Planner]: Error in path reconstruction. Most likely successor has same ID has current state ");
      return;
    }

    current_state = state_hasher_.GetState(most_likely_succ_id);
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
bool LAOPlanner<AbstractMDP>::Plan(const LAOPlannerParams &params) {
  planner_params_ = params;

  if (planner_params_.plan_from_scratch) {
    state_hasher_.Reset();
  }

  if (!abstract_mdp_) {
    printf("[LAO Planner]: Environment is not defined\n");
    return false;
  }

  PlannerState start_state(start_state_id_,
                           abstract_mdp_->GetGoalHeuristic(start_state_id_), -1);
  state_hasher_.InsertState(start_state, start_state_id_);

  bool exists_non_terminal_states = true;
  planner_stats_.expansions = 0;
  planner_stats_.num_backups = 0;
  high_res_clock::time_point begin_time = high_res_clock::now();

  double total_residual = std::numeric_limits<double>::max();

  while (exists_non_terminal_states ||
         total_residual > planner_params_.max_allowed_bellman_residual) {
    // Reset residual to 0 for this iteration.
    total_residual = 0;

    if (planner_stats_.expansions > planner_params_.max_expansions) {
      printf("[LAO Planner]: Exceeded max expansion. Use current policy at your own risk.\n");
      break;
    }

    auto current_time = high_res_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>
                        (current_time - begin_time);

    if (elapsed_time.count() > planner_params_.max_plan_time) {
      printf("[LAO Planner]: Exceeded max plan time. Use current policy at your own risk.\n");
      break;
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
      PlannerState s = state_hasher_.GetState(dfs_traversal[ii]);

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

        // printf("Succs:");
        // for (int ii = 0; ii < succ_state_ids_map.size(); ++ii)
        // {
        // printf("Edge: %d", action_ids[ii]);
        // for (int jj = 0; jj < succ_state_ids_map[ii].size(); ++jj)
        // {
        // printf("State: %d: %f %f", succ_state_ids_map[ii][jj], succ_state_probabilities_map[ii][jj], action_costs_map[ii][jj]);
        // }
        // }

        for (size_t ii = 0; ii < action_ids.size(); ++ii) {
          for (size_t jj = 0; jj < succ_state_ids_map[ii].size(); ++jj) {
            const int succ_id = succ_state_ids_map[ii][jj];

            // Skip successors that have already been generated
            if (state_hasher_.Exists(succ_id)) {
              continue;
            }

            PlannerState succ_state(succ_state_ids_map[ii][jj],
                                    abstract_mdp_->GetGoalHeuristic(succ_state_ids_map[ii][jj]), s.state_id);
            state_hasher_.InsertState(succ_state, succ_id);
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
          PlannerState succ_state = state_hasher_.GetState(s.succ_state_ids_map[jj][kk]);
          expected_cost += ((succ_state.v + s.action_costs_map[jj][kk]) *
                            s.succ_state_probabilities_map[jj][kk]);
        }

        if (expected_cost < min_expected_cost) {
          min_expected_cost = expected_cost;
          best_idx = jj;
        }
      }


      ++planner_stats_.num_backups;

      // Add residual(s) to the total residual.
      total_residual += fabs(s.v - min_expected_cost);

      assert(best_idx != -1);
      s.best_action_id = s.action_ids[best_idx];
      s.best_vec_idx = best_idx;
      s.v = min_expected_cost;

      // Update policy map.
      policy_map_[s.state_id] = s.best_action_id;

      // Update the state in PlannerStateMap
      state_hasher_.UpdateState(s);
    }
  }

  auto end_time = high_res_clock::now();
  planner_stats_.time =
    std::chrono::duration_cast<std::chrono::duration<double>>
    (end_time - begin_time).count();
  planner_stats_.cost = state_hasher_.GetState(start_state_id_).v;

  // Reconstruct path
  printf("[LAO Planner]: LAO* done, reconstructing optimal policy\n");

  // DFS traversal of the final solution graph after convergence is the optimal
  // policy.
  std::vector<int> dfs_traversal;
  DFSTraversal(&dfs_traversal);
  printf("[LAO Planner]: Finished econstructing optimal policy\n");

  for (size_t ii = 0; ii < dfs_traversal.size(); ++ii) {
    PlannerState s = state_hasher_.GetState(dfs_traversal[ii]);

    if (s.best_action_id != -1) {
      optimal_policy_map_[s.state_id] = s.best_action_id;
    }
  }

  // SolutionValueIteration();
  // ReconstructOptimisticPath(state_ids, action_ids);
  // ReconstructMostLikelyPath(state_ids, action_ids);
  // PrintPlannerStateMap();
  return true;
}

template <class AbstractMDP>
void LAOPlanner<AbstractMDP>::SolutionValueIteration() {
  // Get the DFS traversal of the best partial solution graph
  std::vector<int> dfs_traversal;
  DFSTraversal(&dfs_traversal);
  const int max_iter = 0;
  const double error_tol = 1e-3;
  double error = std::numeric_limits<double>::max();
  int iter = 0;

  while (error > error_tol && iter < max_iter) {
    iter++;
    error = 0.0;

    for (size_t ii = 0; ii < dfs_traversal.size(); ++ii) {
      PlannerState s = state_hasher_.GetState(dfs_traversal[ii]);
      // Update V-values: V(s) = min_{a\in A}  \sum_{s'} (c(s,a,s') + V(s')) * P(s'|s,a)
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
          PlannerState succ_state = state_hasher_.GetState(s.succ_state_ids_map[jj][kk]);
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

      // Update the state in PlannerStateMap
      state_hasher_.UpdateState(s);
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
} // namespace sbpl
