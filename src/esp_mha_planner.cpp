/*
 * Copyright (c) 2016, Venkatraman Narayanan
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
#include <esp_planner/esp_mha_planner.h>
#include <esp_planner/edge_selector_ssp.h>
#include <esp_planner/lao_planner.h>

#include <boost/math/distributions.hpp>

namespace {
// Maximum time allowed for computing an LAO policy. Note that this time could
// be exceeded if it takes longer to compute an action for the start state.
constexpr double kMaxLAOPolicyPlanningTime = 0.01; // s

// Should a queue be "retired" afer it has generated a state with 0 heuristic?
constexpr bool kUseDTSRetirement = true;

// Which queues should disallow sharing to them.
int probability_queue = 0;
}

using namespace boost::math;
using namespace std;
using namespace mha_planner;

ESPPlanner::ESPPlanner(EnvironmentESP *environment,
                       int num_heuristics,
                       bool bSearchForward) :
  params(0.0),
  num_heuristics(num_heuristics) {
  probability_queue = num_heuristics - 1;
  bforwardsearch = bSearchForward;
  environment_wrapper_.reset(new EnvWrapper(environment));
  replan_number = 0;

  heaps.resize(num_heuristics);
  incons.resize(num_heuristics);
  states.resize(num_heuristics);

  best_h_states.resize(num_heuristics);

  const int max_edge_cost = 60;  // TODO: Get from environment.
  max_heur_dec.resize(num_heuristics, max_edge_cost);

  // DTS
  alpha.resize(num_heuristics, 1.0);
  beta.resize(num_heuristics, 1.0);
  betaC = 3;
  gsl_rng_env_setup();
  gsl_rand_T = gsl_rng_default;
  gsl_rand = gsl_rng_alloc(gsl_rand_T);

  goal_state_id = -1;
  start_state_id = -1;
}

ESPPlanner::~ESPPlanner() {
  gsl_rng_free(gsl_rand);
}

ESPState *ESPPlanner::GetState(int q_id, int id) {
  assert(q_id < num_heuristics);

  //if this stateID is out of bounds of our state vector then grow the list
  if (id >= int(states[q_id].size())) {
    for (int i = states[q_id].size(); i <= id; i++) {
      states[q_id].push_back(NULL);
    }
  }

  //if we have never seen this state then create one
  if (states[q_id][id] == NULL) {
    states[q_id][id] = new ESPState();
    states[q_id][id]->id = id;
    states[q_id][id]->replan_number = -1;
    states[q_id][id]->q_id = q_id;
  }

  //initialize the state if it hasn't been for this call to replan
  ESPState *s = states[q_id][id];

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
    if (q_id == 0) {
      if (bforwardsearch) {
        s->h = environment_wrapper_->GetGoalHeuristic(s->id);
      } else {
        s->h = environment_wrapper_->GetStartHeuristic(s->id);
      }
    } else {
      // const double prob = environment_wrapper_->GetStateProbability(s->id);
      // s->h = static_cast<int>(10000.0 * (1.0 - prob));
      // s->h = static_cast<int>((2.0 - 1.0 * prob)  * 0.5 *
      //                         static_cast<double>(environment_wrapper_->GetGoalHeuristic(s->id)));
      s->h = environment_wrapper_->GetGoalHeuristic(q_id, s->id);
    }
  }

  return s;
}

BestHState *ESPPlanner::GetBestHState(int q_id, int id) {
  assert(q_id < num_heuristics);

  //if this stateID is out of bounds of our state vector then grow the list
  if (id >= int(best_h_states[q_id].size())) {
    for (int i = best_h_states[q_id].size(); i <= id; i++) {
      best_h_states[q_id].push_back(NULL);
    }
  }

  //if we have never seen this state then create one
  if (best_h_states[q_id][id] == NULL) {
    best_h_states[q_id][id] = new BestHState();
    best_h_states[q_id][id]->id = id;
    best_h_states[q_id][id]->heapindex = 0;
  }

  return best_h_states[q_id][id];
}

void ESPPlanner::ExpandState(int q_id, ESPState *parent) {
  bool print = false; //parent->id == 12352;

  if (print) {
    printf("expand %d, q_id: %d\n", parent->id, q_id);
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

  //DTS
  int prev_best_h = queue_best_h_dts[q_id];

  // SMHA sharing.
  for (int i = 0; i < (int)children.size(); i++) {
    //printf("  succ %d\n",children[i]);
    vector<int> equiv_wrapper_ids = environment_wrapper_->AllSubsetWrapperIDs(
                                      children[i]);
    bool prune = false;

    for (const int equiv_wrapper_id : equiv_wrapper_ids) {
      for (int k = 0; k < num_heuristics; ++k) {
        // Don't check subset condition if we are expanding from anchor and
        // the state is closed inadmissibly.
        if (q_id == 0 && k != 0) {
          continue;
        }

        ESPState *child = GetState(k, equiv_wrapper_id);
        if (child->iteration_closed == search_iteration) {
          prune = true;
          break;
        }
      }

      if (prune) {
        break;
      }
    }

    if (prune) {
      continue;
    }

    for (int j = 0; j < num_heuristics; ++j) {
      // if (use_anchor && q_id == 0 && j != 0) continue;
      // Don't add to the unshared queue unless we are expanding from that
      // queue.
      if (j == probability_queue && q_id != j) {
        continue;
      }


      ESPState *child = GetState(j, children[i]);

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
        putStateInHeap(j, child);
      }

      //Meta-A*
      if (meta_search_type == MetaSearchType::META_A_STAR) {
        BestHState *best_h_state = GetBestHState(j, child->id);
        CKey key;
        // TODO: fix it.
        // key.key[0] = static_cast<int>(10000.0 * environment_wrapper_->GetStateProbability(child->id));
        key.key[0] = child->h;

        // Assuming never need to update h-values once computed
        if (best_h_state->heapindex == 0) {
          //printf("Inserting in %d with %d\n", j, key.key[0]);
          queue_best_h_meta_heaps[j].insertheap(best_h_state, key);
        }

        // TODO: Remove this after verifying it works
        // key = queue_best_h_meta_heaps[j].getminkeyheap();
        // int best_h = key.key[0];
        // assert(best_h >= 0);
      }

      //if (best_h == 0)
      //ROS_ERROR("meta queue %d is done",j);
      //DTS
      if (child->h < queue_best_h_dts[j]) {
        queue_best_h_dts[j] = child->h;

        if (queue_best_h_dts[j] == 0) {
          printf("dts queue %d is done\n", j);
          //std::cin.get();
        }

        assert(queue_best_h_dts[j] >= 0);
      }
    }
  }

  /*
     ROS_INFO("queue %d expand",q_id);
     for(int i=0; i<(int)children.size(); i++){
     ESPState* child = GetState(q_id, children[i]);
     printf("child %d has heur %d\n",child->id,child->h);
     }
     */
  //std::cin.get();

  // Meta-A*
  queue_expands[q_id]++;

  if (meta_search_type == MetaSearchType::META_A_STAR) {
    BestHState *best_h_state = GetBestHState(q_id, parent->id);
    queue_best_h_meta_heaps[q_id].deleteheap(best_h_state);

    // delete from the other queues as well if SMHA.
    if (planner_type == mha_planner::PlannerType::SMHA) {
      for (int j = 0; j < num_heuristics; ++j) {
        if (j != q_id) {
          if (j == probability_queue) {
            continue;
          }

          BestHState *best_h_state_to_del = GetBestHState(j, parent->id);
          queue_best_h_meta_heaps[j].deleteheap(best_h_state_to_del);
        }
      }
    }
  }

  // DTS
  double reward = 0;

  if (queue_best_h_dts[q_id] < prev_best_h) {
    reward = 1;
  } else {
    //ROS_ERROR("queue %d got no reward!",q_id);
    //std::cin.get();
  }

  if (alpha[q_id] + beta[q_id] < betaC) {
    alpha[q_id] = alpha[q_id] + reward;
    beta[q_id] = beta[q_id] + (1 - reward);
  } else {
    alpha[q_id] = (alpha[q_id] + reward) * betaC / (betaC + 1);
    beta[q_id] = (beta[q_id] + (1 - reward)) * betaC / (betaC + 1);
  }

  //TEMPORARY CODE TO FIND THE BIGGEST HEURISTIC DROPS
  /*
     ESPState* p = GetState(q_id, parent->id);
     for(int i=0; i<(int)children.size(); i++){
     ESPState* child = GetState(q_id, children[i]);
     int dec = p->h - child->h;
     if(dec > max_heur_dec[q_id]){
     max_heur_dec[q_id] = dec;
     printf("a bigger h decrease %d for queue %d (parent=%d child=%d)\n",dec,q_id,p->h,child->h);
     }
     }
     */
  //TEMPORARY CODE TO FIND THE BIGGEST HEURISTIC DROPS

}

bool ESPPlanner::UpdateGoal(ESPState *state) {
  bool goal_updated = false;

  if (goal_state_id == state->id &&
      state->g < goal_state->g) {
    printf("UPDATING GOAL!!!\n");
    goal_state->g = state->g;
    goal_state->best_parent = state->best_parent;
    goal_updated = true;
  }

  return goal_updated;
}

void ESPPlanner::putStateInHeap(int q_id, ESPState *state) {
  if (UpdateGoal(state)) {
    return;
  }

  bool print = false; //state->id == 12352;

  //we only allow one expansion per search iteration
  //so insert into heap if not closed yet
  if (state->iteration_closed != search_iteration) {
    if (print) {
      printf("put state in open\n");
    }

    CKey key;


    switch (mha_type) {

    case mha_planner::MHAType::ORIGINAL: {
      key.key[0] = long(state->g + int(inflation_eps * state->h));
      break;
    }

    case mha_planner::MHAType::PLUS:
    case mha_planner::MHAType::UNCONSTRAINED: {
      if (q_id == 0) {
        key.key[0] = long(state->g + int(inflation_eps * state->h));
      } else {
        key.key[0] = long(state->h);
        // key.key[1] = long(state->g);
        key.key[1] = long(state->g + int(inflation_eps *
                                         environment_wrapper_->GetGoalHeuristic(state->id)));
      }

      break;
    }

    case mha_planner::MHAType::FOCAL: {
      if (q_id == 0) {
        key.key[0] = long(state->g + state->h);
      } else {
        key.key[0] = long(state->h);
        key.key[1] = long(state->g + environment_wrapper_->GetGoalHeuristic(
                            state->id));
      }

      break;
    }

    case mha_planner::MHAType::GBFS: {
      key.key[0] = long(state->h);
      key.key[1] = long(state->g);
      break;
    }
    }

    //if the state is already in the heap, just update its priority
    if (state->heapindex != 0) {
      heaps[q_id].updateheap(state, key);
    } else { //otherwise add it to the heap
      heaps[q_id].insertheap(state, key);
    }
  }
  //if the state has already been expanded once for this iteration
  //then add it to the incons list so we can keep track of states
  //that we know we have better costs for
  else if (!state->in_incons) {
    if (print) {
      printf("put state in incons\n");
    }

    incons[q_id].push_back(state);
    state->in_incons = true;
  }
}

//returns 1 if the solution is found, 0 if the solution does not exist and 2 if it ran out of time
int ESPPlanner::ImprovePath() {

  //if the goal <= min key in any list whose min key <= min key0 * w2

  //expand states until done
  expands = 0;
  bool spin_again = false;
  int q_id = 0;
  CKey min_key = heaps[0].getminkeyheap();


  int anchor_val = min_key.key[0];
  int valid_path_idx = -1;
  int valid_goal_wrapper_id = -1;

  while (!heaps[0].emptyheap() &&
         min_key.key[0] < INFINITECOST &&
         !outOfTime()) {


    // Termination for different planners
    bool terminate = true;

    switch (mha_type) {
    case mha_planner::MHAType::ORIGINAL: {

      if ((goal_state->g > int(anchor_eps * inflation_eps * anchor_val)) &&
          (goal_state->v > anchor_eps * inflation_eps * anchor_val)) {
        terminate = false;
      }

      break;
    }

    case mha_planner::MHAType::PLUS:
    case mha_planner::MHAType::UNCONSTRAINED: {

      if ((goal_state->g > anchor_val) && (goal_state->v > anchor_val)) {
        terminate = false;
      }

      break;
    }

    case mha_planner::MHAType::FOCAL: {

      if ((goal_state->g > int(inflation_eps * anchor_val)) &&
          (goal_state->v > inflation_eps * anchor_val)) {
        terminate = false;
      }

      break;
    }

    case mha_planner::MHAType::GBFS: {

      if ((goal_state->g == INFINITECOST) && (goal_state->v == INFINITECOST)) {
        terminate = false;
      }

      break;
    }
    }

    if (terminate == true) {
      break;
    }


    // if (goal_state->g < INFINITECOST) {
    //   printf("GOAL WAS SEEN!!!\n");
    //   printf("HEAP 0 Size: %d\n", heaps[0].currentsize);
    //   //std::cin.get();
    // }

    if (!spin_again || meta_search_type != mha_planner::MetaSearchType::DTS) {
      q_id = GetBestHeuristicID();
      // queue_expands[q_id]++;
      //printf("chose queue %d\n",q_id);
    } else {
      //printf("spin again %d\n",q_id);
    }


    ESPState *state;

    CKey anchor_min_key = heaps[0].getminkeyheap();
    CKey best_q_min_key = heaps[q_id].getminkeyheap();

    switch (mha_type) {
    case mha_planner::MHAType::ORIGINAL: {
      // MHA - original
      anchor_val = max(anchor_val, int(anchor_min_key.key[0]));

      // Note: The alternative would be to find a q_id whose min_key is within the suboptimality bound.
      if (best_q_min_key.key[0] > anchor_eps * anchor_val) {
        printf("Anchors aweigh! chosen queue (%d) has min key %ld and anchor has min key %d\n",
               q_id, best_q_min_key.key[0], anchor_val);
        //std::cin.get();
        q_id = 0;
      } else if (q_id != 0) {
        state = (ESPState *)heaps[q_id].getminheap();
      }

      break;
    }

    case mha_planner::MHAType::PLUS: {
      // Improved MHA* - MHA*++ (g+h < g_anchor + w*h_anchor)
      state = (ESPState *)heaps[q_id].getminheap();
      ESPState *anchor_state = GetState(0, state->id);
      anchor_val = max(anchor_val, int(anchor_min_key.key[0]));
      int uhs_val = anchor_state->g + anchor_state->h;
      bool mha_lite_anchor = uhs_val > anchor_val;

      if (mha_lite_anchor) {
        printf("Anchor state ID:%d   G:%d    H:%d\n", anchor_state->id,
               anchor_state->g, anchor_state->h);
        printf("Anchors aweigh! chosen queue (%d) has f-val %d, anchor-h %d, minkey %ld, and anchor has min key %d\n",
               q_id, anchor_state->g + anchor_state->h,
               int(inflation_eps * anchor_state->h), best_q_min_key.key[0],
               anchor_val);
        //std::cin.get();

        // Get best state from eps-focal list. There will be atleast one state because FOCAL is a subset of
        // EPS-FOCAL and it contains the anchor state for sure.
        CKey best_key;
        best_key.SetKeytoInfinity();

        for (int kk = 1; kk <= heaps[0].currentsize; ++kk) {
          ESPState *sa = (ESPState *)heaps[0].heap[kk].heapstate;

          int uhs_val = int(sa->g + sa->h);

          if (uhs_val > anchor_val) {
            continue;  // Not in EPS-FOCAL
          }

          ESPState *uhs_state = GetState(q_id, sa->id);

          // Skip this state if it was already expanded inadmissibly
          if (uhs_state->iteration_closed == search_iteration) {
            continue;
          }

          // Skip this state if it is not in the heap.
          if (uhs_state->heapindex == 0) {
            continue;
          }

          CKey key = heaps[q_id].getkeyheap(uhs_state);

          if (key < best_key) {
            state = uhs_state;
            best_key = key;
          }
        }
      }

      break;
    }

    case mha_planner::MHAType::FOCAL: {
      // Improved MHA* - Focal-MHA* (g+h < w*(g_anchor + h_anchor))
      state = (ESPState *)heaps[q_id].getminheap();
      ESPState *anchor_state = GetState(0, state->id);
      anchor_val = max(anchor_val, int(anchor_min_key.key[0]));
      int uhs_val = anchor_state->g + anchor_state->h;
      bool mha_lite_anchor = uhs_val > inflation_eps * anchor_val;

      if (mha_lite_anchor) {
        printf("Anchor state ID:%d   G:%d    H:%d\n", anchor_state->id,
               anchor_state->g, anchor_state->h);
        printf("Anchors aweigh! chosen queue (%d) has f-val %d, anchor-h %d, minkey %ld, and anchor has min key %d\n",
               q_id, anchor_state->g + anchor_state->h,
               int(inflation_eps * anchor_state->h), best_q_min_key.key[0],
               anchor_val);
        //std::cin.get();

        // Get best state from eps-focal list. There will be atleast one state because FOCAL is a subset of
        // EPS-FOCAL and it contains the anchor state for sure.
        CKey best_key;
        best_key.SetKeytoInfinity();

        for (int kk = 1; kk <= heaps[0].currentsize; ++kk) {
          ESPState *sa = (ESPState *)heaps[0].heap[kk].heapstate;

          int uhs_val = int(sa->g + sa->h);

          if (uhs_val > inflation_eps * anchor_val) {
            continue;  // Not in EPS-FOCAL
          }

          ESPState *uhs_state = GetState(q_id, sa->id);

          // Skip this state if it was already expanded inadmissibly
          if (uhs_state->iteration_closed == search_iteration) {
            continue;
          }

          // Skip this state if it is not in the heap.
          if (uhs_state->heapindex == 0) {
            continue;
          }

          CKey key = heaps[q_id].getkeyheap(uhs_state);

          if (key < best_key) {
            state = uhs_state;
            best_key = key;
          }
        }
      }

      break;
    }

    case mha_planner::MHAType::UNCONSTRAINED: {
      state = (ESPState *)heaps[q_id].getminheap();

      // We can't let any of the inadmissible searches expand the goal state!!!
      if (state->id == goal_state_id) {
        // We'll remove the goal, get the next best state and put back the goal state in.
        // TODO: probably can be done more efficiently
        ESPState *goal_state_q_id = state;
        CKey goal_q_min_key = heaps[q_id].getminkeyheap();
        heaps[q_id].deleteheap(goal_state_q_id);
        // TODO: need to assert heap in not empty after removing goal
        state = (ESPState *)heaps[q_id].getminheap();
        heaps[q_id].insertheap(goal_state_q_id, goal_q_min_key);
      }

      anchor_val = max(anchor_val, int(anchor_min_key.key[0]));
      break;
    }

    case mha_planner::MHAType::GBFS: {
      // GBFS will never expand the goal state because it doesn't care about bounds
      state = (ESPState *)heaps[q_id].getminheap();
    }
    }

    bool print = false;

    if (print) {
      for (int ii = 0; ii < num_heuristics; ++ii) {
        printf("%d ", heaps[ii].currentsize);
      }

      printf("\n");
      CKey best_q_min_key = heaps[q_id].getminkeyheap();
      printf("Q_id: %d, Minkey: %ld\n", q_id, best_q_min_key.key[0]);
    }

    //checkHeaps("before delete heap");

    //get the state
    if (q_id == 0) {
      state = (ESPState *)heaps[q_id].deleteminheap();
    } else {
      heaps[q_id].deleteheap(state);
    }

    // delete from the other queues as well if SMHA.
    if (planner_type == mha_planner::PlannerType::SMHA) {
      // if (use_anchor && q_id==0) break;
      for (int j = 0; j < num_heuristics; ++j) {
        //TODO(Venkat): don't delete from anchor
        // if (use_anchor && j==0) continue;
        if (j != q_id) {
          if (j == probability_queue) {
            continue;
          }

          ESPState *state_to_delete = GetState(j, state->id);

          if (state_to_delete->iteration_closed == search_iteration) {
            continue;  // We already removed this state from the inadmissible queue
          }

          heaps[j].deleteheap(state_to_delete);
        }
      }
    }

    //checkHeaps("after delete heap");

    // GBFS (fast-downward) allows re-expansions
    if (mha_type != mha_planner::MHAType::GBFS && state->v == state->g) {
      printf("ERROR: consistent state is being expanded\n");
      printf("id=%d, q_id=%d, v=%d g=%d\n",
             state->id, state->q_id, state->v, state->g);
      //throw new SBPL_Exception(); //TODO: SMHA-specific
    }

    //mark the state as expanded
    if (planner_type == mha_planner::PlannerType::IMHA) {
      state->v = state->g;
      state->expanded_best_parent = state->best_parent;
      state->iteration_closed = search_iteration;
    } else {
      // if (use_anchor && q_id==0) break;
      for (int j = 0; j < num_heuristics; j++) {
        // if (use_anchor && q_id == 0 && j != 0) {
        if (j == probability_queue && q_id != j) {
          continue;
        }

        if (mha_type == mha_planner::MHAType::GBFS) {
          if (use_anchor && q_id != 0 && j == 0) {
            continue;  // Don't mark an anchor state as expanded if we are expanding from an inadmissible search
          }
        }

        ESPState *tmp_state = GetState(j, state->id);
        tmp_state->v = tmp_state->g;
        tmp_state->expanded_best_parent =
          tmp_state->best_parent; //TODO--verify this with Mike
        tmp_state->iteration_closed = search_iteration;
      }
    }

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
      valid_path_idx = RunEvaluation(&valid_goal_wrapper_id);
      // Reset meta-methods.
      queue_best_h_dts.clear();
      queue_best_h_dts.resize(num_heuristics, INFINITECOST);

      if (valid_path_idx != -1) {
        // TODO: check current termination for weighted versions.
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

      // If the state about to be expanded traverses some edges already known
      // to be invalid, skip it.
      bool is_feasible = !(environment_wrapper_->WrapperContainsInvalidEdge(state->id, invalid_edges_));
      if (!is_subset && is_feasible) {
        ExpandState(q_id, state);
      }
    }

    if (expands % 100000 == 0) {
      printf("expands so far=%u\n", expands);
    }

    //get the min key for the next iteration
    min_key = heaps[0].getminkeyheap();
    anchor_val = max(anchor_val, int(min_key.key[0]));
    // printf("min_key =%d\n",min_key.key[0]);
    // printf("anchor_val =%d\n",anchor_val);
  }

  if (valid_path_idx != -1) {
    // Hack.
    ESPState *temp_goal_state;

    for (int ii = 0; ii < num_heuristics; ++ii) {
      temp_goal_state = GetState(ii, valid_goal_wrapper_id);

      if (temp_goal_state->expanded_best_parent != NULL) {
        break;
      }
    }

    if (temp_goal_state->expanded_best_parent == NULL) {
      printf("The goal state doesn't have a best expanded parent!!\n");
    }

    *goal_state = *temp_goal_state;
    goal_state->id = goal_state_id;
  }

  search_expands += expands;

  if (goal_state->v < goal_state->g) {
    goal_state->g = goal_state->v;
    goal_state->best_parent = goal_state->expanded_best_parent;
  }

  if (goal_state->g == INFINITECOST && (heaps[0].emptyheap() ||
                                        min_key.key[0] >= INFINITECOST)) {
    return 0;  //solution does not exists
  }

  if (!heaps[0].emptyheap() && goal_state->g > inflation_eps * anchor_val) {
    return 2;  //search exited because it ran out of time
  }

  printf("search exited with a solution for eps=%.3f\n", inflation_eps);

  if (goal_state->g < goal_state->v) {
    goal_state->expanded_best_parent = goal_state->best_parent;
    goal_state->v = goal_state->g;
  }

  return 1;
}

void ESPPlanner::checkHeaps(string msg) {
  if (planner_type == mha_planner::PlannerType::IMHA) {
    return;
  }

  bool sameSizes = true;

  for (int i = 1; i < num_heuristics; i++) {
    if (heaps[0].currentsize != heaps[i].currentsize) {
      sameSizes = false;
      printf("%s\n", msg.c_str());
      printf("heap[0] has size %d and heap[%d] has size %d\n", heaps[0].currentsize,
             i, heaps[i].currentsize);
      std::cin.get();
    }
  }

  if (sameSizes) {
    return;
  }

  for (int i = 1; i <= heaps[0].currentsize; ++i) {
    ESPState *state = (ESPState *)heaps[0].heap[i].heapstate;

    for (int j = 1; j < num_heuristics; j++) {
      bool found = false;

      for (int k = 1; k <= heaps[j].currentsize; ++k) {
        ESPState *state2 = (ESPState *)heaps[j].heap[k].heapstate;

        if (state->id == state2->id) {
          found = true;

          if (state->g != state2->g ||
              state->v != state2->v ||
              state->best_parent != state2->best_parent ||
              state->expanded_best_parent != state2->expanded_best_parent) {
            printf("%s\n", msg.c_str());
            printf("state %d found in queues 0 and %d but state internals didn't match\n",
                   state->id, j);
            printf("heap 0: g=%d v=%d parent=%p expanded_parent=%p\n",
                   state->g, state->v, state->best_parent, state->expanded_best_parent);
            printf("heap %d: g=%d v=%d parent=%p expanded_parent=%p\n", j,
                   state2->g, state2->v, state2->best_parent, state2->expanded_best_parent);
            std::cin.get();
          }

          break;
        }
      }

      if (!found) {
        printf("%s", msg.c_str());
        printf("heap[0] has state %d and heap[%d] doesn't\n", state->id, j);
        std::cin.get();
      }
    }
  }

}

vector<int> ESPPlanner::GetSearchPath(ESPState *end_state, int &solcost) {
  vector<int> SuccIDV;
  vector<int> CostV;
  vector<int> wholePathIds;
  vector<double> ProbV;
  vector<double> TimesV;
  vector<int> EdgeGroupsV;

  ESPState *state;
  ESPState *final_state;

  if (bforwardsearch) {
    state = GetState(end_state->q_id, end_state->id);
    final_state = start_state;
  } else {
    state = start_state;
    final_state = GetState(end_state->q_id, end_state->id);
  }

  wholePathIds.push_back(state->id);
  solcost = 0;

  while (state->id != final_state->id) {
    if (state->expanded_best_parent == NULL) {
      printf("a state (%d) along the path has no parent!\n", state->id);
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
  lao_planner.Plan(LAOPlannerParams::ParamsForGreedyOneStepPolicy());

  auto policy_map = lao_planner.GetPolicyMap();
  // printf("Policy for Edge Evaluation\n");

  // for (const auto &element : policy_map) {
  //   const int state_id = element.first;
  //   const SSPState state = ssp->state_hasher_.GetState(state_id);
  //   cout << state << endl << "Action: " << element.second << endl << endl;
  // }

  // Actually run the policy.
  SSPState current_state = start_state;
  const double kBoundForTermination = 1e-3;
  int best_valid_path_idx = 0;

  while (ssp->GetSuboptimalityBound(current_state) > kBoundForTermination) {
    // Best action to execute.
    const int current_state_id = ssp->state_hasher_.GetStateID(current_state);


    auto it = policy_map.find(current_state_id);

    if (it == policy_map.end()) {
      // cout << "ERROR: We do not have a policy for state " << current_state_id << endl
      //      <<
      //      current_state << endl;

      start_id = ssp->state_hasher_.GetStateIDForceful(current_state);
      lao_planner = LAOPlanner<EdgeSelectorSSP>(ssp);
      lao_planner.SetStart(start_id);
      lao_planner.Plan(LAOPlannerParams::ParamsForGreedyOneStepPolicy());
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
      valid_edges_.insert(edge_to_evaluate);
    } else {
      current_state.invalid_bits.set(edge_id, true);
      invalid_edges_.insert(edge_to_evaluate);
      printf("Edge failed %d:  (%d  %d) \n", edge_id, edge_to_evaluate.first,
             edge_to_evaluate.second);
    }

    best_valid_path_idx = ssp->GetBestValidPathIdx(current_state);
    // printf("Current best path idx is %d\n", best_valid_path_idx);
    // cout << "Current state " << current_state.to_string() << endl;
    // cout << "Current bound " << ssp->GetSuboptimalityBound(current_state) << endl;
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
      valid_edges_.insert(edge_to_evaluate);
    } else {
      current_state.invalid_bits.set(edge_id, true);
      invalid_edges_.insert(edge_to_evaluate);
    }

    best_valid_path_idx = ssp->GetBestValidPathIdx(current_state);
    printf("Current best path idx is %d\n", best_valid_path_idx);
    cout << "Current state " << current_state.to_string() << endl;
    cout << "Current bound " << ssp->GetSuboptimalityBound(current_state) << endl;
  }


  return best_valid_path_idx;
}

int ESPPlanner::RunEvaluation(int *valid_goal_wrapper_id) {
  *valid_goal_wrapper_id = -1;
  vector<int> path_ids;
  auto possible_paths = GetCurrentSolutionPaths(&path_ids);
  int valid_path_idx = GetTruePathIdx(possible_paths) ;

  if (valid_path_idx != -1) {
    // Mark these paths/goals as evaluated, so that they are considered again in
    // the future.
    int goal_wrapper_id = path_ids[valid_path_idx];
    // evaluated_goal_wrapper_ids_.insert(path_ids.begin(), path_ids.end());
    printf("A feasible solution (%d) has been found!!\n", goal_wrapper_id);
    // Exclude the valid path ID so that it can be obtained later.
    evaluated_goal_wrapper_ids_.erase(goal_wrapper_id);
    *valid_goal_wrapper_id = goal_wrapper_id;
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
  reconstructTime = 0;
  totalTime = 0;

  //clear open list, incons list, and stats list
  for (int ii = 0; ii < num_heuristics; ++ii) {
    heaps[ii].makeemptyheap();
    incons[ii].clear();
  }

  stats.clear();

  //set MHA parameters
  inflation_eps = params.initial_eps;
  anchor_eps = params.initial_eps;
  // planner_type = params.planner_type;
  // meta_search_type = params.meta_search_type;
  // mha_type = params.mha_type;
  meta_search_type = mha_planner::MetaSearchType::DTS;
  planner_type = mha_planner::PlannerType::SMHA;
  mha_type = mha_planner::MHAType::FOCAL;
  use_anchor = true;
  // Reset the Meta-A* state variables
  queue_expands.clear();
  queue_best_h_dts.clear();

  queue_expands.resize(num_heuristics, 0);
  queue_best_h_dts.resize(num_heuristics, INFINITECOST);

  queue_best_h_meta_heaps.clear();

  if (meta_search_type == MetaSearchType::META_A_STAR) {
    queue_best_h_meta_heaps.resize(num_heuristics);

    for (int ii = 0; ii < num_heuristics; ++ii) {
      queue_best_h_meta_heaps[ii].makeemptyheap();
    }
  }

  // Reset the DTS state variables
  for (int i = 0; i < num_heuristics; i++) {
    alpha[i] = 1.0;
    beta[i] = 1.0;
  }

  //print the planner configuration
  if (meta_search_type == mha_planner::MetaSearchType::ROUND_ROBIN) {
    printf("Round Robin ");
  } else if (meta_search_type == mha_planner::MetaSearchType::META_A_STAR) {
    printf("Meta-A* ");
  } else if (meta_search_type == mha_planner::MetaSearchType::DTS) {
    printf("DTS ");
  } else {
    printf("Meta Search approach is unknown!\n");
  }

  if (planner_type == mha_planner::PlannerType::IMHA) {
    printf("IMHA ");
  } else if (planner_type == mha_planner::PlannerType::SMHA) {
    printf("SMHA ");
  } else {
    printf("Planner type is unknown!\n");
  }

  printf("with inflation eps=%f and anchor eps=%f\n", inflation_eps, anchor_eps);

  eps_satisfied = INFINITECOST;

  //TODO: for backward search goal_state is set to the start_state_id
  //and start_state is set to the goal_state_id

  goal_state = GetState(0, goal_state_id);

  for (int ii = 0; ii < num_heuristics; ++ii) {
    //call get state to initialize the start and goal states
    //put start state in the heap
    start_state = GetState(ii, start_state_id);
    start_state->g = 0;
    CKey key;
    key.key[0] = start_state->h;
    heaps[ii].insertheap(start_state, key);

    queue_best_h_dts[ii] = start_state->h;

    if (meta_search_type == MetaSearchType::META_A_STAR) {
      BestHState *best_h_state = GetBestHState(ii, start_state->id);
      CKey h_key;
      h_key.key[0] = start_state->h;
      queue_best_h_meta_heaps[ii].insertheap(best_h_state, h_key);
    }
  }

  start_state = GetState(0, start_state_id);

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
      eps_satisfied = inflation_eps;
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
  pathIds = GetSearchPath(goal_state, PathCost);
  reconstructTime = double(clock() - before_reconstruct) / CLOCKS_PER_SEC;
  totalTime = totalPlanTime + reconstructTime;

  return true;
}

void ESPPlanner::prepareNextSearchIteration() {
  //decrease epsilon
  inflation_eps -= params.dec_eps;

  if (inflation_eps < params.final_eps) {
    inflation_eps = params.final_eps;
  }

  //dump the inconsistent states into the open list
  CKey key;

  for (int ii = 0; ii < num_heuristics; ++ii) {
    while (!incons[ii].empty()) {
      ESPState *s = incons[ii].back();
      incons[ii].pop_back();
      s->in_incons = false;
      key.key[0] = s->g + int(inflation_eps * s->h);
      heaps[ii].insertheap(s, key);
    }

    //recompute priorities for states in OPEN and reorder it
    for (int jj = 1; jj <= heaps[ii].currentsize; ++jj) {
      ESPState *state = (ESPState *)heaps[ii].heap[jj].heapstate;
      heaps[ii].heap[jj].key.key[0] = state->g + int(inflation_eps * state->h);
    }

    heaps[ii].makeheap();
  }

  search_iteration++;
}

int ESPPlanner::GetBestHeuristicID() {
  if (queue_best_h_dts[probability_queue] != 0) {
    return probability_queue;
  }

  // Note: Anchor is skipped for original MHA, but not for lite
  int starting_ind;

  if (mha_type == mha_planner::MHAType::ORIGINAL) {
    if (use_anchor) {
      starting_ind = 1;
    } else {
      starting_ind = 0;
    }
  } else {
    starting_ind = 0;
  }

  if (meta_search_type == mha_planner::MetaSearchType::ROUND_ROBIN) {
    // Round-robin -- Meta-Dijkstra
    // Note: Simply cycling through 0 to num_heuristics does not work because of lazy evaluation
    int best_id = -1;
    int best_priority = INFINITECOST;
    bool print = false;

    for (int ii = starting_ind; ii < num_heuristics; ++ii) {
      int priority = queue_expands[ii];

      if (heaps[ii].getminkeyheap().key[0] >= INFINITECOST) {
        priority = INFINITECOST;
      }

      if (priority < best_priority) {
        best_priority = priority;
        best_id = ii;
      }

      if (print) {
        printf("                      qid=%d g=%d\n", ii,
               queue_expands[ii]);
      }
    }

    if (print) {
      printf("%d\n", best_id);
    }

    assert(best_id != -1);
    return best_id;
  } else if (meta_search_type == mha_planner::MetaSearchType::META_A_STAR) {
    // Meta-A*
    int best_id = -1;
    int best_priority = INFINITECOST;
    bool print = true;
    bool atleast_one_nonzero_heur = false;

    for (int ii = starting_ind; ii < num_heuristics; ++ii) {
      CKey key = queue_best_h_meta_heaps[ii].getminkeyheap();
      int best_h = key.key[0];

      if (best_h != 0) {
        atleast_one_nonzero_heur = true;
        break;
      }
    }

    for (int ii = starting_ind; ii < num_heuristics; ++ii) {
      CKey key = queue_best_h_meta_heaps[ii].getminkeyheap();
      int best_h = key.key[0];

      if (planner_type == mha_planner::PlannerType::SMHA &&
          atleast_one_nonzero_heur && best_h == 0) {
        continue;
      }

      //TODO: remove magic inflation?
      const int meta_heur_inflation = 10;
      const int priority = queue_expands[ii] + meta_heur_inflation * int(
                             best_h / max_heur_dec[ii]);

      if (priority < best_priority) {
        best_priority = priority;
        best_id = ii;
      }

      if (print) {
        printf("                      qid=%d g=%d h=%d f=%d (queue best h=%d)\n", ii,
               queue_expands[ii], best_h / max_heur_dec[ii], priority, best_h);
      }
    }

    if (print) {
      printf("%d\n", best_id);
    }

    assert(best_id != -1);
    return best_id;
  } else if (meta_search_type == mha_planner::MetaSearchType::DTS) {
    // Dynamic Thompson Sampling (DTS)
    //compute a random value from each beta distribution
    bool print = false;
    vector<double> rand_vals(num_heuristics, 0);

    for (int i = starting_ind; i < num_heuristics; i++) {
      // bool retire_queue = kUseDTSRetirement && i == probability_queue;
      if ((kUseDTSRetirement && queue_best_h_dts[i] == 0) ||
          heaps[i].getminkeyheap().key[0] >= INFINITECOST) {
        if (print) {
          printf("%d: done\n", i);
        }

        rand_vals[i] = -1;
        continue;
      }

      //beta_distribution<> dist(alpha[i], beta[i]);
      //double uniformRand = ((double)rand()/(double)RAND_MAX);
      //double betaRand = quantile(dist, uniformRand);
      double betaRand = gsl_ran_beta(gsl_rand, alpha[i], beta[i]);
      double betaMean = alpha[i] / (alpha[i] + beta[i]);

      if (print) {
        printf("%d: alpha=%f beta=%f mean=%f betaRand=%f\n", i, alpha[i], beta[i],
               betaMean, betaRand);
      }

      rand_vals[i] = betaRand;
    }

    //find the best highest random value
    double best_rand = -1;

    for (int i = starting_ind; i < num_heuristics; i++)
      if (rand_vals[i] > best_rand) {
        best_rand = rand_vals[i];
      }

    //because of quantization we can get the exact same random value
    //for multiple queues more often than we'd like
    //especially when beta is very low (we get 1 very easily)
    //or when alpha is very low (we get 0 very easily)
    //in these cases, there is a bias toward the lower index queues
    //because they "improve" best_rand first
    //so when there are multiple queues near the best_rand value,
    //we will choose uniformly at random from them
    vector<int> near_best_rand;

    for (int i = starting_ind; i < num_heuristics; i++)
      if (fabs(best_rand - rand_vals[i]) < 0.0001) {
        near_best_rand.push_back(i);
      }

    //if(near_best_rand.size() > 1)
    //printf("choose uniformly among %lu queues\n",near_best_rand.size());
    int best_id = near_best_rand[rand() % near_best_rand.size()];

    if (print) {
      printf("best=%d\n", best_id);
    }

    //std::cin.get();
    assert(best_id != -1);
    return best_id;
  } else {
    printf("Unsupported meta search method!\n");
    assert(false);
    return -1;
  }
}


//-----------------------------Interface function-----------------------------------------------------

void ESPPlanner::interrupt() {
  interruptFlag = true;
}

int ESPPlanner::replan(vector<int> *solution_stateIDs_V, ReplanParams p) {
  int solcost = 0;
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
  int PathCost = 0;
  bool solnFound = Search(pathIds, PathCost);
  printf("total expands=%d planning time=%.3f reconstruct path time=%.3f total time=%.3f solution cost=%d\n",
         totalExpands, totalPlanTime, reconstructTime, totalTime, goal_state->g);

  for (int i = 0; i < num_heuristics; i++) {
    printf("max heuristic decrease found for queue %d was %d\n", i,
           max_heur_dec[i]);
  }

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

    // Find the queue from which it was expanded.
    ESPState *search_state;

    for (size_t jj = 0; jj < num_heuristics; ++jj) {
      search_state = GetState(jj, all_goal_wrapper_ids[ii]);

      if (search_state->expanded_best_parent != NULL) {
        break;
      }
    }

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
    vector<int> wrapper_ids_path = GetSearchPath(search_state,
                                                 path_cost);
    auto solution_path = environment_wrapper_->ConvertWrapperIDsPathToSBPLPath(
                           wrapper_ids_path);
    solution_path.cost = path_cost;
    solution_paths.push_back(solution_path);
    path_ids->push_back(all_goal_wrapper_ids[ii]);
  }

  return solution_paths;
}


//---------------------------------------------------------------------------------------------------------


void ESPPlanner::get_search_stats(vector<PlannerStats> *s) {
  s->clear();
  s->reserve(stats.size());

  for (unsigned int i = 0; i < stats.size(); i++) {
    s->push_back(stats[i]);
  }
}
