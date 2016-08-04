#include <esp_planner/edge_selector_ssp.h>
#include <esp_planner/lao_planner.h>
#include <esp_planner/esp_utils.h>
#include <esp_planner/esp_planner.h>
#include <ros/package.h>

#include <iostream>
#include <memory>

using namespace sbpl;
using namespace std;

namespace {
// TODO: remove hardcoded path.
const string kDefaultDebugDir =
  "/usr0/home/venkatrn/indigo_workspace/src/esp_planner/visualization/";
}

void test_planner() {

  // (from, to, cost, probability of existence, evaluation time)
  // 
  // 3 lazy paths to goal.
  // StochasticGraph g(6);
  // vector<vector<double>> edges = {
  //   {0, 1, 10, 0.1, 1.3},
  //   {1, 2, 10, 1.0, 1.5},
  //   {2, 3, 10, 1.0, 2.5},
  //   {3, 4, 10, 0.6, 3.5},
  //   {4, 5, 10, 1.0, 0.1},
  //
  //   {0, 5, 20, 0.5, 1.0},
  //
  //   {0, 3, 40, 1.0, 1.0},
  //   {3, 5, 40, 1.0, 1.0}
  // };
  // int goal_id = 5;

  // 17 lazy paths to goal.
  // StochasticGraph g(6);
  // vector<vector<double>> edges = {
  //   {0, 1, 1, 0.1, 1.3},
  //   {1, 2, 1, 0.1, 1.5},
  //   {2, 3, 1, 0.1, 1.5},
  //   {3, 4, 1, 0.1, 3.5},
  //   {4, 5, 1, 0.1, 0.1},
  //
  //   {0, 5, 100, 1.0, 1.0},
  //
  //   {1, 3, 1, 0.1, 1.0},
  //   {1, 4, 1, 0.1, 1.0},
  //   {1, 5, 1, 0.1, 1.0},
  //   {2, 4, 1, 0.1, 1.0},
  //   {2, 5, 1, 0.1, 1.0},
  //   {3, 5, 1, 0.1, 1.0},
  // };
  // int goal_id = 5;

  // 26 lazy paths to goal.
  StochasticGraph g(7);
  vector<vector<double>> edges = {
    {0, 1, 1, 0.5, 1.0},
    {1, 7, 1, 0.5, 1.0},
    {0, 2, 1, 0.5, 1.0},
    {2, 7, 1, 0.5, 1.0},
    {0, 3, 1, 0.5, 1.0},
    {3, 7, 1, 0.5, 1.0},
    {0, 4, 1, 0.5, 1.0},
    {4, 7, 1, 0.5, 1.0},
    {0, 5, 1, 0.5, 1.0},
    {5, 7, 1, 0.5, 1.0},

    {1, 2, 1, 0.5, 1.5},
    {2, 3, 1, 0.5, 2.5},
    {3, 4, 1, 0.5, 3.5},
    {4, 5, 1, 0.5, 0.1},

    {0, 7, 100, 1, 0.1},
  };
  int goal_id = 7;

  vector<vector<int>> heuristics = {
    {0, 0},
    {1, 0},
    {2, 0},
    {3, 0},
    {4, 0},
    {5, 0},
  };

  auto edge_bundle_map = get(bo::edge_bundle, g);

  for (const auto &edge : edges) {
    auto& edge_properties = edge_bundle_map[add_edge(edge[0], edge[1], g).first];
    edge_properties.cost = static_cast<int>(edge[2]);
    edge_properties.probability = edge[3];
    edge_properties.evaluation_time = edge[4];
  }

  for (const auto &heuristic : heuristics) {
    g[heuristic[0]].heuristic = heuristic[1];
  }

  EnvBGStochastic bg_env(g);
  unique_ptr<ESPPlanner> planner(new ESPPlanner(&bg_env, true)); 
  // unique_ptr<SBPLPlanner> planner(new LazyARAPlanner(&bg_env, true)); 
  planner->set_start(0);
  planner->set_goal(goal_id);
  ReplanParams params(1.0);
  params.initial_eps = 1.0;
  params.final_eps = 1.0;
  params.return_first_solution = true;
  // vector<int> solution_ids;
  vector<sbpl::Path> solution_paths;
  int solution_cost = 0;
  planner->replan(&solution_paths, params, &solution_cost);
  printf("Found %d possible paths:\n", static_cast<int>(solution_paths.size()));
  for (size_t ii = 0; ii < solution_paths.size(); ++ii) {
    const auto &solution_path = solution_paths[ii];
    cout << "Cost: " << solution_path.cost << "     ";
    for (size_t jj = 0; jj < solution_path.state_ids.size(); ++jj) {
      cout << solution_path.state_ids[jj] << " "; 
    }
    cout << endl;
  }
  cout << endl;
}

// Common edge in p2 and p3.
void test1(std::shared_ptr<EdgeSelectorSSP> ssp) {
  Path p1;
  p1.state_ids = std::vector<int>({0, 1, 2, 3, 4});
  p1.edge_probabilities = std::vector<double>({1, 0.4, 1, 0.1});
  p1.edge_eval_times = std::vector<double>(p1.edge_probabilities.size(), 1.0);
  p1.cost = 5;

  Path p2;
  p2.state_ids = std::vector<int>({0, 2, 4});
  p2.edge_probabilities = std::vector<double>({1 , 0.9});
  p2.edge_eval_times = std::vector<double>(p2.edge_probabilities.size(), 1.0);
  p2.cost = 30;

  Path p3;
  p3.state_ids = std::vector<int>({0, 6, 10, 2, 4});
  p3.edge_probabilities = std::vector<double>({1, 1, 0.4, 0.9});
  p3.edge_eval_times = std::vector<double>(p3.edge_probabilities.size(), 1.0);
  p3.cost = 50;

  Path p4;
  p4.state_ids = std::vector<int>({0, 5, 8, 4});
  p4.edge_probabilities = std::vector<double>({1 , 1, 1});
  p4.edge_eval_times = std::vector<double>(p4.edge_probabilities.size(), 1.0);
  p4.cost = 150;

  vector<Path> paths = {p1, p2, p3, p4};
  ssp->SetPaths(paths);
}

void test2(std::shared_ptr<EdgeSelectorSSP> ssp) {
  Path p1;
  p1.state_ids = std::vector<int>({0, 1, 2, 3, 4});
  p1.edge_probabilities = std::vector<double>({1, 0.4, 1, 0.1});
  p1.edge_eval_times = std::vector<double>(p1.edge_probabilities.size(), 1.0);
  p1.cost = 5;

  Path p2;
  p2.state_ids = std::vector<int>({0, 2, 4});
  p2.edge_probabilities = std::vector<double>({1 , 0.9});
  p2.edge_eval_times = std::vector<double>(p2.edge_probabilities.size(), 1.0);
  p2.cost = 50;

  Path p3;
  p3.state_ids = std::vector<int>({0, 6, 10, 5, 4});
  p3.edge_probabilities = std::vector<double>({1, 1, 0.4, 0.9});
  p3.edge_eval_times = std::vector<double>(p3.edge_probabilities.size(), 1.0);
  p3.cost = 30;

  Path p4;
  p4.state_ids = std::vector<int>({0, 5, 8, 4});
  p4.edge_probabilities = std::vector<double>({1 , 1, 1});
  p4.edge_eval_times = std::vector<double>(p4.edge_probabilities.size(), 1.0);
  p4.cost = 150;

  vector<Path> paths = {p1, p2, p3, p4};
  ssp->SetPaths(paths);
}

void test3(std::shared_ptr<EdgeSelectorSSP> ssp) {
  const int num_paths = 15;
  vector<Path> paths(num_paths);

  for (int ii = 0; ii < num_paths; ++ii) {
    Path p;
    p.edge_probabilities = {static_cast<double>(ii) / (num_paths - 1), 1.0};
    p.edge_eval_times = {1.0, 1.0};
    p.state_ids = {0, ii + 1, 10000};
    p.cost = 100 * (ii + 1);
    paths[ii] = p;
  }

  ssp->SetPaths(paths);
}

int main(int argc, char **argv) {
  std::shared_ptr<EdgeSelectorSSP> ssp(new EdgeSelectorSSP());
  // test1(ssp);
  // test2(ssp);
  // test3(ssp);
  test_planner();
  return 0;

  const int num_edges = ssp->NumStochasticEdges();
  SSPState start_state(num_edges);
  start_state.suboptimality_bound = ssp->GetSuboptimalityBound(start_state);
  const int start_id = ssp->state_hasher_.GetStateIDForceful(start_state);

  // vector<vector<int>> succs;
  // vector<int> action_ids;
  // vector<vector<double>> costs;
  // vector<vector<double>> probs;
  // ssp->GetSuccs(start_id, &succs, &probs, &action_ids, &costs);
  // cout << "Start: \n" << start_state << endl;
  // for (size_t ii = 0; ii < succs.size(); ++ii) {
  //   printf("Action: %d\n", action_ids[ii]);
  //   for (size_t jj = 0; jj < probs[ii].size(); ++jj) {
  //     printf("Cost: %0.2f, Prob: %f\n", costs[ii][jj], probs[ii][jj]);
  //     cout << ssp->state_hasher_.GetState(succs[ii][jj]);
  //   }
  //   cout << endl;
  // }
  //

  // Exercise planner.
  LAOPlanner<EdgeSelectorSSP> lao_planner(ssp);
  lao_planner.SetStart(start_id);
  vector<int> expected_path;
  vector<int> best_action_ids;
  lao_planner.Plan(&expected_path, &best_action_ids);

  const auto &policy_map = lao_planner.GetPolicyMap();
  printf("Policy for Edge Evaluation\n");

  for (const auto &element : policy_map) {
    const int state_id = element.first;
    const SSPState state = ssp->state_hasher_.GetState(state_id);
    cout << state << endl << "Action: " << element.second << endl << endl;
  }

  printf("Most likely path from executing optimal policy:\n");

  for (size_t ii = 0; ii < expected_path.size(); ++ii) {
    auto ssp_state = ssp->state_hasher_.GetState(expected_path[ii]);
    cout << ssp_state << endl;
  }

  cout << endl;

  printf("Planner Statistics:\n");
  const auto &planner_stats = lao_planner.GetPlannerStats();
  cout << "Time: " << planner_stats.time << " |  Expansions: " <<
       planner_stats.expansions << " |  Cost: " << planner_stats.cost <<
       " |  Backups: " << planner_stats.num_backups << endl;

  // Print the policy and original paths to a DOT file for visualization.
  PrintPolicyAndPathsAsDOTGraph("test1", kDefaultDebugDir, policy_map, *ssp);

  return 0;
}
