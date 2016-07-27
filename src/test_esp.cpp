#include <esp_planner/edge_selector_ssp.h>
#include <esp_planner/lao_planner.h>

using namespace sbpl;
using namespace std;

int main(int argc, char **argv) {
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
  p3.state_ids = std::vector<int>({0, 6, 1, 2, 4});
  p3.edge_probabilities = std::vector<double>({1, 1, 0.4, 1});
  p3.edge_eval_times = std::vector<double>(p3.edge_probabilities.size(), 1.0);
  p3.cost = 50;

  Path p4;
  p4.state_ids = std::vector<int>({0, 5, 2, 4});
  p4.edge_probabilities = std::vector<double>({1 , 1, 1});
  p4.edge_eval_times = std::vector<double>(p4.edge_probabilities.size(), 1.0);
  p4.cost = 150;

  vector<Path> paths = {p1, p2, p3, p4};
  std::shared_ptr<EdgeSelectorSSP> ssp(new EdgeSelectorSSP());
  ssp->SetPaths(paths);

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

  printf("Most likely path from executing optimal policy:\n");
  for (size_t ii = 0; ii < expected_path.size(); ++ii) {
    auto ssp_state = ssp->state_hasher_.GetState(expected_path[ii]);
    cout << ssp_state << endl;
  }

  return 0;
}
