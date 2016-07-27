#include <esp_planner/edge_selector_ssp.h>

using namespace sbpl;
using namespace std;

int main(int argc, char ** argv) {
  EdgeSelectorSSP ssp;
  Path p1;
  p1.state_ids = std::vector<int>({0, 1, 2, 3, 4});
  p1.edge_probabilities = std::vector<double>({1, 0.4, 1, 0.1});
  p1.edge_eval_times = std::vector<double>(p1.edge_probabilities.size(), 1.0);
  p1.cost = 5;

  Path p2;
  p2.state_ids = std::vector<int>({0, 2, 4});
  p2.edge_probabilities = std::vector<double>({1 , 0.5});
  p2.edge_eval_times = std::vector<double>(p2.edge_probabilities.size(), 1.0);
  p2.cost = 30;

  Path p3;
  p3.state_ids = std::vector<int>({0, 5, 2, 4});
  p3.edge_probabilities = std::vector<double>({1 , 1, 1});
  p3.edge_eval_times = std::vector<double>(p3.edge_probabilities.size(), 1.0);
  p3.cost = 50;

  vector<Path> paths = {p1, p2, p3};
  ssp.SetPaths(paths);

  const int num_edges = ssp.NumStochasticEdges();
  SSPState start_state(num_edges);
  const int start_id = ssp.state_hasher_.GetStateIDForceful(start_state);
  vector<vector<int>> succs;
  vector<int> action_ids;
  vector<vector<int>> costs;
  vector<vector<double>> probs;
  ssp.GetSuccs(start_id, &succs, &probs, &action_ids, &costs);
  cout << "Start: \n" << start_state << endl;
  for (size_t ii = 0; ii < succs.size(); ++ii) {
    printf("Action: %d\n", action_ids[ii]);
    for (size_t jj = 0; jj < probs[ii].size(); ++jj) {
      printf("Cost: %d, Prob: %f\n", costs[ii][jj], probs[ii][jj]);
      cout << ssp.state_hasher_.GetState(succs[ii][jj]);
    }
    cout << endl;
  }
  return 0;
}
