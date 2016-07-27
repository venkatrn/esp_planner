#include <esp_planner/edge_selector_ssp.h>

using namespace sbpl;
using namespace std;

int main(int argc, char ** argv) {
  EdgeSelectorSSP ssp;
  Path p1;
  p1.state_ids = std::vector<int>({0, 1, 2, 3, 4});
  p1.edge_probabilities = std::vector<double>({1, 1, 1, 0.1});
  p1.edge_eval_times = std::vector<double>(p1.edge_probabilities.size(), 1.0);
  p1.cost = 10;

  Path p2;
  p2.state_ids = std::vector<int>({0, 2, 4});
  p2.edge_probabilities = std::vector<double>({1 , 0.5});
  p2.edge_eval_times = std::vector<double>(p2.edge_probabilities.size(), 1.0);
  p2.cost = 30;

  Path p3;
  p3.state_ids = std::vector<int>({0, 5, 2, 4});
  p3.edge_probabilities = std::vector<double>({0.1 , 1, 0.5});
  p3.edge_eval_times = std::vector<double>(p3.edge_probabilities.size(), 1.0);
  p3.cost = 50;

  vector<Path> paths = {p1, p2, p3};
  ssp.SetPaths(paths);
  
  int lb = 0;
  int ub = 0;

  // EdgeStatusMap edge_status_map = {{0,1}};
  // ssp.ComputeBounds(edge_status_map_state, &lb, &ub);

  SSPState ssp_state(3);
  ssp_state.valid_bits.set(1);

  ssp.ComputeBounds(ssp_state, &lb, &ub);
  printf("LB is %d   ;   UB is %d\n", lb, ub);
  return 0;
}
