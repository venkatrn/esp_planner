#include <esp_planner/edge_selector_ssp.h>
#include <esp_planner/lao_planner.h>
#include <ros/package.h>

#include <iostream>
#include <fstream>

using namespace sbpl;
using namespace std;

namespace {
// TODO: remove hardcoded path.
const string kDefaultDebugDir =
  "/usr0/home/venkatrn/indigo_workspace/src/esp_planner/visualization/";
}

void PrintPolicyAndPathsAsDOTGraph(const string &name, const string &dir,
                                   const unordered_map<int, int> &policy_map, const EdgeSelectorSSP &ssp) {
  const string policy_file = dir + "/" + name + "_policy.dot";
  std::ofstream dot_file(policy_file);
  dot_file << "digraph D {\n"
           << "  rankdir=LR\n"
           << "  size=\"4,3\"\n"
           << "  ratio=\"fill\"\n"
           << "  edge[style=\"bold\", labelfontcolor=\"red\"]\n"
           << "  node[shape=\"circle\"]\n";

  for (const auto &element : policy_map) {
    const int state_id = element.first;
    const SSPState state = ssp.state_hasher_.GetState(state_id);
    const int action_id = element.second;
    const Edge edge = ssp.edge_hasher_.GetState(action_id);

    string parent_string = state.to_string();
    // Edge valid outcome
    string valid_string = parent_string;
    string invalid_string = state.to_string();
    valid_string.replace(action_id, 1, 1, '1');
    invalid_string.replace(action_id, 1, 1, '0');
    parent_string = "\"" + parent_string + "\"";
    valid_string = "\"" + valid_string + "\"";
    invalid_string = "\"" + invalid_string + "\"";

    // Best-action label for each node
    dot_file << parent_string
             << "[xlabel=<<font color=\"red\">" << std::to_string(action_id) + "</font>>"
             << "]\n";

    // dot_file << parent_string << " -> " << valid_string
    //   << "[taillabel=\"" << std::to_string(action_id) << "\", label=\"" << "1" << "\"";
    std::stringstream valid_edge_prob;
    valid_edge_prob << fixed << setprecision(2) << edge.probability;
    dot_file << parent_string << " -> " << valid_string
             << "[label=\"" << valid_edge_prob.str() << "\"";

    if (edge.probability >= 0.5) {
      dot_file << ", color=\"black\"";
    } else {
      dot_file << ", color=\"grey\"";
    }

    dot_file << "]\n";

    // Edge invalid outcome
    std::stringstream invalid_edge_prob;
    invalid_edge_prob << fixed << setprecision(2) << (1 - edge.probability);
    dot_file << parent_string << " -> " << invalid_string
             << "[label=\"" << invalid_edge_prob.str() << "\"";

    if (edge.probability < 0.5) {
      dot_file << ", color=\"black\"";
    } else {
      dot_file << ", color=\"grey\"";
    }

    dot_file << "]\n";


  }

  dot_file << "}";

  // Print the paths.
  const string paths_file = dir + "/" + name + "_paths.dot";
  ssp.PrintPathsAsDOTGraph(paths_file);
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
  test3(ssp);

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
