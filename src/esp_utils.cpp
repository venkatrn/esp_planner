#include <esp_planner/esp_utils.h>
#include <esp_planner/edge_selector_ssp.h>

#include <iostream> 
#include <fstream> 

using namespace std;

namespace sbpl {

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

EnvBGStochastic::EnvBGStochastic(const StochasticGraph& graph) : BGEnvironment(graph) {
  edge_bundle_map_ = get(bo::edge_bundle, graph_);
}

void EnvBGStochastic::GetSuccs(int parent_id, std::vector<int> *succ_ids,
                    std::vector<int> *costs, std::vector<double> *edge_probabilites,
                    std::vector<double> *edge_eval_times,
                    std::vector<int> *edge_groups) {
  succ_ids->clear();
  costs->clear();
  edge_probabilites->clear();
  edge_eval_times->clear();
  IndexMap index_map = get(bo::vertex_index, graph_);
  auto parent_vertex = bo::vertex(parent_id, graph_);
  OutEdgeIterator out_i, out_end;
  Edge e;

  for (std::tie(out_i, out_end) = out_edges(parent_vertex, graph_);
       out_i != out_end; ++out_i) {
    e = *out_i;
    // Vertex src = bo::source(e, graph_);
    const auto& edge_properties = edge_bundle_map_[e];
    Vertex targ = bo::target(e, graph_);
    succ_ids->push_back(index_map[targ]);
    costs->push_back(edge_properties.cost);
    edge_probabilites->push_back(edge_properties.probability);
    edge_eval_times->push_back(edge_properties.evaluation_time);
  }
}

bool EnvBGStochastic::EvaluateEdge(int parent_id, int child_id) {
  return false;
}

int EnvBGStochastic::GetGoalHeuristic(int state_id) {
  return BGEnvironment::GetGoalHeuristic(state_id);
}

} // namespace sbpl
