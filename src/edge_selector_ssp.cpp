#include <esp_planner/edge_selector_ssp.h>

namespace sbpl {
EdgeSelectorSSP::EdgeSelectorSSP() {

}
void EdgeSelectorSSP::SetPaths(const std::vector<Path> &paths) {
  paths_ = paths;
  simplified_paths_.resize(paths.size());

  for (size_t ii = 0; ii < paths.size(); ++ii) {
    const auto &state_ids = paths[ii].state_ids;
    simplified_paths_[ii].resize(state_ids.size() - 1);

    for (size_t jj = 0; jj < state_ids.size() - 1; ++jj) {
      Edge edge(state_ids[jj], state_ids[jj + 1], paths[ii].edge_probabilities[jj]);
      const int edge_id = edge_hasher_.GetStateIDForceful(edge);
      simplified_paths_[ii][jj] = edge_id;
    }
  }
}
}
