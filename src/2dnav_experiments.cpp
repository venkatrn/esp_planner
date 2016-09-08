#include <esp_planner/environments/environment_nav2D.h>
#include <esp_planner/esp_mha_planner.h>

#include <sbpl/headers.h>

#include <opencv2/core/core.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/filesystem.hpp>

#include <fstream>
#include <memory>
#include <string>

using namespace std;

enum PlannerType {
  INVALID_PLANNER_TYPE = -1,
  PLANNER_TYPE_LAZY_ARA,
  PLANNER_TYPE_ESP,

  NUM_PLANNER_TYPES
};

int group_id = 1;
constexpr double kTimeLimit = 30.0;  // s

// Defaults for existence probability and evaluation time.
double kExistenceProbability = 0.5;
int kEdgeEvaluationTime = 1000000; // us

vector<PlannerStats> plan2d(PlannerType planner_type, cv::Mat costs,
                            cv::Mat probabilities, cv::Mat edge_groups, unsigned char obsthresh,
                            int start_x, int start_y, int goal_x, int goal_y) {
  vector<PlannerStats> ee_stats;
  int bRet = 0;
  double allocated_time_secs = 100.0; // in seconds
  double initialEpsilon = 3.0;
  MDPConfig MDPCfg;
  bool bsearchuntilfirstsolution = false;

  const int width = costs.cols;
  const int height = costs.rows;

  // Initialize Environment (should be called before initializing anything else)
  EnvironmentNAV2DProb environment_nav2D;

  if (!environment_nav2D.InitializeEnv(width, height, costs.ptr<uchar>(0),
                                       probabilities.ptr<double>(0), edge_groups.ptr<uchar>(0), start_x, start_y,
                                       goal_x, goal_y, obsthresh)) {
    printf("ERROR: InitializeEnv failed\n");
  }
  environment_nav2D.SetStochasticEdgeEvaluationTime(kEdgeEvaluationTime);

  // Initialize MDP Info
  if (!environment_nav2D.InitializeMDPCfg(&MDPCfg)) {
    printf("ERROR: InitializeMDPCfg failed\n");
  }

  if (planner_type == PLANNER_TYPE_ESP) {
    unique_ptr<ESPPlanner> planner(new ESPPlanner(&environment_nav2D, 1, true));

    if (planner->set_start(MDPCfg.startstateid) == 0) {
      printf("ERROR: failed to set start state\n");
      return ee_stats;
    }

    if (planner->set_goal(MDPCfg.goalstateid) == 0) {
      printf("ERROR: failed to set goal state\n");
      return ee_stats;
    }

    MHAReplanParams params(kTimeLimit);
    params.initial_eps = 1;
    params.final_eps = 1;
    params.return_first_solution = false;
    params.mha_type = mha_planner::MHAType::FOCAL;

    EdgeEvaluationParams edge_eval_params;
    edge_eval_params.strategy = EvaluationStrategy::AFTER_N_EXPANDS;
    // edge_eval_params.strategy = EvaluationStrategy::AFTER_N_PATHS;
    edge_eval_params.after_n_expands = 10000;
    edge_eval_params.after_n_paths = 1;
    // edge_eval_params.selector = EdgeOrderStrategy::SSP;
    // edge_eval_params.selector = EdgeOrderStrategy::MOST_LIKELY;
    planner->SetEdgeEvaluationParams(edge_eval_params);

    // vector<int> solution_ids;
    vector<sbpl::Path> solution_paths;
    int solution_cost = 0;
    bool success = planner->replan(&solution_paths, params, &solution_cost);

    if (success) {
      printf("Found solution with size %zu and cost %d\n",
             solution_paths[0].state_ids.size(), solution_cost);
    } else {
      printf("Lazy ESP failed to get solution\n");
    }

    // printf("Found %d possible paths:\n", static_cast<int>(solution_paths.size()));
    // for (size_t ii = 0; ii < solution_paths.size(); ++ii) {
    //   const auto &solution_path = solution_paths[ii];
    //   cout << "Cost: " << solution_path.cost << "     ";
    //
    //   for (size_t jj = 0; jj < solution_path.state_ids.size(); ++jj) {
    //     cout << solution_path.state_ids[jj] << " ";
    //   }
    //
    //   cout << endl;
    // }
    // cout << endl;

    planner->get_ee_stats(&ee_stats);
  } else if (planner_type == PLANNER_TYPE_LAZY_ARA) {

    unique_ptr<LazyARAPlanner> planner(new LazyARAPlanner(&environment_nav2D,
                                                          true));

    if (planner->set_start(MDPCfg.startstateid) == 0) {
      printf("ERROR: failed to set start state\n");
      return ee_stats;
    }

    if (planner->set_goal(MDPCfg.goalstateid) == 0) {
      printf("ERROR: failed to set goal state\n");
      return ee_stats;
    }

    ReplanParams params(kTimeLimit);
    params.initial_eps = 1;
    params.final_eps = 1;
    params.dec_eps = 1;
    params.return_first_solution = false;
    // vector<int> solution_ids;
    vector<int> solution;
    int solution_cost = 0;
    bool success = planner->replan(&solution, params, &solution_cost);

    if (success) {
      printf("Found solution with size %zu and cost %d\n",
                solution.size(), solution_cost);
    } else {
      printf("Lazy ARA failed to get solution\n");
    }

    planner->get_search_stats(&ee_stats);
  }

  return ee_stats;
}

void GenerateGroundTruth(const cv::Mat &full_graph,
                         const cv::Mat &existence_probabilities, const cv::Mat &edge_groups,
                         cv::Mat &graph_instance) {
  graph_instance = full_graph.clone();
  double min, max;
  cv::minMaxIdx(edge_groups, &min, &max);
  int num_groups = max - 1;
  for (int ii = 1; ii <= num_groups; ++ii) {
    int rand_val = rand() % 100;
    bool should_block = rand_val > (kExistenceProbability * 100);

    if (should_block) {
      graph_instance.setTo(1.0, edge_groups == ii);
    }
  }
}

void GetRandomValidStartGoalPair(const cv::Mat &graph_instance,
                                 int *start_x, int *start_y, int *goal_x, int *goal_y) {
  int width = graph_instance.cols;
  int height = graph_instance.rows;
  const int kMaxIter = 10000;

  *start_x = rand() % (width);
  *start_y = rand() % (height);
  int iter = 0;

  while (graph_instance.at<unsigned char>(cv::Point(*start_x, *start_y)) != 0 ||
         iter < kMaxIter) {
    *start_x = rand() % (width);
    *start_y = rand() % (height);
    iter++;
  }

  *goal_x = rand() % (width);
  *goal_y = rand() % (height);
  iter = 0;

  while (graph_instance.at<unsigned char>(cv::Point(*goal_x, *goal_y)) != 0 ||
         iter < kMaxIter) {
    *goal_x = rand() % (width);
    *goal_y = rand() % (height);
    iter++;
  }
}

void AddStochasticEdges(cv::Mat probabilities, cv::Mat edge_groups, int x,
                        int y, double prob = kExistenceProbability) {
  const int radius = 8;
  cv::circle(probabilities, cv::Point(x, y), radius, prob, -1);
  cv::circle(edge_groups, cv::Point(x, y), radius, group_id, -1);
  group_id++;
}

int main(int argc, char *argv[]) {
  if (argc < 4) {
    printf("Usage: ./2dnav_experiments image_file existence_probability collision_check_time\n");
    return -1;
  }

  kExistenceProbability = stod(string(argv[2]));
  kEdgeEvaluationTime = stoi(string(argv[3]));

  cout << "Existence prob: " << kExistenceProbability << endl;
  cout << "Eval time: " << kEdgeEvaluationTime << endl;

  // Fixed seed for repeatability.
  srand (1);
  // srand (time(NULL));

  cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
  printf("Loaded map with %d rows and %d cols\n", image.rows, image.cols);

  auto planner = PLANNER_TYPE_ESP;
  cv::Mat costs, probabilities, edge_groups;
  costs = image > 245;
  costs = 1 - costs;
  unsigned char obsthresh = 1;
  int width = costs.cols;
  int height = costs.rows;
  probabilities.create(costs.rows, costs.cols, CV_64FC1);
  probabilities.setTo(0.0);
  edge_groups.create(costs.rows, costs.cols, CV_8UC1);
  edge_groups.setTo(0.0);

  probabilities.setTo(1.0);
  edge_groups.setTo(0);
  AddStochasticEdges(probabilities, edge_groups, 115, 76);
  // AddStochasticEdges(probabilities, edge_groups, 128, 77);
  AddStochasticEdges(probabilities, edge_groups, 215, 180);
  AddStochasticEdges(probabilities, edge_groups, 134, 292);
  AddStochasticEdges(probabilities, edge_groups, 204, 291);
  AddStochasticEdges(probabilities, edge_groups, 439, 170);


  AddStochasticEdges(probabilities, edge_groups, 222, 104);
  AddStochasticEdges(probabilities, edge_groups, 351, 152);
  AddStochasticEdges(probabilities, edge_groups, 360, 291);
  AddStochasticEdges(probabilities, edge_groups, 24, 388);
  AddStochasticEdges(probabilities, edge_groups, 441, 392);
  AddStochasticEdges(probabilities, edge_groups, 160, 344);
  AddStochasticEdges(probabilities, edge_groups, 390, 352);
  AddStochasticEdges(probabilities, edge_groups, 503, 370);
  AddStochasticEdges(probabilities, edge_groups, 500, 285);
  AddStochasticEdges(probabilities, edge_groups, 347, 175);


  probabilities.setTo(0.0, costs == 1.0);
  edge_groups.setTo(group_id, edge_groups == 0);
  edge_groups.setTo(0, costs == 1.0);

  cv::Mat edge_groups_color;
  cv::normalize(edge_groups, edge_groups_color, 0, 255, cv::NORM_MINMAX,
                CV_8UC1);
  cv::applyColorMap(edge_groups_color, edge_groups_color, cv::COLORMAP_JET);
  edge_groups_color.setTo(cv::Vec3b(0, 0, 0), edge_groups == 0);
  edge_groups_color.setTo(cv::Vec3b(255, 255, 255), edge_groups == group_id);
  cv::imshow("Edge groups", edge_groups_color);
  cv::imwrite("/usr0/home/venkatrn/indigo_workspace/src/esp_planner/edge_groups.png", edge_groups_color);
  // cv::waitKey(0);
  // return 1;

  boost::filesystem::path output_file =
    "/usr0/home/venkatrn/indigo_workspace/src/esp_planner/stats";
  ofstream fs_esp, fs_lazy;
  string filename = output_file.string() + "_esp_" + argv[2] + "_" +
    argv[3];
  fs_esp.open ((filename + "_esp.txt").c_str());
  fs_lazy.open ((filename + "_lazy.txt").c_str());

  if (!fs_esp.is_open () || fs_esp.fail ()) {
    printf("Can't open stats file %s\n", output_file.string().c_str());
    return (false);
  }

  if (!fs_lazy.is_open () || fs_lazy.fail ()) {
    printf("Can't open stats file %s\n", output_file.string().c_str());
    return (false);
  }

  const int num_trials = 1;
  const int num_start_goal_pairs = 5;

  std::unique_ptr<ofstream> fs;

  for (int ii = 0; ii < num_trials; ++ii) {
    cv::Mat instance;
    GenerateGroundTruth(costs, probabilities, edge_groups, instance);

    int start_goal_pair = 0;

    // Will ensure num_start_goal_pairs experiments are successful (i.e, path
    // exists for this instance using the lazy A* planner).
    while (start_goal_pair < num_start_goal_pairs) {
      int start_x, start_y, goal_x, goal_y;
      GetRandomValidStartGoalPair(instance, &start_x, &start_y, &goal_x, &goal_y);

      // cv::imshow("Instance", 255 * (instance));
      // cv::waitKey(0);
      for (int planner_type = 0; planner_type < NUM_PLANNER_TYPES; planner_type++) {

        auto ee_stats = plan2d(static_cast<PlannerType>(planner_type), instance,
                               probabilities, edge_groups,
                               obsthresh, start_x, start_y, goal_x, goal_y);

        // if (planner_type == 0) {
        //   continue;
        // } else {
        //  start_goal_pair++;
        // }

        // Skip this trial if no solution exists.
        if (planner_type == 0) {
          if (ee_stats.empty() || ee_stats.back().cost == INFINITECOST) {
            break;
          } else {
            start_goal_pair++;
          }
        }

        // auto ee_stats = plan2d(static_cast<PlannerType>(planner_type), instance,
        //                        probabilities, edge_groups,
        //                        obsthresh, 100, 90, 305, 415);
        // auto ee_stats = plan2d(planner, instance, probabilities, edge_groups,
        //                        obsthresh, 100, 90, 113, 218);
        printf("Anytime stats:\n\n");

        // Ugh. Do the correct thing.
        switch (planner_type) {
        case PLANNER_TYPE_ESP:
          for (const auto &ee_stat : ee_stats) {
            printf("%f %d\n", ee_stat.time, ee_stat.cost);
            fs_esp << std::setprecision(5) << ee_stat.time << " ";
          }

          fs_esp << endl;

          for (const auto &ee_stat : ee_stats) {
            fs_esp << std::setprecision(5) << ee_stat.cost << " ";
          }

          fs_esp << endl;
          break;

        case PLANNER_TYPE_LAZY_ARA:
          for (const auto &ee_stat : ee_stats) {
            printf("%f %d\n", ee_stat.time, ee_stat.cost);
            fs_lazy << std::setprecision(5) << ee_stat.time << " ";
          }

          fs_lazy << endl;

          for (const auto &ee_stat : ee_stats) {
            fs_lazy << std::setprecision(5) << ee_stat.cost << " ";
          }

          fs_lazy << endl;
          break;
        }
      }
    }
  }

  fs_esp.close();
  fs_lazy.close();
}

// Unused gunk.

// cv::Mat dist_transform, probabilities_float;
// cv::distanceTransform(1 - costs, dist_transform, CV_DIST_L1, 3);
//
// probabilities.setTo(0.0, costs == 1);
// probabilities.setTo(0.5, dist_transform < 4);
// probabilities.setTo(1.0, dist_transform >= 4);
//
// cv::Mat prob_color;
// prob_color = 255 * probabilities;
// cv::normalize(prob_color, prob_color, 0, 255, cv::NORM_MINMAX, CV_8UC1);
// cv::applyColorMap(prob_color, prob_color, cv::COLORMAP_JET);
// cv::imshow("Probabilities", prob_color);
// cv::waitKey(0);

// cv::Mat edge_groups, edge_groups2, edge_groups3, edge_groups4, edge_groups_color;
// int element_len = 8;
// cv::Mat horz_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(element_len, 1));
// cv::Mat vert_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, element_len));
// cv::dilate(costs, edge_groups, horz_element, cv::Point(0, 0));
// cv::dilate(costs, edge_groups2, horz_element, cv::Point(element_len - 1,0));
// cv::dilate(costs, edge_groups3, vert_element, cv::Point(0, 0));
// cv::dilate(costs, edge_groups4, vert_element, cv::Point(0, element_len - 1));
// edge_groups = edge_groups & edge_groups2;
// edge_groups3 = edge_groups3 & edge_groups4;
// edge_groups = edge_groups | edge_groups3;
// cv::normalize(edge_groups - costs, edge_groups_color, 0, 255, cv::NORM_MINMAX, CV_8UC1);
// cv::applyColorMap(edge_groups_color, edge_groups_color, cv::COLORMAP_JET);
// cv::imshow("Edge groups", 255*(edge_groups - costs));
// // cv::imshow("Edge groups", 255 - 255*(edge_groups + costs));
// cv::waitKey(0);
// // return 1;
// probabilities.setTo(1.0);
// probabilities.setTo(0.5, (edge_groups - costs) == 1);

