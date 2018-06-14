#ifndef BP_H
#define BP_H
#include <string>
#include <unordered_map>
#include "json.hpp"
#define MPH2MPS (0.44704)

using namespace std;
using json = nlohmann::json;

class Behavior_Planner {
public:

  string state_;
  int lane_id_ = 1;
  double goal_v_ = 49.5 * MPH2MPS; // in m/s
  double ref_v_ = 0; // in m/s
  double cooling_ = 0;
  // map data;
  unordered_map<string, vector<double>> map_data_;

  //json real-time data
  double car_x_;
  double car_y_;
  double car_s_;
  double car_d_;
  double car_yaw_;
  double car_speed_; // in m/s
  double end_path_s_;
  double end_path_d_;
  nlohmann::basic_json<> previous_path_x_;
  nlohmann::basic_json<> previous_path_y_;
  nlohmann::basic_json<> sensor_fusion_;

  /**
  * Constructor
  */
  Behavior_Planner();
  Behavior_Planner(unordered_map<string, vector<double>> &map_data, string state);

  /**
  * Destructor
  */
  virtual ~Behavior_Planner();

  /**
  * Functions
  */
  void update_json(json &j);
  vector<vector<double>> plan_and_generate_traj();
  vector<vector<double>> generate_traj_state(string state);
  vector<vector<double>> gen_traj(string state);
};

#endif
