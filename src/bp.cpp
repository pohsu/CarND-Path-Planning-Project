#include <algorithm>
#include <iostream>
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include <cfloat>
#include "bp.hpp"
#include "spline.hpp"
#include "helper.hpp"
#include "json.hpp"

using namespace std;
using json = nlohmann::json;

Behavior_Planner::Behavior_Planner(){}

Behavior_Planner::Behavior_Planner(unordered_map<string, vector<double>> &map_data, string state){
  state_ = state;
  map_data_ = map_data;
}

Behavior_Planner::~Behavior_Planner(){}

void Behavior_Planner::update_json(json &j){
  car_x_ = j[1]["x"];
  car_y_ = j[1]["y"];
  car_s_ = j[1]["s"];
  car_d_ = j[1]["d"];
  car_yaw_ = j[1]["yaw"];
  car_speed_ = j[1]["speed"];
  car_speed_ = car_speed_ * MPH2MPS;
  previous_path_x_ = j[1]["previous_path_x"];
  previous_path_y_ = j[1]["previous_path_y"];
  end_path_s_ = j[1]["end_path_s"];
  end_path_d_ = j[1]["end_path_d"];
  sensor_fusion_ = j[1]["sensor_fusion"];
}

vector<vector<double>> Behavior_Planner::plan_and_generate_traj(){
  //compute potential velocities of three lanes

  vector<double> ahead_v(3, goal_v_);
  vector<double> ahead_s(3, 100000);
  vector<double> behind_v(3, -1);
  vector<double> behind_s(3, -1);

  double car_s_end = car_s_;
  double car_d_end = car_d_;
  double car_v_end = car_speed_;

  // obtain targets' statistic for different lanes
  for(int i = 0; i< sensor_fusion_.size();i++){
    double target_s = sensor_fusion_[i][5];
    double target_d = sensor_fusion_[i][6];
    double target_vx = sensor_fusion_[i][3];
    double target_vy = sensor_fusion_[i][4];
    double target_v = distance(target_vx,target_vy,0.0,0.0);
    int id = (int)(target_d/4.0);
    id = min(2,max(0,id));
    if(target_s > car_s_end && target_s < (car_s_end + 50)){
      ahead_v[id] = min(ahead_v[id],target_v);
      ahead_s[id] = min(ahead_s[id],target_s);
    }
    else if (target_s <= car_s_end && target_s > (car_s_end - 30)){
      behind_v[id] = max(behind_v[id],target_v);
      behind_s[id] = max(behind_s[id],target_s);
    }
  }
  vector<double> cost(3, 0);
  for(int i = 0; i <=2; i++){
    cost[i] = pow(goal_v_-ahead_v[i],2) + (cooling_*cooling_+5.0)*pow(lane_id_-i,2) + 5.1*pow(1-i,2);
  }
  cooling_ = max(--cooling_,0.0);
  // we default the prederred land to the current one so if all the lanes
  // are equally efficient, the car does not change lane
  int prefer_lane = 1;
  for(int i = 0; i <=2; i++){
    if (cost[prefer_lane] > cost[i])
      prefer_lane = i;
  }

  // for debug
  cout << "current state: " << state_ << endl;
  print_vector(cost, "cost");
  cout << "prefer_lane: " << prefer_lane << endl;

  //State machine implementaion
  string next_state = state_;
  if (state_ == "KL"){
    if(prefer_lane != lane_id_){
      int id = lane_id_ + (prefer_lane>lane_id_? 1:-1);
      bool cond1 = car_s_end > (behind_s[id] + 10);
      bool cond2 = car_v_end > (behind_v[id]);
      bool cond3 = car_s_end < (ahead_s[id] - 10);
      bool cond4 = car_v_end < ahead_v[id];
      // bool cond5 = car_v_end > 30 * MPH2MPS; // in m/s
      if ( cond1 && cond2 && cond3 && cond4){
        next_state = prefer_lane > lane_id_ ? "CR":"CL";
        lane_id_ += prefer_lane > lane_id_ ? 1:-1;
        cooling_ = 10;
      }
    }
  }
  else if(state_ == "CR" || state_ == "CL"){
    bool cond1 = car_d_end < (lane_id_*4+3.5);
    bool cond2 = car_d_end > (lane_id_*4+0.5);
    if ( cond1 && cond2 ){
      next_state = "KL";
    }
  }
  state_ = next_state;
  return generate_traj_state(next_state);
}

vector<vector<double>> Behavior_Planner::generate_traj_state(string state) {

    if (state == "KL"){
      return gen_traj("KL");
    }
    if (state == "CR" || state_ == "CL"){
      return gen_traj("C");
    }

}

vector<vector<double>>  Behavior_Planner::gen_traj(string state){
  // soft-start and safe distance control
  double dist_s = 100000;
  double dist_safe = 35 * ref_v_/goal_v_;
  double car_s_end = car_s_;
  double saft_v = 100000;
  if (state == "KL" || state == "C"){
    //find the closest vehicle in front of us
    for(int i = 0; i< sensor_fusion_.size();i++){
      double target_d = sensor_fusion_[i][6];
      double target_s = sensor_fusion_[i][5];
      double target_vx = sensor_fusion_[i][3];
      double target_vy = sensor_fusion_[i][4];
      double target_v = distance(target_vx,target_vy,0.0,0.0);
      if( target_s > car_s_end && target_d > (lane_id_*4 -1) && target_d < (lane_id_*4+ 4 +1) ){
        if (dist_s > (target_s-car_s_end)){
          dist_s = target_s-car_s_end;
          saft_v = target_v;
        }
      }
    }
    double MAX_ACC = 0.1;
    if(dist_s < dist_safe ){
      if (ref_v_ > saft_v)
        ref_v_ -= MAX_ACC;
      else
        ref_v_ -= MAX_ACC*0.02*(dist_safe-dist_s)/dist_safe;
    }
    else if (ref_v_ < goal_v_){
      ref_v_ += MAX_ACC;
    }
  }
  /**
  * tran generation
  */
  int prev_size = previous_path_x_.size();
  int num_points = 50;
  vector<double> pts_x, pts_y; // points for generating next path
  double ref_x = car_x_;
  double ref_y = car_y_;
  double ref_yaw = deg2rad(car_yaw_);

  //append first two anchors
  if(prev_size <= 1){
    pts_x.push_back(ref_x - cos(ref_yaw));
    pts_y.push_back(ref_y - sin(ref_yaw));
  }
  else{
    double ref_x_prev = previous_path_x_[prev_size-2];
    double ref_y_prev = previous_path_y_[prev_size-2];
    ref_x = previous_path_x_[prev_size-1];
    ref_y = previous_path_y_[prev_size-1];
    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
    pts_x.push_back(ref_x_prev);
    pts_y.push_back(ref_y_prev);
  }
  pts_x.push_back(ref_x);
  pts_y.push_back(ref_y);

  double lane_d = lane_id_*4+2;
  double end_point = (state == "KL")? 35.0: 50.0;
  vector<double> wp0 = getXY(car_s_+end_point, lane_d, map_data_["map_waypoints_s"], map_data_["map_waypoints_x"], map_data_["map_waypoints_y"]);
  vector<double> wp1 = getXY(car_s_+end_point*2, lane_d, map_data_["map_waypoints_s"], map_data_["map_waypoints_x"], map_data_["map_waypoints_y"]);
  vector<double> wp2 = getXY(car_s_+end_point*3, lane_d, map_data_["map_waypoints_s"], map_data_["map_waypoints_x"], map_data_["map_waypoints_y"]);

  pts_x.push_back(wp0[0]);
  pts_x.push_back(wp1[0]);
  pts_x.push_back(wp2[0]);
  pts_y.push_back(wp0[1]);
  pts_y.push_back(wp1[1]);
  pts_y.push_back(wp2[1]);

  global2car(pts_x, pts_y, ref_x, ref_y, ref_yaw); //passing by reference

  //spline method
  tk::spline s;
  s.set_points(pts_x, pts_y);

  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = distance(target_x,target_y,0,0);
  double x_0 = 0;
  double y_0 = 0;

  vector<double> next_path_x;
  vector<double> next_path_y;
  for(int i = 0; i< prev_size; i++){
    next_path_x.push_back(previous_path_x_[i]);
    next_path_y.push_back(previous_path_y_[i]);
  }
  for(int i = next_path_x.size(); i < num_points; i++){
    double N = (target_dist/(0.02*ref_v_));
    x_0 += target_x/N;
    y_0 = s(x_0);
    double x_point = x_0;
    double y_point = y_0;
    car2global(x_point, y_point, ref_x, ref_y, ref_yaw); //passing by reference
    next_path_x.push_back(x_point);
    next_path_y.push_back(y_point);
  }


  return {next_path_x, next_path_y};
}
