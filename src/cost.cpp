#include "cost.h"

#include <functional>
#include <iterator>
#include <map>
#include <math.h>
#include "vehicle.h"
#include "main.h"


double inefficiency_cost(const Vehicle & vehicle, int goal_lane, const map<int, vector<Vehicle>> & predictions) {

    double proposed_speed_current = lane_speed(vehicle, predictions, vehicle.lane);
    // If no vehicle is in the proposed lane, we can travel at target speed.
    if (proposed_speed_current < 0) {
        proposed_speed_current = vehicle.target_speed;
    }

    double proposed_speed_final = lane_speed(vehicle, predictions, goal_lane);
    if (proposed_speed_final < 0) {
        proposed_speed_final = vehicle.target_speed;
    }

    double cost = (2.0*vehicle.target_speed - proposed_speed_current - proposed_speed_final)/vehicle.target_speed;
    // If car is on one of the side lanes, subtract speed of furthest away lane to encourage lane changing
    /*
    if (goal_lane == 1 && vehicle.lane == 2){
      double other_speed = lane_speed(vehicle, predictions, 0);
      if (other_speed < 0){
        other_speed = vehicle.target_speed;
      }
      cost -= (vehicle.target_speed - other_speed)/vehicle.target_speed;
    }
    if (goal_lane == 1 && vehicle.lane == 0){
      double other_speed = lane_speed(vehicle, predictions, 2);
      if (other_speed < 0){
        other_speed = vehicle.target_speed;
      }
      cost -= (vehicle.target_speed - other_speed)/vehicle.target_speed;
    }
    */

    return cost;
}

double lane_speed(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, int lane) {
  /*
    Average the speed of all the cars within 50m of our car in the proposed lane
  */
  double velocity = 0.0;
  unsigned int num_veh = 0;
  for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it) {
      int key = it->first;
      Vehicle other_vehicle = it->second[0];
      if (vehicle.lane == lane && key != -1 && (other_vehicle.s > vehicle.s && other_vehicle.s < vehicle.s + 80)) {
        ++num_veh;
        velocity += vehicle.v;
      }
  }
  if (num_veh>0){
    return velocity/num_veh;
  }
  else{
    //Found no vehicle in the lane
    return -1.0;
  }
}

double proximity_cost(const Vehicle& vehicle, const map<int, vector<Vehicle>>& predictions){
  double cost = 0;
  unsigned int in_range = 0;
  int pretended_lane_d = 2+4*vehicle.lane;
  for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it){
    auto aux_vehi = it->second[0];
    if (vehicle.lane == aux_vehi.lane && abs(vehicle.s - aux_vehi.s) < 100 && aux_vehi.s > vehicle.s-20){
      cost += abs(vehicle.s - aux_vehi.s)/50;
      cost += abs(pretended_lane_d - aux_vehi.d)/12;
      ++in_range;
    }
  }
  cost = (cost > 0 ? exp(-cost/2) : 0);
  return cost;
}

double change_lane_cost(const Vehicle& vehicle, int goal_lane){
  return abs(vehicle.lane - goal_lane);
}

double collision_cost(vector<vector<double>>& trajectory, map<int, vector<Vehicle>>& predictions, double coll_rad){
  bool collision = false;
  double path1_x;
  double path1_y;
  double path2_x;
  double path2_y;
  unsigned int path_size = trajectory[0].size();

  map<int, vector<Vehicle>>::iterator it = predictions.begin();
  while (!collision && it != predictions.end()){
    unsigned int i = 0;
    vector<Vehicle> pred = it->second;
    if (distance(trajectory[0][0], trajectory[1][0], pred[0].x, pred[0].y) > 100){
      ++it;
      continue;
    }
    while (!collision && i<path_size){
      double path1_x = trajectory[0][i];
      double path1_y = trajectory[1][i];
      double path2_x = pred[i].x;
      double path2_y = pred[i].y;
      collision = distance(path1_x, path1_y, path2_x, path2_y) < coll_rad;
      ++i;
    }
    ++it;
  }
  return double(collision);
}

