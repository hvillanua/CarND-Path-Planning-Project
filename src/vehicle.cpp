#include "vehicle.h"

#include <algorithm>
#include <iostream>
#include <cmath>
#include <map>
#include <string>
#include <iterator>

#include "cost.h"
#include "spline.h"
#include "main.h"

/**
 * Initializes Vehicle
 */

Vehicle::Vehicle(){
  state = "KL";
  lane = -1;
  x = 0;
  y = 0;
  yaw = 0;
  s = 0;
  d = 6;
  v = 0;
  ref_vel = 0;
}

Vehicle::Vehicle(int lane, double x, double y, double yaw, double s, double d, double v, double ref_vel) {
  state = "KL";
  this->lane = lane;
  this->x = x;
  this->y = y;
  this->yaw = yaw;
  this->s = s;
  this->d = d;
  this->v = v;
  this->ref_vel = ref_vel;
}

Vehicle::~Vehicle() {}

vector<vector<double>> Vehicle::choose_next_state(unsigned int steps, map<int, vector<Vehicle>> predictions, vector<double> previous_path_x, vector<double> previous_path_y) {

  vector<string> possible_successor_states = successor_states();
  //vector<string> possible_successor_states = {"LCR"};

  vector<float> costs;
  vector<double> state_ref_vel;
  float cost_for_state;
  vector<vector<vector<double>>> final_trajectories;

  for (unsigned int i=0; i<possible_successor_states.size(); ++i){
    string state = possible_successor_states[i];
    Vehicle aux_vehicle(lane, x, y, yaw, s, d, v, ref_vel);
    aux_vehicle.apply_new_state(state, ref_vel);
    auto trajectory_for_state = aux_vehicle.general_trajectory(steps, predictions, previous_path_x, previous_path_y);

    // calculate the "cost" associated with that trajectory.
    // don't check for collision, since we start in the middle of the road with 0 velocity
    // every car behind us has a projection that collisions with our car at the very start!
    //bool collision = check_collision(trajectory_for_state, predictions, 0);
    //if (!collision){
      //cost_for_state = calculate_cost(*this, predictions, trajectory_for_state);
    cost_for_state = 0;
    cost_for_state += INEFF_W*inefficiency_cost(*this, aux_vehicle.lane, predictions);
    cost_for_state += PROX_W*proximity_cost(aux_vehicle, predictions);
    cost_for_state += COLL_W*collision_cost(trajectory_for_state, predictions, 5);
    cost_for_state += LANE_W*change_lane_cost(*this, aux_vehicle.lane);
    costs.push_back(cost_for_state);
    final_trajectories.push_back(trajectory_for_state);
    state_ref_vel.push_back(aux_vehicle.ref_vel);
    //}
  }

  vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
  int best_idx = distance(begin(costs), best_cost);
  string best_state = possible_successor_states[best_idx];
  double best_ref_vel = state_ref_vel[best_idx];
  apply_new_state(best_state, best_ref_vel);
  return final_trajectories[best_idx];
}

void Vehicle::apply_new_state(string state, double ref_vel){
  this->lane += lane_direction[state];
  this->state = state;
  this->ref_vel = ref_vel;
}

vector<string> Vehicle::successor_states() {

  vector<string> states;
  states.push_back("KL");
  string state = this->state;
  if(state.compare("KL") == 0) {
    if (lane > 0){
      states.push_back("LCL");
    }
    if ( lane < lanes_available - 1){
      states.push_back("LCR");
    }
  }
  return states;
}

double Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int lane){
  /*
  Gets next timestep velocity for a given lane.
  Tries to choose the maximum velocity and acceleration,
  given other vehicle positions and accel/velocity constraints.
  */
  double max_velocity_accel_limit = this->max_acceleration + ref_vel;
  double new_ref_vel;
  Vehicle vehicle_ahead;
  Vehicle vehicle_behind;
  double front_max_dist = 40;
  double back_max_dist = 40;

  if (get_vehicle_ahead(predictions, lane, front_max_dist, vehicle_ahead)) {
    if (get_vehicle_behind(predictions, lane, back_max_dist, vehicle_behind)) {
      //must travel at the speed of traffic, regardless of preferred buffer
      double max_velocity_in_front = (vehicle_ahead.s - this->s) + vehicle_ahead.v;
      new_ref_vel = min(max_velocity_in_front, this->target_speed);
    }
    else {
      double max_velocity_in_front = (vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.v;
      new_ref_vel = min(min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed);
    }
  }
  else {
    new_ref_vel = min(max_velocity_accel_limit, this->target_speed);
  }

  if (new_ref_vel > ref_vel){
    ref_vel += max_acceleration;
  }
  else{
    ref_vel -= max_acceleration;
  }
  return ref_vel;
}

vector<vector<double>> Vehicle::general_trajectory(unsigned int steps, map<int, vector<Vehicle>>& predictions, vector<double> previous_path_x, vector<double> previous_path_y){

  vector<vector<double>> trajectory;
  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x = this->x;
  double ref_y = this->y;
  double ref_yaw = deg2rad(this->yaw);
  int prev_size = previous_path_x.size();

  // Enough points from previous path?
  if ( prev_size < 2 ) {
      // Not enough, we calculate previous point
      double prev_car_x = x - cos(ref_yaw);
      double prev_car_y = y - sin(ref_yaw);

      ptsx.push_back(prev_car_x);
      ptsx.push_back(this->x);

      ptsy.push_back(prev_car_y);
      ptsy.push_back(this->y);
  } else {
      // Use the last two points
      ref_x = previous_path_x[prev_size - 1];
      ref_y = previous_path_y[prev_size - 1];

      double ref_x_prev = previous_path_x[prev_size - 2];
      double ref_y_prev = previous_path_y[prev_size - 2];
      ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

      ptsx.push_back(ref_x_prev);
      ptsx.push_back(ref_x);

      ptsy.push_back(ref_y_prev);
      ptsy.push_back(ref_y);
  }

  // In Frenet add 3 evenly 30m spaced points ahead of starting reference
  vector<double> next_wp0 = getXY(s + 60, 2 + 4*this->lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(s + 90, 2 + 4*this->lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(s + 120, 2 + 4*this->lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  // Transform to local car coordinates
  for ( int i = 0; i < ptsx.size(); i++ ) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
    ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
  }

  // Create the spline
  tk::spline s;
  s.set_points(ptsx, ptsy);

  // Start with all of the points from previous path
  vector<double> next_x_vals;
  vector<double> next_y_vals;
  for ( int i = 0; i < prev_size; i++ ) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  // Calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);

  double x_add_on = 0;
  double vel = this->v;
  for( int i = 1; i < steps - prev_size; i++ ) {

    ref_vel = get_kinematics(predictions, lane);

    // frequency = 0.02, mph to mps = 2.24
    double N = target_dist/(0.02*ref_vel/2.24);
    double x_point = x_add_on + target_x/N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // rotate and translate back to global coordinates
    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }

  trajectory.push_back(next_x_vals);
  trajectory.push_back(next_y_vals);
  return trajectory;
}

map<int, vector<Vehicle>> Vehicle::sensor_close_by_cars(vector<vector<double>> sensor_fusion, unsigned int pred_steps){

  map<int, vector<Vehicle>> predictions;
  int car_id;
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_speed_x;
  double car_speed_y;
  double car_speed;
  int lane;

  for (unsigned int i=0; i<sensor_fusion.size(); ++i){
    car_id = sensor_fusion[i][0];
    car_s = sensor_fusion[i][5];
    // If car is too far away don't use it to predict their motion
    if (abs(this->s - car_s) > 50){
      continue;
    }
    car_d = sensor_fusion[i][6];
    car_x = sensor_fusion[i][1];
    car_y = sensor_fusion[i][2];
    car_speed_x = sensor_fusion[i][3];
    car_speed_y = sensor_fusion[i][4];
    car_speed = sqrt(car_speed_x * car_speed_x + car_speed_y * car_speed_y);
    lane = -1;
    if (car_d > 0 && car_d < 4){
      lane = 0;
    }
    else if (car_d >= 4 && car_d < 8){
      lane = 1;
    }
    else if (car_d >= 8 && car_d < 12){
      lane = 2;
    }
    Vehicle vehicle(lane, car_x, car_y, 0.0, car_s, car_d, car_speed, 0.0);
    auto trajectory = vehicle.non_ego_prediction(pred_steps);
    predictions[car_id] = trajectory;
  }
  return predictions;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>>& predictions, int lane, double max_dist, Vehicle & rVehicle) {
  /*
  Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
  rVehicle is updated if a vehicle is found.
  */

  int max_s = -1;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
      temp_vehicle = it->second[0];
      double dist = abs(temp_vehicle.s - this->s);
      if (temp_vehicle.lane == lane && dist < max_dist && temp_vehicle.s < this->s && temp_vehicle.s > max_s) {
          max_s = temp_vehicle.s;
          rVehicle = temp_vehicle;
          found_vehicle = true;
      }
  }
  return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>>& predictions, int lane, double max_dist, Vehicle & rVehicle) {
  /*
  Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
  rVehicle is updated if a vehicle is found.
  */

  int min_s = 10e5;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
      temp_vehicle = it->second[0];
      double dist = abs(temp_vehicle.s - this->s);
      if (temp_vehicle.lane == lane && dist < max_dist && temp_vehicle.s > this->s && temp_vehicle.s < min_s) {
          min_s = temp_vehicle.s;
          rVehicle = temp_vehicle;
          found_vehicle = true;
      }
  }
  return found_vehicle;
}

vector<Vehicle> Vehicle::non_ego_prediction(unsigned int steps){
  // Assume non-ego vehicles keep their lane and drive at constant speed
  vector<Vehicle> predictions;
  double next_s = this->s;
  for (int i=0; i<steps; ++i){
    vector<double> xy = getXY(next_s, this->d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    Vehicle veh(this->lane, xy[0], xy[1], this->yaw, next_s, this->d, this->v, 0.0);
    predictions.push_back(veh);
    //next_s += this->v*0.02;
    next_s += this->v;
  }
  return predictions;
}

bool Vehicle::check_collision(vector<vector<double>> trajectory, map<int, vector<Vehicle>> predictions, double coll_rad){
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
  return collision;
}
