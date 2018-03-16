#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
public:

  map<string, int> lane_direction = {{"KL", 0}, {"LCL", -1}, {"LCR", 1}};

  string state;

  int lane;

  double x;

  double y;

  double yaw;

  double s;

  double d;

  double v;

  // Velocity at last point of the generated trajectory
  double ref_vel;

  // Maximum target speed
  double target_speed = 49.5;

  int lanes_available = 3;

  // Safe distance to car ahead
  int preferred_buffer = 10;

  // Maximum acceleration to prevent jerk
  double max_acceleration = 0.224;

  // Inefficiency cost weight
  double INEFF_W = 1.2;
  // Proximity cost weight
  double PROX_W = 2.7;
  // Collision cost weight
  double COLL_W = 2.0;
  // Lane change cost weight
  double LANE_W = 0.7;

  /**
  * Constructor
  */
  Vehicle();
  Vehicle(int lane, double x, double y, double yaw, double s, double d, double v, double ref_vel);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  vector<vector<double>> choose_next_state(unsigned int steps, map<int, vector<Vehicle>> predictions, vector<double> previous_path_x, vector<double> previous_path_y);

  void apply_new_state(string state, double ref_vel);

  vector<string> successor_states();

  double get_kinematics(map<int, vector<Vehicle>> predictions, int lane);

  vector<vector<double>> general_trajectory(unsigned int steps, map<int, vector<Vehicle>>& predictions, vector<double> previous_path_x, vector<double> previous_path_y);

  map<int, vector<Vehicle>> sensor_close_by_cars(vector<vector<double>> sensor_fusion, unsigned int pred_steps);

  bool get_vehicle_behind(map<int, vector<Vehicle>>& predictions, int lane, double max_dist, Vehicle & rVehicle);

  bool get_vehicle_ahead(map<int, vector<Vehicle>>& predictions, int lane, double max_dist, Vehicle & rVehicle);

  vector<Vehicle> non_ego_prediction(unsigned int steps);

  bool check_collision(vector<vector<double>> trajectory, map<int, vector<Vehicle>> predictions, double coll_rad);

};

#endif
