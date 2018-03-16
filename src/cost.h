#ifndef COST_H
#define COST_H
#include "vehicle.h"

using namespace std;

double inefficiency_cost(const Vehicle & vehicle, int goal_lane, const map<int, vector<Vehicle>> & predictions);

double lane_speed(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, int lane);

double proximity_cost(const Vehicle& vehicle, const map<int, vector<Vehicle>>& predictions);

double change_lane_cost(const Vehicle& vehicle, int goal_lane);

double collision_cost(vector<vector<double>>& trajectory, map<int, vector<Vehicle>>& predictions, double coll_rad);

#endif
