#ifndef COST_H
#define COST_H

#include "vehicle.h"

using std::map;
using std::string;
using std::vector;

float collision_cost(const Vehicle &predicted_self, const vector<Vehicle> &trajectory, const vector<Vehicle> &predictions);

float speed_cost(const Vehicle &predicted_self, const vector<Vehicle> &trajectory, const vector<Vehicle> &predictions);

float lane_change_cost(const Vehicle &predicted_self, const vector<Vehicle> &trajectory, const vector<Vehicle> &predictions);

float calculate_cost(const Vehicle &predicted_self, 
                     const vector<Vehicle> &predictions, 
                     const vector<Vehicle> &trajectory);

#endif  // COST_H
