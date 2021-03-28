#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>

using std::map;
using std::string;
using std::vector;

class Vehicle {
 public:
  // Constructors
  Vehicle();
  Vehicle(int lane, double d, double s, double v, double a, double x, double y, double vx, double vy, double yaw, string state="CS");

  // Destructor
  virtual ~Vehicle();

  // Vehicle functions

  vector<Vehicle> choose_next_state(vector<Vehicle> &predictions, Vehicle &predicted_self);

  vector<string> successor_states();

  vector<Vehicle> generate_trajectory(string state, vector<Vehicle> &predictions, Vehicle &predicted_self);

  vector<double> get_kinematics(vector<Vehicle> &predictions, int lane, Vehicle &predicted_self);

  vector<Vehicle> keep_lane_trajectory(vector<Vehicle> &predictions, Vehicle &predicted_self);

  vector<Vehicle> lane_change_trajectory(string state, vector<Vehicle> &predictions, Vehicle &predicted_self);

  vector<Vehicle> prep_lane_change_trajectory(string state, vector<Vehicle> &predictions, Vehicle &predicted_self);

  bool get_vehicle_behind(vector<Vehicle> &predictions, int lane, Vehicle &rVehicle, Vehicle &predicted_self);

  bool get_vehicle_ahead(vector<Vehicle> &predictions, int lane, Vehicle &rVehicle, Vehicle &predicted_self);

  vector<Vehicle> generate_predictions(Vehicle ego, vector<Vehicle> other_cars, double timesteps);

  Vehicle predict_self(Vehicle ego, double timesteps, double fut_vel);

  void configure(vector<double> &road_data); 

  map<string, int> lane_direction = {{"PLCL", -1}, {"LCL", -1}, {"LCR", 1}, {"PLCR", 1}};

  int L = 1;

  int preferred_buffer = 25; // used in get_kinematics() as the ideal distance to stay away from other vehicles.

  int lane, goal_lane, goal_s, lanes_available;

  double target_speed, max_acceleration, ahead_horizon, behind_horizon;

  double d, s, v, a, x, y, vx, vy, yaw;

  string state;
};

#endif  // VEHICLE_H

// A Vehicle object can have
// lane, d, s, v, a, x, y, vx, vy, yaw, state

// The Vehicle object for the self-driving car has:
// lane, d, s, v, a, x, y,   ,   , yaw, state

// The Vehicle objects for the other cars have:
// lane, d, s, v,  , x, y, vx, vy,    ,    







