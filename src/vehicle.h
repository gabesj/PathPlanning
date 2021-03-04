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

  vector<Vehicle> constant_speed_trajectory();

  vector<Vehicle> keep_lane_trajectory(vector<Vehicle> &predictions, Vehicle &predicted_self);

  vector<Vehicle> lane_change_trajectory(string state, vector<Vehicle> &predictions);

  vector<Vehicle> prep_lane_change_trajectory(string state, vector<Vehicle> &predictions, Vehicle &predicted_self);

  void increment(int dt);

  float position_at(int t);

  bool get_vehicle_behind(vector<Vehicle> &predictions, int lane, Vehicle &rVehicle, Vehicle &predicted_self);

  bool get_vehicle_ahead(vector<Vehicle> &predictions, int lane, Vehicle &rVehicle, Vehicle &predicted_self);

  vector<Vehicle> generate_predictions(Vehicle ego, vector<Vehicle> other_cars, double timesteps);

  Vehicle predict_self(Vehicle ego, double timesteps);

  void realize_next_state(vector<Vehicle> &trajectory);

  void configure(vector<double> &road_data); ///(vector<int> &road_data);

  // public Vehicle variables
  struct collider{
    bool collision; // is there a collision?
    int  time; // time collision happens
  };

  map<string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1}, 
                                     {"LCR", -1}, {"PLCR", -1}};

  int L = 1;

  int preferred_buffer = 15; // impacts "keep lane" behavior.

  int lane, goal_lane, goal_s, lanes_available;

  double target_speed, max_acceleration, ahead_horizon, behind_horizon;

  double d, s, v, a, x, y, vx, vy, yaw;

  string state;
};

#endif  // VEHICLE_H

///a Vehicle can have
// lane, d, s, v, a, x, y, vx, vy, yaw, state

///The Vehicle for the self-driving car has:
// lane, d, s, v,  , x, y,   ,   , yaw, state

///The Vehicles for the other cars have:
// lane, d, s, v,  , x, y, vx, vy,    ,    







