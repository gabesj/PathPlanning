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
//  vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> &predictions);
  vector<Vehicle> choose_next_state(vector<Vehicle> &predictions);

  vector<string> successor_states();

  vector<Vehicle> generate_trajectory(string state, 
                                      map<int, vector<Vehicle>> &predictions);

  vector<float> get_kinematics(map<int, vector<Vehicle>> &predictions, int lane);

  vector<Vehicle> constant_speed_trajectory();

  vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> &predictions);

  vector<Vehicle> lane_change_trajectory(string state, 
                                         map<int, vector<Vehicle>> &predictions);

  vector<Vehicle> prep_lane_change_trajectory(string state, 
                                              map<int, vector<Vehicle>> &predictions);

  void increment(int dt);

  float position_at(int t);

  bool get_vehicle_behind(map<int, vector<Vehicle>> &predictions, int lane, 
                          Vehicle &rVehicle);

  bool get_vehicle_ahead(map<int, vector<Vehicle>> &predictions, int lane, 
                         Vehicle &rVehicle);

  vector<Vehicle> generate_predictions(Vehicle ego, vector<Vehicle> other_cars, int prev_size, double ahead_horizon, double behind_horizon);

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

  int preferred_buffer = 6; // impacts "keep lane" behavior.

  int lane, goal_lane, goal_s, lanes_available;

  float target_speed, max_acceleration;

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







