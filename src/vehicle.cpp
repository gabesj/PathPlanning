#include "vehicle.h"
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "cost.h"
#include <math.h>/////////

using std::string;
using std::vector;

// Initializes Vehicle
Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, double d, double s, double v, double a, double x, double y, double vx, double vy, double yaw, string state) {
  this->lane = lane;
  this->d = d;
  this->s = s;
  this->v = v;
  this->a = a;
  this->x = x;
  this->y = y;
  this->vx = vx;
  this->vy = vy;
  this->yaw = yaw;
  this->state = state;
//  max_acceleration = -1;
}

Vehicle::~Vehicle() {}

//vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> &predictions) {
vector<Vehicle> Vehicle::choose_next_state(vector<Vehicle> &predictions, Vehicle &predicted_self) {
  /**
   * Here you can implement the transition_function code from the Behavior 
   *   Planning Pseudocode classroom concept.
   *
   * @param A predictions map. This is a map of vehicle id keys with predicted
   *   vehicle trajectories as values. Trajectories are a vector of Vehicle 
   *   objects representing the vehicle at the current timestep and one timestep
   *   in the future.
   * @output The best (lowest cost) trajectory corresponding to the next ego 
   *   vehicle state.
   *
   * Functions that will be useful:
   * 1. successor_states - Uses the current state to return a vector of possible
   *    successor states for the finite state machine.
   * 2. generate_trajectory - Returns a vector of Vehicle objects representing 
   *    a vehicle trajectory, given a state and predictions. Note that 
   *    trajectory vectors might have size 0 if no possible trajectory exists 
   *    for the state. 
   * 3. calculate_cost - Included from cost.cpp, computes the cost for a trajectory.
   *
   * TODO: Your solution here.
   */

  vector<string> states = successor_states();
  //std::cout << "states.size() = " << states.size() << std::endl;
  //std::cout << "states[0] = " << states[0] << std::endl;
  //for(int i=0; i<states.size(); ++i){
  //  std::cout << "successor_states = " << states[i] << std::endl;
  //}
  float cost;
  vector<float> costs;
  vector<vector<Vehicle>> final_trajectories;
/*
  for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
    vector<Vehicle> trajectory = generate_trajectory(*it, predictions, predicted_self);
    if (trajectory.size() != 0) {
      cost = calculate_cost(*this, predictions, trajectory);
      costs.push_back(cost);
      final_trajectories.push_back(trajectory);
    }
  }

  vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
  int best_idx = distance(begin(costs), best_cost);

  

  return final_trajectories[best_idx];
*/  
  vector<Vehicle> trajectory = generate_trajectory("KL", predictions, predicted_self);///////temporary step in debug
  return trajectory;
}

vector<string> Vehicle::successor_states() {
  // Provides the possible next states given the current state for the FSM 
  //   discussed in the course, with the exception that lane changes happen 
  //   instantaneously, so LCL and LCR can only transition back to KL.
  int lanes_available = 3;
  vector<string> states;
  states.push_back("KL");
  string state = this->state;
  if(state.compare("KL") == 0) {
    states.push_back("PLCL");
    states.push_back("PLCR");
  } else if (state.compare("PLCL") == 0) {
    if (lane != 0) {
      states.push_back("PLCL");
      states.push_back("LCL");
    }
  } else if (state.compare("PLCR") == 0) {
    if (lane != lanes_available - 1) {
      states.push_back("PLCR");
      states.push_back("LCR");
    }
  }
    
  // If state is "LCL" or "LCR", then just return "KL"
  return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state, vector<Vehicle> &predictions, Vehicle &predicted_self) {
  // Given a possible next state, generate the appropriate trajectory to realize
  //   the next state.
  vector<Vehicle> trajectory;
//  if (state.compare("CS") == 0) {
//    trajectory = constant_speed_trajectory();
//  } else if (state.compare("KL") == 0) {
    trajectory = keep_lane_trajectory(predictions, predicted_self);
//  } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
//    trajectory = lane_change_trajectory(state, predictions);
//  } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
//    trajectory = prep_lane_change_trajectory(state, predictions);
//  }

  return trajectory;
}

vector<double> Vehicle::get_kinematics(vector<Vehicle> &predictions, int lane, Vehicle &predicted_self) {
  // Gets next timestep kinematics (position, velocity, acceleration) 
  //   for a given lane. Tries to choose the maximum velocity and acceleration, 
  //   given other vehicle positions and accel/velocity constraints.

  ///double max_velocity_accel_limit = this->max_acceleration + this->v;/////////////////////////////////// NEEDS WORK//////////////
  double new_position;
  double new_velocity;
  double new_accel;
  double gap_to_close; //approximate gap between self-driving car position and the target position
  int catch_up_time = 10; //number of seconds during which I would like to close the gap
  double accel_to_catch_up; //acceleration needed to close the gap in the desired time
  Vehicle vehicle_ahead;
  Vehicle vehicle_behind;

  if (get_vehicle_ahead(predictions, lane, vehicle_ahead, predicted_self)) { 
    if (get_vehicle_behind(predictions, lane, vehicle_behind, predicted_self)) {
      // must travel at the speed of traffic, regardless of preferred buffer
      ///new_velocity = vehicle_ahead.v;
      double relative_midpoint = (vehicle_ahead.s - vehicle_behind.s - 2*this->preferred_buffer) / 2;
      if(relative_midpoint<=0) {
        gap_to_close = ((vehicle_ahead.s - vehicle_behind.s) / 2 + vehicle_behind.s) - predicted_self.s;
      }
      else {
        gap_to_close = (relative_midpoint + vehicle_behind.s + this->preferred_buffer) - predicted_self.s;
      }
      /// find the acceleration needed to close the gap in the desired amount of time.
      /// the gap_to_close is equal to the integral from 0 to catch_up_time of d+vt+at^2.  
      /// Since gap_to_close is just the distance traveled by the car in addition to its current trajectory, it's not the absolute distance 
      /// so v=0 in the equation, which makes it easier. The acceleration will later just be added to the car's current velocity. 
      /// the integrated equation simplifies to:
      accel_to_catch_up = 3*gap_to_close / pow(catch_up_time, 3) + predicted_self.a;     
      ///new_velocity = std::min(std::min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed);
      new_accel = std::min(accel_to_catch_up, this->max_acceleration);
        
      
    } 
    else {
      ///double max_velocity_in_front = (vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.v - 0.5 * (this->a);
      gap_to_close = (vehicle_ahead.s - predicted_self.s - this->preferred_buffer);
      assert(gap_to_close > 0);
      /// find the acceleration needed to close the gap in the desired amount of time.
      /// the gap_to_close is equal to the integral from 0 to catch_up_time of d+vt+at^2.  
      /// Since gap_to_close is just the distance traveled by the car in addition to its current trajectory, it's not the absolute distance 
      /// so v=0 in the equation, which makes it easier. The acceleration will later just be added to the car's current velocity. 
      /// the integrated equation simplifies to:
      accel_to_catch_up = 3*gap_to_close / pow(catch_up_time, 3);     
      ///new_velocity = std::min(std::min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed);
      new_accel = std::min(accel_to_catch_up, this->max_acceleration);
      
    }
  } 
  else {
    ///new_velocity = std::min(max_velocity_accel_limit, this->target_speed);
    new_accel = this->max_acceleration;
  }
  if(predicted_self.v >= this->target_speed) {
	double accel_to_slow_down = this->target_speed - predicted_self.v;
	new_accel = accel_to_slow_down;
  }  
  ///new_accel = new_velocity - this->v; // Equation: (v_1 - v_0)/t = acceleration
  ///new_position = this->s + new_velocity + new_accel / 2.0;
  new_velocity = this->v;
  new_position = this->s;
    
  return{new_position, new_velocity, new_accel};
}

vector<Vehicle> Vehicle::constant_speed_trajectory() {
  // Generate a constant speed trajectory.
  float next_pos = position_at(1);
  vector<Vehicle> trajectory = {Vehicle(this->lane, this->d, this->s,this->v,this->a,this->x, this->y, this->vx, this->vy, this->yaw, this->state), 
                                Vehicle(this->lane, this->d, next_pos,this->v,0, this->x, this->y, this->vx, this->vy, this->yaw, this->state)};
  return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(vector<Vehicle> &predictions, Vehicle &predicted_self) {
  // Generate a keep lane trajectory.
  vector<Vehicle> trajectory = {Vehicle(this->lane, this->d, this->s, this->v, this->a, this->x, this->y, this->vx, this->vy, this->yaw, this->state)};
  vector<double> kinematics = get_kinematics(predictions, this->lane, predicted_self);
  //double new_s = kinematics[0];
  //double new_v = kinematics[1];
  double new_a = kinematics[2];
  //double new_x;///
  //double new_y;///
  //double new_vx;///
  //double new_vy;///
  //double new_yaw;///
  trajectory.push_back(Vehicle(this->lane, this->d, this->s, this->v, new_a, this->x, this->y, this->vx, this->vy, this->yaw, "KL"));
  
  return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, vector<Vehicle> &predictions, Vehicle &predicted_self) {
  // Generate a trajectory preparing for a lane change.
  //float new_s;
  //float new_v;
  //float new_a;
  double new_s;
  double new_v;
  double new_a;
  double new_x;///
  double new_y;///
  double new_vx;///
  double new_vy;///
  double new_yaw;///
  Vehicle vehicle_behind;
  int new_lane = this->lane + lane_direction[state];
  vector<Vehicle> trajectory = {Vehicle(this->lane, this->d, this->s, this->v, this->a, this->x, this->y, this->vx, this->vy, this->yaw, this->state)};
  vector<double> curr_lane_new_kinematics = get_kinematics(predictions, this->lane, predicted_self);

  if (get_vehicle_behind(predictions, this->lane, vehicle_behind, predicted_self)) {
    // Keep speed of current lane so as not to collide with car behind.
    new_s = curr_lane_new_kinematics[0];
    new_v = curr_lane_new_kinematics[1];
    new_a = curr_lane_new_kinematics[2];    
  } else {
    vector<double> best_kinematics;
    vector<double> next_lane_new_kinematics = get_kinematics(predictions, new_lane, predicted_self);
    // Choose kinematics with lowest velocity.
    if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
      best_kinematics = next_lane_new_kinematics;
    } else {
      best_kinematics = curr_lane_new_kinematics;
    }
    //new_s = best_kinematics[0];
    //new_v = best_kinematics[1];
    //new_a = best_kinematics[2];
    new_s = (double)best_kinematics[0];
    new_v = (double)best_kinematics[1];
    new_a = (double)best_kinematics[2];
    
  }

  trajectory.push_back(Vehicle(this->lane, this->d, new_s, new_v, new_a, new_x, new_y, new_vx, new_vy, new_yaw, state));
  
  return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state, vector<Vehicle> &predictions) {
  // Generate a lane change trajectory.
  int new_lane = this->lane + lane_direction[state];
  vector<Vehicle> trajectory;
  Vehicle next_lane_vehicle;
  // Check if a lane change is possible (check if another vehicle occupies 
  //   that spot).
/*  for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); 
       it != predictions.end(); ++it) {
    next_lane_vehicle = it->second[0];
    if (next_lane_vehicle.s == this->s && next_lane_vehicle.lane == new_lane) {
      // If lane change is not possible, return empty trajectory.
      return trajectory;
    }
  }
  trajectory.push_back(Vehicle(this->lane, this->d, this->s, this->v, this->a, this->x, this->y, this->vx, this->vy, this->yaw, this->state));
  vector<float> kinematics = get_kinematics(predictions, new_lane, predicted_self);
  trajectory.push_back(Vehicle(new_lane, this->d, kinematics[0], kinematics[1], 
                               kinematics[2], this->x, this->y, this->vx, this->vy, this->yaw, state));
*/  return trajectory;
}

void Vehicle::increment(int dt = 1) {
  this->s = position_at(dt);
}

float Vehicle::position_at(int t) {
  return this->s + this->v*t + this->a*t*t/2.0;
}

bool Vehicle::get_vehicle_behind(vector<Vehicle> &predictions, int lane, Vehicle &rVehicle, Vehicle &predicted_self) {
  // Returns a true if a vehicle is found behind the current vehicle, false 
  //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
  double max_s = predicted_self.s - this->behind_horizon - 1;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
//  for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
  for (int i=0; i<predictions.size(); ++i) {
    temp_vehicle = predictions[i]; //it->second[0];
    if (temp_vehicle.lane == predicted_self.lane && temp_vehicle.s < predicted_self.s && temp_vehicle.s > max_s) {
      max_s = temp_vehicle.s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }
  
  return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(vector<Vehicle> &predictions, int lane, Vehicle &rVehicle, Vehicle &predicted_self) {
  // Returns a true if a vehicle is found ahead of the current vehicle, false 
  //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
  double min_s = predicted_self.s + this->ahead_horizon + 1;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  //for (vector<Vehicle>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
  for (int i=0; i<predictions.size(); ++i) {
    temp_vehicle = predictions[i];//it->first[0];
    if (temp_vehicle.lane == predicted_self.lane && temp_vehicle.s > predicted_self.s  && temp_vehicle.s < min_s) {
      min_s = temp_vehicle.s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }
  
  return found_vehicle;
}


//Vehicle Vehicle::predict_self(Vehicle ego, int prev_size) {
Vehicle predict_self(Vehicle ego, double timesteps) {

  // Generates predictions for ego vehicle to be used in trajectory generation
  Vehicle predicted_self = ego;
  double step_time = 0.02 * timesteps;
  predicted_self.s += predicted_self.v*step_time + predicted_self.a*step_time*step_time;
  return predicted_self;
}



vector<Vehicle> generate_predictions(Vehicle ego, vector<Vehicle> other_cars, double timesteps) {
  //generates predictions for other vehicles between the behind_horizon and ahead_horizon
  vector<Vehicle> predictions;

  for(int i=0; i<other_cars.size(); ++i) {
    Vehicle predict_car = other_cars[i];
    //std::cout << "ego.s = " << ego.s << "  predict_car.s = " << predict_car.s << std::endl;
    if(((ego.s>predict_car.s)&&((ego.s-predict_car.s)<(ego.behind_horizon))) || ((predict_car.s>=ego.s)&&((predict_car.s-ego.s)<(ego.ahead_horizon)))) {
      double step_time = 0.02 * timesteps;
      predict_car.s += predict_car.v*step_time + predict_car.a*step_time*step_time; /// a is zero
      predictions.push_back(predict_car);
    }
  }
  //std::cout << "predictions.size() = " << predictions.size() << std::endl;
  return predictions;
}




void Vehicle::realize_next_state(vector<Vehicle> &trajectory) {
  // Sets state and kinematics for ego vehicle using the last state of the trajectory.
  Vehicle next_state = trajectory[1];
  this->state = next_state.state;
  this->lane = next_state.lane;
  this->s = next_state.s;
  this->v = next_state.v;
  this->a = next_state.a;
}

void Vehicle::configure(vector<double> &road_data) {//////(vector<int> &road_data) {
  // Called by simulator before simulation begins. Sets various parameters which
  //   will impact the ego vehicle.
  target_speed = road_data[0];
  ///lanes_available = road_data[1];
  ///goal_s = road_data[2];
  ///goal_lane = road_data[3];
  ///max_acceleration = road_data[4];
  max_acceleration = road_data[1];
  ahead_horizon = road_data[2];
  behind_horizon = road_data[3];
}
