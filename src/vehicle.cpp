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

// Initialize Vehicle
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
}

Vehicle::~Vehicle() {}



vector<Vehicle> Vehicle::choose_next_state(vector<Vehicle> &predictions, Vehicle &predicted_self) {
  vector<string> states = successor_states();
  float cost;
  vector<float> costs;
  vector<vector<Vehicle>> final_trajectories;
  for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
    vector<Vehicle> trajectory = generate_trajectory(*it, predictions, predicted_self);
    if (trajectory.size() != 0) {
      cost = calculate_cost(predicted_self, predictions, trajectory);
      costs.push_back(cost);
      final_trajectories.push_back(trajectory);
    }
  }
  vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
  int best_idx = distance(begin(costs), best_cost);

  return final_trajectories[best_idx];
}



vector<string> Vehicle::successor_states() {
  // Provides the possible next states given the current state for the FSM 
  //   discussed in the course, with the exception that lane changes happen 
  //   instantaneously, so LCL and LCR can only transition back to KL.
  int lanes_available = 3;
  vector<string> states;
  states.push_back("KL"); // If state is "LCL" or "LCR", then just return "KL"
  string state = this->state;
  if(state.compare("KL") == 0) {
    if (lane != 0) {
      states.push_back("PLCL");
    }
    if (lane != lanes_available - 1) {
      states.push_back("PLCR");
    }
  } 
  else if (state.compare("PLCL") == 0) {
    if (lane != 0) {
      states.push_back("PLCL");
      states.push_back("LCL");
    }
  } 
  else if (state.compare("PLCR") == 0) {
    if (lane != lanes_available - 1) {
      states.push_back("PLCR");
      states.push_back("LCR");
    }
  }

  return states;
}



vector<Vehicle> Vehicle::generate_trajectory(string state, vector<Vehicle> &predictions, Vehicle &predicted_self) {
  // Given a possible next state, generate the appropriate trajectory to realize
  //   the next state.
  vector<Vehicle> trajectory;
  if (state.compare("KL") == 0) {
    trajectory = keep_lane_trajectory(predictions, predicted_self);
  } 
  else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
    trajectory = lane_change_trajectory(state, predictions, predicted_self);
  } 
  else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
    trajectory = prep_lane_change_trajectory(state, predictions, predicted_self);
  }

  return trajectory;
}



vector<double> Vehicle::get_kinematics(vector<Vehicle> &predictions, int lane, Vehicle &predicted_self) {
  // Gets next timestep kinematics (position, velocity, acceleration) 
  //   for a given lane. Tries to choose the maximum velocity and acceleration, 
  //   given other vehicle positions and accel/velocity constraints.

  double new_position;
  double new_velocity;
  double new_accel;
  double gap_to_close; //approximate gap between self-driving car position and the target position
  int catch_up_time = 1; //number of seconds during which I would like to close the gap
  double accel_to_catch_up; //acceleration needed to close the gap in the desired time
  Vehicle vehicle_ahead;
  Vehicle vehicle_behind;

  if (get_vehicle_ahead(predictions, lane, vehicle_ahead, predicted_self)) { 
    if (predicted_self.state.compare("PLCL") != 0 && (get_vehicle_behind(predictions, lane, vehicle_behind, predicted_self))) {
      // must travel at the speed of traffic, regardless of preferred buffer
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
      /////accel_to_catch_up = 3*gap_to_close / pow(catch_up_time, 3) + predicted_self.a;   

      ///solve for acceleration in equation of form ax^2+bx+c=d, 
      ///where a=new_accel, x=catch_up_time, b=(vehicle speed - target speed), c=0, and d=gap_to_close
      accel_to_catch_up = (gap_to_close - ((this->v) - (vehicle_ahead.v - vehicle_behind.v)/2))/(catch_up_time*catch_up_time);
      new_accel = std::min(accel_to_catch_up, this->max_acceleration);
      new_accel = std::max(accel_to_catch_up, (-1 * this->max_acceleration));  
    } 
    else {
      gap_to_close = (vehicle_ahead.s - predicted_self.s - this->preferred_buffer);

      /// find the acceleration needed to close the gap in the desired amount of time.
      /// the gap_to_close is equal to the integral from 0 to catch_up_time of d+vt+at^2.  
      /// Since gap_to_close is just the distance traveled by the car in addition to its current trajectory, it's not the absolute distance 
      /// so v=0 in the equation, which makes it easier. The acceleration will later just be added to the car's current velocity. 
      /// the integrated equation simplifies to:
      /////accel_to_catch_up = 3*gap_to_close / pow(catch_up_time, 3);   
      ///solve for acceleration in equation of form ax^2+bx+c=d, 
      ///where a=new_accel, x=catch_up_time, b=(vehicle speed - vehicle ahead speed), c=0, and d=gap_to_close
      accel_to_catch_up = (gap_to_close - ((this->v) - vehicle_ahead.v))/(catch_up_time*catch_up_time);  
      new_accel = std::min(accel_to_catch_up, this->max_acceleration);
      new_accel = std::max(accel_to_catch_up, (-1 * this->max_acceleration));
    }
    new_velocity = vehicle_ahead.v;
  } 
  else {
    new_accel = this->max_acceleration;
    new_velocity = this->target_speed;
  }
  if(predicted_self.v > this->target_speed) {
	double accel_to_slow_down = this->target_speed - predicted_self.v;
	new_accel = std::min(new_accel, accel_to_slow_down);
  }  
  new_position = predicted_self.s;
  
  return{new_position, new_velocity, new_accel};
}



vector<Vehicle> Vehicle::keep_lane_trajectory(vector<Vehicle> &predictions, Vehicle &predicted_self) {
  // Generate a keep lane trajectory.
  vector<double> kinematics = get_kinematics(predictions, predicted_self.lane, predicted_self);
  double new_v = kinematics[1];
  double new_a = kinematics[2];
  vector<Vehicle> trajectory = {Vehicle(predicted_self.lane, predicted_self.d, predicted_self.s, new_v, new_a, predicted_self.x, predicted_self.y, predicted_self.vx, predicted_self.vy, predicted_self.yaw, "KL")}; // intended lane
  trajectory.push_back(Vehicle(predicted_self.lane, predicted_self.d, predicted_self.s, new_v, new_a, predicted_self.x, predicted_self.y, predicted_self.vx, predicted_self.vy, predicted_self.yaw, "KL")); // final lane
  
  return trajectory;
}



vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, vector<Vehicle> &predictions, Vehicle &predicted_self) {
  // Generate a trajectory preparing for a lane change.
  
  int intended_lane = this->lane + lane_direction[state];
  vector<double> intended_lane_kinematics = get_kinematics(predictions, intended_lane, predicted_self);
  double intended_lane_new_v = intended_lane_kinematics[1];
  double intended_lane_new_a = intended_lane_kinematics[2]; 
  double intended_lane_new_d = 2 + intended_lane * 4;
  vector<double> final_lane_kinematics = get_kinematics(predictions, this->lane, predicted_self);
  double final_lane_new_v = final_lane_kinematics[1];
  double final_lane_new_a = final_lane_kinematics[2]; 
  
  vector<Vehicle> trajectory = {Vehicle(intended_lane, intended_lane_new_d, predicted_self.s, intended_lane_new_v, intended_lane_new_a, predicted_self.x, predicted_self.y, predicted_self.vx, predicted_self.vy, predicted_self.yaw, state)}; // intended lane
  trajectory.push_back(Vehicle(this->lane, predicted_self.d, predicted_self.s, final_lane_new_v, final_lane_new_a, predicted_self.x, predicted_self.y, predicted_self.vx, predicted_self.vy, predicted_self.yaw, state)); // final lane
 
  return trajectory;
}



vector<Vehicle> Vehicle::lane_change_trajectory(string state, vector<Vehicle> &predictions, Vehicle &predicted_self) {
  // Generate a lane change trajectory.
  int new_lane = this->lane + lane_direction[state];
  vector<Vehicle> trajectory;
  vector<double> kinematics = get_kinematics(predictions, new_lane, predicted_self);
  double new_v = kinematics[1];
  double new_a = kinematics[2]; 
  double new_d = 2 + new_lane * 4;
  trajectory.push_back(Vehicle(new_lane, new_d, predicted_self.s, new_v, new_a, predicted_self.x, predicted_self.y, predicted_self.vx, predicted_self.vy, predicted_self.yaw, state));  // intended lane
  trajectory.push_back(Vehicle(new_lane, new_d, predicted_self.s, new_v, new_a, predicted_self.x, predicted_self.y, predicted_self.vx, predicted_self.vy, predicted_self.yaw, state)); // final lane
  
  return trajectory;
}



bool Vehicle::get_vehicle_behind(vector<Vehicle> &predictions, int lane, Vehicle &rVehicle, Vehicle &predicted_self) {
  // Returns a true if a vehicle is found behind the current vehicle, false 
  //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
  double max_s = predicted_self.s - this->behind_horizon - 1;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  for (int i=0; i<predictions.size(); ++i) {
    temp_vehicle = predictions[i]; 
    if (temp_vehicle.lane == lane && temp_vehicle.s < predicted_self.s && temp_vehicle.s > max_s) {
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
  for (int i=0; i<predictions.size(); ++i) {
    temp_vehicle = predictions[i];
    if (temp_vehicle.lane == lane && temp_vehicle.s >= predicted_self.s  && temp_vehicle.s < min_s) {
      min_s = temp_vehicle.s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }
  
  return found_vehicle;
}



Vehicle predict_self(Vehicle ego, double timesteps) {
  // Generates predictions for ego vehicle to be used in trajectory generation
  Vehicle predicted_self = ego;
  double step_time = 0.02 * timesteps;
  predicted_self.s += predicted_self.v*step_time + predicted_self.a*step_time*step_time;
  predicted_self.v += predicted_self.a*step_time;

  return predicted_self;
}



vector<Vehicle> generate_predictions(Vehicle ego, vector<Vehicle> other_cars, double timesteps) {
  //generates predictions for other vehicles between the behind_horizon and ahead_horizon
  vector<Vehicle> predictions;
  for(int i=0; i<other_cars.size(); ++i) {
    Vehicle predict_car = other_cars[i];
    double step_time = 0.02 * timesteps;
    predict_car.s += predict_car.v*step_time + predict_car.a*step_time*step_time; 
    predictions.push_back(predict_car);
  }
  
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



void Vehicle::configure(vector<double> &road_data) {
  // Called by simulator before simulation begins. Sets various parameters which
  //   will impact the ego vehicle.
  target_speed = road_data[0];
  max_acceleration = road_data[1];
  ahead_horizon = road_data[2];
  behind_horizon = road_data[3];
}
