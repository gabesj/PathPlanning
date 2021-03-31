// SUMMARY:
// The goal of this project is to drive a vehicle which must obey the speed limit, not collide with other vehicles, stay in its lane unless changing lanes,
// avoid uncomfortable acceleration and jerk, and change lanes to get around slow-moving traffic.
// The decision of changing lanes is evaluated as a finite state machine with cost functions.
// This program creates a continuously updated list of consecutive points that the car should travel through in 0.02 second increments.
// That list is sent to the simulator and determines the car's behavior such as speed, acceleration, and position.
// The row of green dots in front of the car in the simulator show the listed points of the future trajectory.
// The simulator provides the vehicle's position/speed/angle as well as those of the other vehicles on the right side of the road.


#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"
#include "vehicle.cpp"
#include "cost.cpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // Start in lane 1.  Lane 0 is left lane, lane 2 is right lane.
  int lane = 1;

  // Create the self-driving car
  double init_s = 0.0;
  double init_d = 0.0;
  double init_speed = 0.0;
  double init_accel = 0.0;
  double init_x = 0.0;
  double init_y = 0.0;
  double init_vx = 0.0;
  double init_vy = 0.0;
  double init_yaw = 0.0;
  double speed_limit = 50.0;///mph
  double target_speed = (speed_limit - 1.0) / 2.224; // Dividing by 2.224 converts mph to m/s
  double max_accel = 10-1; // m/s^2         
  double ahead_horizon = 50.0;
  double behind_horizon = 70.0;
  double ref_accel = 0.0;
  vector<double> config_data = {target_speed, max_accel, ahead_horizon, behind_horizon};
  Vehicle ego = Vehicle(lane, init_d, init_s, init_speed, init_accel, init_x, init_y, init_vx, init_vy, init_yaw); 
  ego.configure(config_data);
  ego.state = "KL";
  
  ///other variables
  int first_sweep = true;
  int send_path_size = 30; // shorter path size allows faster reaction to other traffic
  bool lane_changing = false;
  int next_lane = lane;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane,&ego,&first_sweep,&send_path_size,&ref_accel,&lane_changing,&next_lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          // Self-driving car's localization Data
	  ego.x = j[1]["x"];
          ego.y = j[1]["y"];
          ego.s = j[1]["s"];
          ego.d = j[1]["d"];
          ego.yaw = j[1]["yaw"];
          ego.v = j[1]["speed"];
	  ego.v = ego.v/2.224; // Dividing by 2.224 converts mph to m/s
	  ego.a = ref_accel;
	  ego.lane = round((ego.d-2)/4);
	  
          // The program sends a list of future points to traverse, and each cycle the simulator returns a subset of these points
  	  // which are the future points that the car has not yet traversed.
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Size of the last calculated path returned from simulator.  
	  // These are the planned points of the trajectory that the car has not yet passed through.
	  int prev_size = previous_path_x.size();
	 
	  double fut_vel = ego.v; // Estimated future velocity at the end of the planned trajectory.
	  // Estimate the vehicle's velocity at the end of its previously planned trajectory.
	  if(prev_size > 12) {
	    double x1 = previous_path_x[prev_size-1];
	    double y1 = previous_path_y[prev_size-1];
	    double x2 = previous_path_x[prev_size-12];
	    double y2 = previous_path_y[prev_size-12];
	    fut_vel = distance(x1,y1,x2,y2)/(11*0.02);
	  }

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
	  // Create a vector of Vehicles representing the sensor fusion info
	  vector<Vehicle> other_cars;
          for(int i=0; i<sensor_fusion.size(); ++i) {
	    double other_x = sensor_fusion[i][1];
	    double other_y = sensor_fusion[i][2];
	    double other_vx = sensor_fusion[i][3];
            double other_vy = sensor_fusion[i][4];	    
            double other_speed = sqrt(other_vx*other_vx+other_vy*other_vy); 
            double other_s = (double)sensor_fusion[i][5];
	    double other_d = (double)sensor_fusion[i][6];
	    double other_a = 0.0;
	    double other_yaw = 0.0;
	    int other_lane = -1;
            if(other_d>=0.0 && other_d<4.0){
              other_lane = 0;
            } else if (other_d>=4.0 && other_d<8.0){
              other_lane = 1;
            } else {
              other_lane = 2;
            }
            assert(other_lane != -1);
            Vehicle other_car = Vehicle(other_lane, other_d, other_s, other_speed, 
					other_a, other_x, other_y, other_vx, other_vy, other_yaw);
            other_cars.push_back(other_car);
	  }

	  // Get predictions for the future locations of other vehicles and the ego (self-driving) vehicle itself.
	  vector<Vehicle> predictions = generate_predictions(ego, other_cars, prev_size); 	  
	  Vehicle predicted_self = predict_self(ego, prev_size, fut_vel); 

	  // Generate a plan which includes vehicle acceleration and lane designation.
 	  // The plan is returned in a vector of two Vehicle objects.  
	  // The first object provides information for the intended lane of the next finite state, 
	  // the second object is for the final lane of the next finite state.
	  vector<Vehicle> trajectory;
	  if (lane_changing != true) {
            trajectory = ego.choose_next_state(predictions, predicted_self);
          }
	  else { // If a lane change has been initiated, keep executing planned lane change and adapt acceleration to new lane.
            Vehicle desired_predicted_self = predicted_self;
            desired_predicted_self.lane = next_lane;
            trajectory = ego.generate_trajectory("KL", predictions, desired_predicted_self);
          }
	  
	  // Accelerate at the fastest safe and comfortable rate.
	  if(first_sweep) {
	    ref_accel = ego.max_acceleration; // Starting from speed of zero.
	  }
	  else {
	    // Accelerate at a rate that will prevent forward collision in either intended or final lane.
	    ref_accel = std::min(trajectory[0].a, trajectory[1].a); 
	  }

	  // Determine if a lane change has been initiated or completed
          if (ego.lane != trajectory[1].lane) {
            lane_changing = true;
	    next_lane = trajectory[1].lane;
          }
	  else if (ego.lane == next_lane) {
	    lane_changing = false;
	  }
   	  lane = next_lane; 
	  ego.state = trajectory[0].state; 
	   
	  // Create a list of widely spaced (x,y) future waypoints, evenly spaced at 30m.
  	  // Later these waypoints will be interpolated with a spline function and used to determine specific points in the trajectory.
	  vector<double> ptsx;
	  vector<double> ptsy;	 
	  
	  // Reference x, y, yaw states of the vehicle's current location.
	  double ref_x = ego.x;
	  double ref_y = ego.y;
	  double ref_yaw = deg2rad(ego.yaw); 

	  // If size of the remaining untraveled trajectory is almost empty (start of simulator), 
	  // use the car as starting reference and extrapolate the previous point
	  if(prev_size < 2){
	    double prev_car_x = ego.x - cos(ref_yaw);
	    double prev_car_y = ego.y - sin(ref_yaw);
	    
	    ptsx.push_back(prev_car_x);
	    ptsx.push_back(ego.x);
	
	    ptsy.push_back(prev_car_y);
	    ptsy.push_back(ego.y);
	  }
	  
	  // otherwise use the previous path's end point as a starting reference
	  else {
	    ref_x = previous_path_x[prev_size-1];
	    ref_y = previous_path_y[prev_size-1];
	    
	    double ref_x_prev = previous_path_x[prev_size-2];
	    double ref_y_prev = previous_path_y[prev_size-2];
	    ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

	    // use two points that make the path tangent to the previous path's end point
	    ptsx.push_back(ref_x_prev);
	    ptsx.push_back(ref_x);

	    ptsy.push_back(ref_y_prev);
	    ptsy.push_back(ref_y);
	  }
	  // In Frenet coordinates find three future waypoints ahead of the starting reference 
	  // at 30,60, and 90 meters ahead in the desired lane.  
	  vector<double> next_wp0;
	  vector<double> next_wp1;
	  vector<double> next_wp2;
	  
	  if(first_sweep) {
	    next_wp0 = getXY(ego.s+30,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
	    next_wp1 = getXY(ego.s+60,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
	    next_wp2 = getXY(ego.s+90,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
	  }
	  else {
	    next_wp0 = getXY(end_path_s+30,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
	    next_wp1 = getXY(end_path_s+60,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
	    next_wp2 = getXY(end_path_s+90,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
	  }

	  ptsx.push_back(next_wp0[0]);
	  ptsx.push_back(next_wp1[0]);
	  ptsx.push_back(next_wp2[0]);

	  ptsy.push_back(next_wp0[1]);
	  ptsy.push_back(next_wp1[1]);
	  ptsy.push_back(next_wp2[1]);
	  
	  // The ptsx and ptsy vectors now each have 5 points: previous point, current point, point 30m ahead, 60m ahead, and 90m ahead

	  // Transform the ptsx and ptsy to the car's perspective, referencing the car's current angle as 0 degrees.
	  // This prevents an error when finding a spline where two points have the same x coordinate and an infinite slope.
	  for (int i=0; i<ptsx.size(); ++i){
	  
	    double shift_x = ptsx[i]-ref_x;
	    double shift_y = ptsy[i]-ref_y;
	    
	    ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
	    ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
	  }

	  // create a spline
	  tk::spline s;

	  // set (x,y) points to the spline
	  s.set_points(ptsx,ptsy);

	  // Define the actual (x,y) points to use for the planner
	  vector<double> next_x_vals;
	  vector<double> next_y_vals;
	
	  // Start with all of the previous path points from the untraveled trajectory 
	  // (use what is already calculated instead of doing it all over).
	  for (int i=0; i<previous_path_x.size(); ++i){
	    next_x_vals.push_back(previous_path_x[i]);
	    next_y_vals.push_back(previous_path_y[i]);
	  }

	  // Interpolate points along the spline at increments that correlate to the desired velocity and acceleration.
	  double x_add_on = 0;
	  
	  // The distance traveled each 0.02 second increment needs to be determined according to the speed and acceleration.
	  double pos_add_on = fut_vel * 0.02; // Position increase at future velocity per 0.02 timestep.
 	  double incr_vel = ref_accel * 0.02; // Velocity increase increment due to acceleration per 0.02 second timestep.
	  double incr_pos = incr_vel * 0.02;  // Position increase increment due to acceleration per 0.02 second timestep.
	  double spd_limit_incr = ego.target_speed * 0.02;  //speed limit
	  // Fill up the rest of the path planner after filling it with previous path points returned by the simulator.
	  // Here it will always output a number of points equal to send_path_size.
	  // The spacing between x coordinates corresponds to velocity (and acceleration) in the direction of forward travel down the road.
	  // The y coordinates correspond to the lateral lane position.
	  for (int i=1; i<= send_path_size-previous_path_x.size(); ++i){
	    double x_point;
	    if ((pos_add_on+i*incr_pos)>(ego.target_speed*0.02)) { // Going over speed limit for this 0.02 second interval
	      x_point = x_add_on + ego.target_speed*0.02; // Stay under speed limit
	    }
	    else {
	      x_point = x_add_on + pos_add_on + i*incr_pos; // Increment according to desired speed and acceleration.
	    }
	    double y_point = s(x_point); // Find lateral lane position along the spline for the corresponding forward movement.
	    x_add_on = x_point;
	    double x_ref = x_point;
	    double y_ref = y_point;

	    // Rotate back from car's perspective angle to world perspective angle after having rotated that way earlier.
	    x_point = (x_ref *cos(ref_yaw)-y_ref*sin(ref_yaw));
	    y_point = (x_ref *sin(ref_yaw)+y_ref*cos(ref_yaw));

	    x_point += ref_x;
	    y_point += ref_y;

	    next_x_vals.push_back(x_point);
	    next_y_vals.push_back(y_point);
	  }
	  

	  json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";
	
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    first_sweep = false;/////////////
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
