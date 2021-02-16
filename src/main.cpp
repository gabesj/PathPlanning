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
#include "vehicle.h"////////
#include "vehicle.cpp"////////
#include "cost.cpp"////////

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

  ///start in lane 1.  Lane 0 is left lane, lane 2 is right lane.
  int lane = 1;

  ///have a reference velocity to target
  double ref_vel = 0.0; //49.5; //mph

  ///create the self-driving car
  double init_s = 0.0;///
  double init_d = 0.0;///
  double init_speed = 0.0;///
  double init_accel = 0.0;///
  double init_x = 0.0;///
  double init_y = 0.0;///
  double speed_limit = 50.0;///
  double target_speed = speed_limit - 1.0;///
  double max_accel = 0.44;///
  vector<double> config_data = {target_speed, max_accel};///
  Vehicle ego = Vehicle(lane, init_d, init_s, init_speed, init_accel, init_x, init_y); ///
  ego.configure(config_data);///
  ego.state = "KL";///

  ///other variables
  float ahead_horizon = 50.0;
  float behind_horizon = 20.0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel,&target_speed]
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
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
	  ///create a vector of Vehicles representing the sensor fusion info
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
	    int other_lane = -1;
            if(other_d>=0.0 && other_d<4.0){
              other_lane = 0;
            } else if (other_d>=4.0 && other_d<8.0){
              other_lane = 1;
            } else {
              other_lane = 2;
            }
            assert(other_lane != -1);

            Vehicle other_car = Vehicle(other_lane, other_d, other_s, other_speed, other_a, other_x, other_y);
            other_cars.push_back(other_car);
	    
	  }

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
	  /// size of the last calculated path
	  int prev_size = previous_path_x.size();

	  if(prev_size>0) {
	    car_s = end_path_s;
	  }
	  ////predict the future locations of the other vehicles.  prev_size is the number of left over 0.02 second increments 
	  ////from the previous planned path that are returned from the simulator.  the first elements in previous_path_x and 
	  ////previous_path_y are almost exactly where the car's localization data says it currently is, and the rest of the 
	  ////previous_path are still future points.  Use prev_size to estimate a time step uses the assumption that the next
	  ////time step will be nearly the same length as the previous one.

	  ///get predictions for the future locations of other vehicles within a certain distance ahead and behind  
//	  vector<Vehicle> predictions = generate_predictions(&ego, &other_cars, prev_size, ahead_horizon, behind_horizon);	  
	  
	  bool too_close = false;
	  ////////double accel_factor = 1;//////////////////////////////////
	  ////////double check_car_s; ////////////////////////////
	  ////////double check_speed;//////////////////
	  //find ref_v to use
	  for(int i=0; i<sensor_fusion.size(); ++i) {
	    //car is in my lane
	    float d = sensor_fusion[i][6];
	    if(d<(2+4*lane+2) && d>(2+4*lane-2)) {
	      double vx = sensor_fusion[i][3];
	      double vy = sensor_fusion[i][4];
	      double check_speed = sqrt(vx*vx+vy*vy);
	      ////////check_speed = sqrt(vx*vx+vy*vy);
	      double check_car_s = sensor_fusion[i][5];
	      ////////check_car_s = sensor_fusion[i][5];

	      check_car_s+=((double)prev_size*0.02*check_speed); //if using previous points can project s value out a time step
	      // check s values greater than mine and s gap
	      if((check_car_s > car_s) && ((check_car_s-car_s) < 30)) {//other car is ahead of me by less than 30m
	        // Do some logic here, lower reference velocity so we don't crash into the car in front of us
		// could also flag to try to change lanes.
		//ref_vel = 29.5; // mph
		too_close = true;
		//if(lane>0) {
		//  lane = 0;
		//}
	        ////////std::cout << "check_speed = " << check_speed << ", car_speed = " << car_speed << std::endl;
	      }
	    }
	  }
	  
	  if(too_close) {
	    ////////accel_factor = abs(60/((check_car_s-car_s))); //becomes more relevant as the distance to preceding car decreases
	    ref_vel -= 0.224;
	    ////////if(((ref_vel*accel_factor)<0.4) && ((0.75*check_speed)<car_speed)){ //don't exceed jerk limit of 10m/s2
	    ////////  ref_vel -= ref_vel * accel_factor;
	      
	    ////////}
	    ////////else if((0.75*check_speed)<car_speed) { 
	    ////////  ref_vel -= 0.4;
	    ////////}
	    ////////else { //if slower than car ahead, speed up a bit
	    ////////  ref_vel += 0.1;
	    ////////}
	  }
	  else if(ref_vel < target_speed) {
	    ////////accel_factor = (49-ref_vel)*(49-ref_vel)/10; //becomes more relevant as the gap between speed and speed limit increases
	    ////////if(((ref_vel*accel_factor)<0.4) && (ref_vel>20)) {
	    ////////  ref_vel += ref_vel * accel_factor;
	    ////////}
	    ////////else {
	    ////////  ref_vel += 0.4;
	    ref_vel += .224;
	    ////////}
	  }

	  /// create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
  	  /// later we will interpolate these waypoints with a spline and fill it in with more points that control speed.
	  vector<double> ptsx;
	  vector<double> ptsy;	 
	  
	  /// reference x, y, yaw states
	  /// either we will reference the starting point as where the car is or at the previous paths end point
	  double ref_x = car_x;
	  double ref_y = car_y;
	  double ref_yaw = deg2rad(car_yaw); 

	  ///if previous size is almost empty, use the car as starting reference and extrapolate the previous point
	  if(prev_size < 2){
	    //double prev_car_x = car_x - cos(car_yaw);
	    //double prev_car_y = car_y - sin(car_yaw);
	    double prev_car_x = car_x - cos(ref_yaw);
	    double prev_car_y = car_y - sin(ref_yaw);
	    
	    ptsx.push_back(prev_car_x);
	    ptsx.push_back(car_x);
	
	    ptsy.push_back(prev_car_y);
	    ptsy.push_back(car_y);
	  }
	  /// otherwise use the previous path's end point as a starting reference
	  else {
	    ref_x = previous_path_x[prev_size-1];
	    ref_y = previous_path_y[prev_size-1];
	    
	    double ref_x_prev = previous_path_x[prev_size-2];
	    double ref_y_prev = previous_path_y[prev_size-2];
	    ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

	    /// use two points that make the path tangent to the previous path's end point
	    ptsx.push_back(ref_x_prev);
	    ptsx.push_back(ref_x);

	    ptsy.push_back(ref_y_prev);
	    ptsy.push_back(ref_y);
	  }

	  /// In Frenet coordinates find three future waypoints ahead of the starting reference at 30,60, and 90 meters ahead
	  vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
	  vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
	  vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);

	  ptsx.push_back(next_wp0[0]);
	  ptsx.push_back(next_wp1[0]);
	  ptsx.push_back(next_wp2[0]);

	  ptsy.push_back(next_wp0[1]);
	  ptsy.push_back(next_wp1[1]);
	  ptsy.push_back(next_wp2[1]);
	  
	  /// the ptsx and ptsy vectors now each have 5 points: previous point, current point, point 30m ahead, 60m ahead, and 90m ahead

	  /// transform the ptsx and ptsy to the car's perspective, referencing the car's current angle as 0 degrees.
	  for (int i=0; i<ptsx.size(); ++i){
	  
	    double shift_x = ptsx[i]-ref_x;
	    double shift_y = ptsy[i]-ref_y;
	    
	    ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
	    ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
	  }

	  /// create a spline
	  tk::spline s;

	  /// set (x,y) points to the spline
	  s.set_points(ptsx,ptsy);

	  /// Define the actual (x,y) points we will use for the planner
	  vector<double> next_x_vals;
	  vector<double> next_y_vals;
	
	  /// Start with all of the previous path points from last time (use what is already calculated instead of doing it all over)
	  /// previous_path_x and previous_path_y are the path points still in the future, as returned by the simulator.
	  for (int i=0; i<previous_path_x.size(); ++i){
	    next_x_vals.push_back(previous_path_x[i]);
	    next_y_vals.push_back(previous_path_y[i]);
	  }

	  /// Calculate how to break up spline points so that we travel at our desired reference velocity
	  double target_x = 30.0;
	  double target_y = s(target_x); ///get the y value of the spline at a given x value
	  double target_dist = sqrt(target_x*target_x+target_y*target_y);

	  double x_add_on = 0;

	  /// Fill up the rest of our path planner after filling it with previous path points returned by the simulator
	  /// here it will always output 50 points ///// changed to 100 points
	  //for (int i=1; i<= 50-previous_path_x.size(); ++i){
	  for (int i=1; i<= 100-previous_path_x.size(); ++i){
	/////////////////try accelerating here///////////////////////////
	    double N = (target_dist/(0.02*ref_vel/2.224)); /// number of 0.02 second time steps to get to the target at correct speed
	    double x_point = x_add_on+(target_x)/N; /// each iteration, go one step in x and then find the spline's y value
	    double y_point = s(x_point);

	    x_add_on = x_point;

	    double x_ref = x_point;
	    double y_ref = y_point;

	    /// rotate back from car's perspective angle to world perspective angle after having rotated that way earlier.
	    x_point = (x_ref *cos(ref_yaw)-y_ref*sin(ref_yaw));
	    y_point = (x_ref *sin(ref_yaw)+y_ref*cos(ref_yaw));

	    x_point += ref_x;
	    y_point += ref_y;

	    next_x_vals.push_back(x_point);
	    next_y_vals.push_back(y_point);
	  }
	    
	    
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	/* /// stay in lane constant average speed
	  double dist_inc = 0.5;
	  for(int i=0; i<50; i++) {
	    double next_s = car_s+(i+1)*dist_inc; ///use i+1 because i would be just the current point and wouldn't move the car.
	    double next_d = 6;
	    vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

	    next_x_vals.push_back(xy[0]);
	    next_y_vals.push_back(xy[1]);
	  }
	*/

	/*  ///Straight Line, too much jerk
	  double dist_inc = 0.5;
	  for (int i = 0; i < 50; ++i) {
  	    next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
  	    next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
	  }	  
	*/

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
