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
  double init_vx = 0.0;///
  double init_vy = 0.0;///
  double init_yaw = 0.0;///
  double speed_limit = 50.0;///mph
  double target_speed = (speed_limit - 1.0) / 2.224;/////Dividing by 2.224 converts mph to m/s
  double max_accel = 10-1; // m/s^2         0.2;///0.44;///
  double ahead_horizon = 50.0;
  double behind_horizon = 50.0;
  double ref_accel = 0.0;
  vector<double> config_data = {target_speed, max_accel, ahead_horizon, behind_horizon};///
  Vehicle ego = Vehicle(lane, init_d, init_s, init_speed, init_accel, init_x, init_y, init_vx, init_vy, init_yaw); ///
  ego.configure(config_data);///
  ego.state = "KL";///
  bool lane_changing = false;
  int next_lane = lane;

  ///other variables
  int first_sweep = true;
  int send_path_size = 40;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel,&ego,&first_sweep,&send_path_size,&ref_accel,&lane_changing,&next_lane]
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
          std::cout << "********************************" << ego.state << "*******************" << std::endl;
          // Main car's localization Data
          //double car_x = j[1]["x"];
          //double car_y = j[1]["y"];
          //double car_s = j[1]["s"];
          //double car_d = j[1]["d"];
          //double car_yaw = j[1]["yaw"];
          //double car_speed = j[1]["speed"];
	  ego.x = j[1]["x"];
          ego.y = j[1]["y"];
          ego.s = j[1]["s"];
          ego.d = j[1]["d"];
          ego.yaw = j[1]["yaw"];
          ego.v = j[1]["speed"];
	  ego.v = ego.v/2.224;/////Dividing by 2.224 converts mph to m/s
	  //std::cout << "ego.v = " << ego.v << std::endl;
	  ego.a = ref_accel;
	  ego.lane = round((ego.d-2)/4);
	  
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
            double other_speed = sqrt(other_vx*other_vx+other_vy*other_vy); // in m/sec
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
	    ///assumes that velocity is in the direction of the lane.  This prevents misattributing a curve in the road to a lane change,
	    /// but it also does not reflect the lateral and straight differentiation in an actual lane change.
            Vehicle other_car = Vehicle(other_lane, other_d, other_s, other_speed, other_a, other_x, other_y, other_vx, other_vy, other_yaw);
            other_cars.push_back(other_car);
	    //std::cout << "other car " << i << " speed is " << other_speed << std::endl;
	  }
	  //std::cout << "other_cars.size() = " << other_cars.size() << std::endl;
          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
	  /// size of the last calculated path
	  int prev_size = previous_path_x.size();
	  /// approximate average number of 0.02 second increments in previous time steps
	  double timestep_running_avg;
	  if(first_sweep){
	    timestep_running_avg = 10;
	  }
	  else {
	    timestep_running_avg = 0.1*(send_path_size-prev_size) + 0.9*timestep_running_avg;
	  }
	  //std::cout << "last size = " << send_path_size-prev_size << ", timestep_running_avg = " << timestep_running_avg << std::endl;
/*
	  if(prev_size>0) {
	    //car_s = end_path_s;
	    ego.s = end_path_s;
	  }
*/
	  ////predict the future locations of the other vehicles.  prev_size is the number of left over 0.02 second increments 
	  ////from the previous planned path that are returned from the simulator.  the first elements in previous_path_x and 
	  ////previous_path_y are almost exactly where the car's localization data says it currently is, and the rest of the 
	  ////previous_path are still future points.

	  ///get predictions for the future locations of other vehicles within a certain distance ahead and behind  
	  
	  vector<Vehicle> predictions = generate_predictions(ego, other_cars, prev_size); //timestep_running_avg);	  
	  Vehicle predicted_self = predict_self(ego, prev_size); // timestep_running_avg);
	  ///choose the best state for the next time step
	  /// the "Vehicles" in trajectory are used just to hold lane and speed info for the trajectory
	  vector<Vehicle> trajectory;
	  if (lane_changing != true) {
            trajectory = ego.choose_next_state(predictions, predicted_self);
          }
	  else {
            Vehicle desired_predicted_self = predicted_self;
            desired_predicted_self.lane = next_lane;
            trajectory = ego.generate_trajectory("KL", predictions, desired_predicted_self);
            
          }

	  if(first_sweep) {
	    ref_accel = ego.max_acceleration;
	  }
	  else {
	    ref_accel = std::min(trajectory[0].a, trajectory[1].a);/////////////////////
	  }
          std::cout << "ego.lane = " << ego.lane << "  trajectory[1].lane = " << trajectory[1].lane << "  lane_changing = " << lane_changing << "  next_lane = " << next_lane << std::endl;
          std::cout << "ego.d = " << ego.d << std::endl;
          std::cout << "ego.s = " << ego.s << std::endl;
	  std::cout << "ego.v = " << ego.v << std::endl;
          std::cout << "ego.a = " << ego.a << std::endl;
	  std::cout << "ref_accel = " << ref_accel << std::endl;
          if (ego.lane != trajectory[1].lane) {
            lane_changing = true;
	    next_lane = trajectory[1].lane;
          }
	  else if (ego.lane == next_lane) {
	    lane_changing = false;
	  }
   	  lane = next_lane; 
          ego.state = trajectory[1].state; 
	  
 	    

/*
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

	      ///check_car_s+=((double)prev_size*0.02*check_speed); //if using previous points can project s value out a time step
	      check_car_s+=(timestep_running_avg*0.02*check_speed); 
	      // check s values greater than mine and s gap
	      //if((check_car_s > ego.s) && ((check_car_s-ego.s) < 30)) {//other car is ahead of me by less than 30m
	      double my_next_s = ego.s + (timestep_running_avg*0.02*ego.v);
	      //if((check_car_s > ego.s) && ((check_car_s-ego.s) < 30)) {
	      if((check_car_s > my_next_s) && ((check_car_s-my_next_s) < 30)) {
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
*/
	  
	  //std::cout << "too close? " << too_close << " , ref_vel = " << ref_vel << std::endl;
	  //prevent backward motion on first frame if starting at 0 velocity and a car is in front
//	  if(ref_vel<0){
//	    ref_vel += .3;
//	  }
	  /// create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
  	  /// later we will interpolate these waypoints with a spline and fill it in with more points that control speed.
	  vector<double> ptsx;
	  vector<double> ptsy;	 
	  
	  /// reference x, y, yaw states
	  /// either we will reference the starting point as where the car is or at the previous paths end point
	  double ref_x = ego.x;
	  double ref_y = ego.y;
	  double ref_yaw = deg2rad(ego.yaw); 

	  ///if previous size is almost empty, use the car as starting reference and extrapolate the previous point
	  if(prev_size < 2){
	    //double prev_car_x = car_x - cos(car_yaw);
	    //double prev_car_y = car_y - sin(car_yaw);
	    double prev_car_x = ego.x - cos(ref_yaw);
	    double prev_car_y = ego.y - sin(ref_yaw);
	    
	    ptsx.push_back(prev_car_x);
	    ptsx.push_back(ego.x);
	
	    ptsy.push_back(prev_car_y);
	    ptsy.push_back(ego.y);
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
	  //vector<double> next_wp0 = getXY(ego.s+30,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
	  //vector<double> next_wp1 = getXY(ego.s+60,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
	  //vector<double> next_wp2 = getXY(ego.s+90,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
	  std::cout << "end_path_s = " << end_path_s << std::endl;
	  
	  vector<double> next_wp0;
	  vector<double> next_wp1;
	  vector<double> next_wp2;
	  
	  if(first_sweep) {
	    next_wp0 = getXY(ego.s+30,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
	    next_wp1 = getXY(ego.s+60,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
	    next_wp2 = getXY(ego.s+90,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
	    //std::cout << "first_sweep" << std::endl;
	  }
	  else {
	    next_wp0 = getXY(end_path_s+30,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
	    next_wp1 = getXY(end_path_s+60,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
	    next_wp2 = getXY(end_path_s+90,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
	    //std::cout << "else" << std::endl;
	  }

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
//	  double target_x = 30.0;
//	  double target_y = s(target_x); ///get the y value of the spline at a given x value
//	  double target_dist = sqrt(target_x*target_x+target_y*target_y);

	  double x_add_on = 0;
	  double fut_vel = 0;
	  
	  if(prev_size > 12) {
	    double x1 = previous_path_x[prev_size-1];
	    double y1 = previous_path_y[prev_size-1];
	    double x2 = previous_path_x[prev_size-12];
	    double y2 = previous_path_y[prev_size-12];
	    fut_vel = distance(x1,y1,x2,y2)/(11*0.02);
	    //std::cout << "(x1,y1) = (" << x1 << "," << y1 << ") and (x2,y2) = (" << x2 << "," << y2 << ")" << std::endl;
	  }
	  std::cout << "fut_vel = " << fut_vel << std::endl;
	  double pos_add_on = fut_vel * 0.02; ///ego.v * 0.02; ///position increase at current velocity per 0.02 timestep 
 	  double incr_vel = ref_accel * 0.02; ///velocity increase increment due to acceleration per 0.02 second timestep
	  double incr_pos = incr_vel * 0.02; //incr_vel * 0.02; ///position increase increment due to acceleration per 0.02 second timestep
	  double spd_limit_incr = ego.target_speed * 0.02;
	  /// Fill up the rest of our path planner after filling it with previous path points returned by the simulator
	  /// here it will always output 50 points ///// changed to 100 points
	  //for (int i=1; i<= 50-previous_path_x.size(); ++i){
	  for (int i=1; i<= send_path_size-previous_path_x.size(); ++i){
	/////////////////try accelerating here///////////////////////////
	    /// number of 0.02 second time steps to get to the target at current speed. Dividing by 2.224 converts mph to meters/sec
	    //double N = (target_dist/(0.02*ref_vel/2.224)); 
	    //double inc_accel = ref_accel * 10 / 1275 * 0.9; /////////don't exceed 90% of the allowable acceleration limit
	    //vel_add_on += inc_accel;
	    	    
	    ///double x_point = x_add_on+(target_x)/N; /// each iteration, go one step in x and then find the spline's y value
	    /// the distance between each point varies according to the acceleration
	    double x_point;
	    if ((pos_add_on+i*incr_pos)>(ego.target_speed*0.02)) { // going over speed limit for this 0.02 second interval
	      x_point = x_add_on + ego.target_speed*0.02;
	    }
	    else {
	      x_point = x_add_on + pos_add_on + i*incr_pos;
	    }
	    
	    //double x_point = x_add_on + pos_add_on + i*incr_pos;

	    //incr_pos += incr_pos;
	    //if ((x_point*incr_vel*i/2*0.02+ego.v)>=ego.target_speed) {
	    //  x_point = x_point - incr_pos;
	    //}
	    //std::cout << "x_point = " << x_point << " x_add_on = " << x_add_on << " pos_add_on = " << pos_add_on << " incr_pos = " << incr_pos << std::endl;
	    
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
	  /// for help in predicting car locations 
	  

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
