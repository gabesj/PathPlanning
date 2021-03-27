#include "cost.h"
#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "vehicle.h"

using std::string;
using std::vector;

const float COLLISION = 1;
const float SPEED = 1;
const float LANE_CHANGE = 1;


float collision_cost(const Vehicle &predicted_self, const vector<Vehicle> &trajectory, const vector<Vehicle> &predictions) {
  float cost = 0.0;
  double ahead_limit = predicted_self.s + predicted_self.preferred_buffer;
  double behind_limit = predicted_self.s - predicted_self.preferred_buffer;
  for (int i=0; i<predictions.size(); ++i) {
    if ((predicted_self.state.compare("KL") != 0) && trajectory[0].lane == predictions[i].lane && (predictions[i].s <= ahead_limit && predictions[i].s >= behind_limit)) {
      cost = 1;
    }
  }

  return cost;
}



float speed_cost(const Vehicle &predicted_self, const vector<Vehicle> &trajectory, const vector<Vehicle> &predictions) {
  //Cost is minimized as the trajectory's speeds approach the speed limit
  float cost = (predicted_self.target_speed - trajectory[0].v) / predicted_self.target_speed;

  return cost;
}



float lane_change_cost(const Vehicle &predicted_self, const vector<Vehicle> &trajectory, const vector<Vehicle> &predictions) {
  //Lane changing is penalized a little to prevent frivolous lane changing unless there is a decent cost benefit elsewhere
  float cost = 0.0;
  if (trajectory[0].lane != trajectory[1].lane) {
    cost = 0.05;
  }

  return cost;
}



float calculate_cost(const Vehicle &predicted_self, 
                     const vector<Vehicle> &predictions, 
                     const vector<Vehicle> &trajectory) {
  // Sum weighted cost functions to get total cost for trajectory.
  float cost = 0.0;
  vector<std::function<float(const Vehicle &, const vector<Vehicle> &, const vector<Vehicle> &)
    >> cf_list = {collision_cost, speed_cost, lane_change_cost};
  vector<float> weight_list = {COLLISION, SPEED, LANE_CHANGE};
  for (int i = 0; i < cf_list.size(); ++i) {
    float new_cost = weight_list[i]*cf_list[i](predicted_self, trajectory, predictions);
    cost += new_cost;
  }

  return cost;
}

