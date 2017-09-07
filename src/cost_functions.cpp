/*
 * cost_functions.cpp
 *
 *  Created on: Sep 7, 2017
 *      Author: ramiz
 */

#include <math.h>
#include "cost_functions.h"
#include "map_utils.h"

CostFunctions::~CostFunctions() {

}

CostFunctions::CostFunctions() {

}

double CostFunctions::CalculateCost(const Vehicle &ego_vehicle,
                       const vector<Vehicle> &vehicles,
                       const Trajectory &trajectory) {
  double total_cost = 0.0;

  for (int i = 0; i < cost_functions_.size(); ++i) {
    double cost = cost_functions_weights_[i] * (this->*cost_functions_[i])(ego_vehicle, vehicles, trajectory);
    if (cost > 0) {
      printf("Cost for function %s: %f", this->cost_functions_names[i].c_str(), cost);
    }
    total_cost += cost;
  }

  return total_cost;
}

double CostFunctions::CollisionCost(const Vehicle &ego_vehicle,
                     const vector<Vehicle> &vehicles,
                     const Trajectory &trajectory) {

  //get first point as it will be from previous path
  //we will use it for angle calculation which is
  //tangent line (slope) between two points
  double prev_x = trajectory.x_values[0];
  double prev_y = trajectory.y_values[0];

  //for each point in trajectory predict where other vehicle
  //will be at that point in time to see if there is a collision
  const int num_timesteps = trajectory.x_values.size();
  double timestep = 0.02; //each point is 0.02 (20 ms) timesteps away from other

  for (int i = 0; i < num_timesteps; ++i) {
    double next_x = trajectory.x_values[i];
    double next_y = trajectory.y_values[i];

    //calculate angle of vehicle at this point in time
    //which is slope (tangent) between this and previous point
    double yaw = atan2(next_y - prev_y, next_x - prev_x);

    //convert current trajectory point to Frenet coordinate system
    vector<double> sd = MapUtils::getFrenet(next_x, next_y, yaw);
    //calculate time at this point in trajectory
    double delta_t = i * timestep;

    double nearest_approach = FindMinimumDistanceToVehicle(vehicles, sd[0], sd[1], delta_t);
    if (nearest_approach < 2*VEHICLE_RADIUS) {
      return 1.0;
    }

    prev_x = next_x;
    prev_y = next_y;
  }
  return 0.0;
}

double CostFunctions::FindMinimumDistanceToVehicle(const vector<Vehicle> &vehicles,
                                    const double ego_vehicle_s,
                                    const double ego_vehicle_d,
                                    double delta_t) {
  int ego_vehicle_lane = MapUtils::GetLane(ego_vehicle_d);
  double min_distance = 999999;

  //for compiler optimization get vehicles size first
  //and not in loop
  const int vehicles_count = vehicles.size();
  for (int i = 0; i < vehicles_count; ++i) {
    //we are only interested in vehicles in our lane
    if (vehicles[i].lane != ego_vehicle_lane) {
      continue;
    }

    //predict vehicle state at time delta_t
    double other_vehicle_predicted_s = vehicles[i].state_at(delta_t)[1];

    double distance = abs(ego_vehicle_s - other_vehicle_predicted_s);
    if (distance < min_distance) {
      min_distance = distance;
    }
  }

  return min_distance;
}
