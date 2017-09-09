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
                                    const CartesianTrajectory &trajectory,
                                    const int current_lane) {
  //convert to FrenetTrajectory
  FrenetTrajectory frenet_trajectory = MapUtils::CartesianToFrenet(trajectory, ego_vehicle.yaw);

  double total_cost = 0.0;

  for (int i = 0; i < cost_functions_.size(); ++i) {
    double cost = cost_functions_weights_[i] * (this->*cost_functions_[i])(ego_vehicle, vehicles, frenet_trajectory, current_lane);
    if (cost > 0) {
      printf("Cost for function %s: %f\n", this->cost_functions_names[i].c_str(), cost);
    }
    total_cost += cost;
  }

  return total_cost;
}

double CostFunctions::FindNearestApproachDuringTrajectory(const vector<Vehicle>& vehicles,
                                                          const FrenetTrajectory& trajectory,
                                                          bool consider_only_leading_vehicles) {

  //for each point in trajectory predict where other vehicle
  //will be at that point in time to see if there is a collision
  const int num_timesteps = trajectory.s_values.size();
  double timestep = 0.02; //each point is 0.02 (20 ms) timesteps away from other

  //ignore previous path points as we are assuming ego vehicle
  //is already at the end of that path
//  int start_point = previous_path_size > 0 ? previous_path_size - 1 : 0;
//  cout << "start point: " << start_point << endl;


  double min_distance = 999999;
  for (int i = 0; i < num_timesteps; ++i) {
    double s = trajectory.s_values[i];
    double d = trajectory.d_values[i];

    //calculate time at this point in trajectory
    double delta_t = (i) * timestep;
    double nearest_approach = FindMinimumDistanceToVehicle(vehicles, s,
        d, delta_t, consider_only_leading_vehicles);

    if (nearest_approach < min_distance) {
      min_distance = nearest_approach;
    }
  }
  return min_distance;
}

double CostFunctions::FindMinimumDistanceToVehicle(const vector<Vehicle> &vehicles,
                                                   const double ego_vehicle_s,
                                                   const double ego_vehicle_d,
                                                   double delta_t,
                                                   bool consider_only_leading_vehicles) {
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

    //we only care about leading vehicles
    if (other_vehicle_predicted_s <= ego_vehicle_s
        && consider_only_leading_vehicles) {
      continue;
    }

    double distance = abs(ego_vehicle_s - other_vehicle_predicted_s);
    if (distance < min_distance) {
      min_distance = distance;
    }
  }

  return min_distance;
}


double CostFunctions::CollisionCost(const Vehicle &ego_vehicle,
                                    const vector<Vehicle> &vehicles,
                                    const FrenetTrajectory &trajectory,
                                    const int current_lane) {

  //for each point in trajectory predict where other vehicle
  //will be at that point in time to see if there is a collision
  double nearest_approach = FindNearestApproachDuringTrajectory(vehicles,
      trajectory, false);

  if (nearest_approach < 10) {
    return 1.0;
  } else {
    return 0.0;
  }
}

double CostFunctions::BufferCost(const Vehicle &ego_vehicle,
                                 const vector<Vehicle> &vehicles,
                                 const FrenetTrajectory &trajectory,
                                 const int current_lane) {
  //for each point in trajectory predict where other vehicle
  //will be at that point in time to see if there is a collision
//  double nearest_approach = FindNearestApproachDuringTrajectory(vehicles,
//      trajectory, true);

  double end_d = trajectory.d_values[trajectory.d_values.size()-1];
  double end_s = trajectory.s_values[trajectory.s_values.size()-1];
  double nearest_approach = FindMinimumDistanceToVehicle(vehicles, end_s, end_d, trajectory.d_values.size() * 0.02, true);

  if (nearest_approach > BUFFER_DISTANCE) {
    return 0.0;
  }

  return Utils::logistic(2 * BUFFER_DISTANCE / nearest_approach);
}

double CostFunctions::ChangeLaneCost(const Vehicle &ego_vehicle,
                  const vector<Vehicle> &vehicles,
                  const FrenetTrajectory &trajectory,
                  const int current_lane) {
  double start_d = trajectory.d_values[0];
  int start_lane = MapUtils::GetLane(start_d);

  double end_d = trajectory.d_values[trajectory.d_values.size()-1];
  int end_lane = MapUtils::GetLane(end_d);

  //we want to penalize lane change as it is not cheap
  if (current_lane != end_lane) {
    printf("start_lane, end_lane: %d, %d\n", start_lane, ego_vehicle.lane);
    return 1.0;
  }

  return 0;
}
