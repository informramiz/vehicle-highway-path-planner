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
    if (cost != 0.0) {
      printf("Cost for function %s: %f\n", this->cost_functions_names[i].c_str(), cost);
    }
    total_cost += cost;
  }

  return total_cost;
}

vector<double> CostFunctions::FindNearestApproachDuringTrajectory(const vector<Vehicle>& vehicles,
                                                          const FrenetTrajectory& trajectory,
                                                          bool consider_only_leading_vehicles) {

  //for each point in trajectory predict where other vehicle
  //will be at that point in time to see if there is a collision
  const int num_timesteps = trajectory.s_values.size();
  double timestep = 0.02; //each point is 0.02 (20 ms) timesteps away from other

  double min_distance = 999999;
  int nearest_vehicle_index = -1;
  double min_distance_ego_vehicle_s = -1;
  double time_of_appraoch = BUFFER_DISTANCE/ 0.02;
  int timesteps = 0;
  for (int i = 0; i < num_timesteps; ++i) {
    double s = trajectory.s_values[i];
    double d = trajectory.d_values[i];

    //calculate time at this point in trajectory
    double delta_t = (i) * timestep;
    int vehicle_index = FindMinimumDistanceVehicleIndex(vehicles, s,
        trajectory.lane, delta_t, consider_only_leading_vehicles);

    double distance = vehicle_index != -1 ? abs(vehicles[vehicle_index].s - s) : 9999999;
    if (distance < min_distance) {
      min_distance = distance;
      nearest_vehicle_index = vehicle_index;
      min_distance_ego_vehicle_s = s;
      time_of_appraoch = delta_t;
      timesteps = i;
    }
  }
  printf("Found nearest approach %f at time %f and timesteps %d\n", min_distance, time_of_appraoch, timesteps);
  return {min_distance, min_distance_ego_vehicle_s, double(nearest_vehicle_index), time_of_appraoch};
}

int CostFunctions::FindMinimumDistanceVehicleIndex(const vector<Vehicle> &vehicles,
                                                   const double ego_vehicle_s,
                                                   int ego_vehicle_lane,
                                                   double delta_t,
                                                   bool consider_only_leading_vehicles) {
  double min_distance = 999999;
  int min_distance_vehicle_index = -1;
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
    if (other_vehicle_predicted_s < ego_vehicle_s
        && consider_only_leading_vehicles) {
      continue;
    }

    double distance = abs(ego_vehicle_s - other_vehicle_predicted_s);
    if (distance < min_distance) {
      min_distance = distance;
      min_distance_vehicle_index = i;
    }
  }

  return min_distance_vehicle_index;
}


double CostFunctions::CollisionCost(const Vehicle &ego_vehicle,
                                    const vector<Vehicle> &vehicles,
                                    const FrenetTrajectory &trajectory,
                                    const int current_lane) {

  //for each point in trajectory predict where other vehicle
  //will be at that point in time to see if there is a collision
  vector<double> nearest_approach = FindNearestApproachDuringTrajectory(vehicles,
      trajectory, false);

  double distance = nearest_approach[0];
  double ego_vehicle_s_at_time = nearest_approach[1];
  double vehicle_index = nearest_approach[2];
  double time_of_approach = nearest_approach[3];

  if (distance > COLLISION_DISTANCE) {
    return 0.0;
  }

  cout << "nearest_approach(distance, s, vehicle index, time: " << endl;
  Utils::print_vector(nearest_approach);

  Vehicle other_vehicle = vehicles[vehicle_index];

  //returns = [lane, s, v, a]
  vector<double> other_vehicle_state_at_time = other_vehicle.state_at(time_of_approach);
  double other_vehicle_s = other_vehicle_state_at_time[1];
  double other_vehicle_v = other_vehicle_state_at_time[2];

  if (trajectory.lane == current_lane) {
    return 0.0;
  }

  double timesteps_away = time_of_approach / 0.02;
  printf("Collision at time %f and timesteps %f\n", time_of_approach, timesteps_away);
//  //if both s are equal then collision is imminent
//  if (ego_vehicle_s_at_time == other_vehicle_s) {
//    return 1.0;
//  } else if (ego_vehicle_s_at_time < other_vehicle_s //Case-2: If ego vehicle is behind and is moving faster
//      && ego_vehicle.v > other_vehicle_v) {
//    return exp(-timesteps_away);
//  } else if (ego_vehicle_s_at_time > other_vehicle_s ////Case-3: If ego vehicle is ahead and is moving slower
//      && ego_vehicle.v < other_vehicle_v) {
//    return exp(-timesteps_away);
//  }

  return exp(-timesteps_away/20.0);
//  return 0.0;
}

double CostFunctions::BufferCost(const Vehicle &ego_vehicle,
                                 const vector<Vehicle> &vehicles,
                                 const FrenetTrajectory &trajectory,
                                 const int current_lane) {
  //for each point in trajectory predict where other vehicle
  //will be at that point in time to see if there is a collision
//  double nearest_approach = FindNearestApproachDuringTrajectory(vehicles,
//      trajectory, true);

//  double end_s = trajectory.s_values[trajectory.s_values.size()-1];
//  double nearest_approach = FindMinimumDistanceToVehicle(vehicles, end_s,
//      trajectory.lane, trajectory.d_values.size() * 0.02, true);

  vector<double> nearest_approach = FindNearestApproachDuringTrajectory(vehicles,
        trajectory, true);

    double distance = nearest_approach[0];
    double ego_vehicle_s = nearest_approach[1];
    double vehicle_index = nearest_approach[2];
    double time_of_approach = nearest_approach[3];

  if (distance > BUFFER_DISTANCE) {
    return 0.0;
  }

  return Utils::logistic((2 * BUFFER_DISTANCE * (1 - time_of_approach) / distance));
}

double CostFunctions::ChangeLaneCost(const Vehicle &ego_vehicle,
                  const vector<Vehicle> &vehicles,
                  const FrenetTrajectory &trajectory,
                  const int current_lane) {
  double end_d = trajectory.d_values[trajectory.d_values.size()-1];
  int end_lane = MapUtils::GetLane(end_d);

  //we want to penalize lane change as it is not cheap
  if (current_lane != trajectory.lane) {
    printf("start_lane, end_lane: %d, %d\n", current_lane, trajectory.lane);
    return 1.0;
  }

  return 0;
}
