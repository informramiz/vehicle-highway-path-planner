/*
 * cost_functions.h
 *
 *  Created on: Sep 7, 2017
 *      Author: ramiz
 */

#ifndef COST_FUNCTIONS_H_
#define COST_FUNCTIONS_H_

#include <iostream>
#include <vector>
#include <string>
#include "trajectory.h"
#include "vehicle.h"
#include "utils.h"

using namespace std;

class CostFunctions {
public:
  virtual ~CostFunctions();
  CostFunctions();

  double CalculateCost(const Vehicle &ego_vehicle,
                       const vector<Vehicle> &vehicles,
                       const CartesianTrajectory &trajectory,
                       const int current_lane);
  double CollisionCost(const Vehicle &ego_vehicle,
                       const vector<Vehicle> &vehicles,
                       const FrenetTrajectory &trajectory,
                       const int current_lane);
  double BufferCost(const Vehicle &ego_vehicle,
                    const vector<Vehicle> &vehicles,
                    const FrenetTrajectory &trajectory,
                    const int current_lane);

  double ChangeLaneCost(const Vehicle &ego_vehicle,
                    const vector<Vehicle> &vehicles,
                    const FrenetTrajectory &trajectory,
                    const int current_lane);

private:
  int FindMinimumDistanceVehicleIndex(const vector<Vehicle> &vehicles,
                                      const double s,
                                      int ego_vehicle_lane,
                                      double delta_t,
                                      bool consider_only_leading_vehicles);

  const double COLLISION_DISTANCE = 10;
  const double BUFFER_DISTANCE = 30;
  const double GOAL_S = 6945.554;
  const double VEHICLE_RADIUS = 1.5;

  //define a typdef for function pointer
  typedef double (CostFunctions::*cost_function_ptr)(
      const Vehicle &ego_vehicle, const vector<Vehicle> &vehicles,
      const FrenetTrajectory &trajectory,
      const int current_lane);

  vector<double> FindNearestApproachDuringTrajectory(
      const vector<Vehicle>& vehicles, const FrenetTrajectory& trajectory,
      bool consider_only_leading_vehicles);

  const vector<cost_function_ptr> cost_functions_ = {
      &CostFunctions::CollisionCost,
      &CostFunctions::BufferCost,
      &CostFunctions::ChangeLaneCost,
  };

  //weight for each cost function
  const vector<double> cost_functions_weights_ = {
      100, //CollisionCost
      20, //BufferCost
      5, //ChangeLaneCost
  };

  const vector<string> cost_functions_names = {
      "CollisionCost",
      "BufferCost",
      "ChangeLaneCost",
  };
};

#endif /* COST_FUNCTIONS_H_ */
