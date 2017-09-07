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
                       const Trajectory &trajectory);
  double CollisionCost(const Vehicle &ego_vehicle,
                       const vector<Vehicle> &vehicles,
                       const Trajectory &trajectory);

private:
  const double BUFFER_DISTANCE = 30;
  const double GOAL_S = 6945.554;

  //define a typdef for function pointer
  typedef double (CostFunctions::*cost_function_ptr)(
      const Vehicle &ego_vehicle, const vector<Vehicle> &vehicles,
      const Trajectory &trajectory);

  const vector<cost_function_ptr> cost_functions_ = {
      &CostFunctions::CollisionCost,
  };

  //weight for each cost function
  const vector<double> cost_functions_weights_ = { 20, //CollisionCost
      };

  const vector<string> cost_functions_names = { "CollisionCost", };
};

#endif /* COST_FUNCTIONS_H_ */
