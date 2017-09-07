/*
 * cost_functions.cpp
 *
 *  Created on: Sep 7, 2017
 *      Author: ramiz
 */

#include "cost_functions.h"

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

    printf("Cost for function %s: %f", this->cost_functions_names[i].c_str(), cost);

    total_cost += cost;
  }

  return total_cost;
}

double CostFunctions::CollisionCost(const Vehicle &ego_vehicle,
                     const vector<Vehicle> &vehicles,
                     const Trajectory &trajectory) {
  return 0.0;
}
