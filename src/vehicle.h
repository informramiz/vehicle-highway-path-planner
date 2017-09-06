/*
 * vehicle.h
 *
 * Helper class. Non-ego vehicles move w/ constant acceleration
 *
 *  Created on: Aug 25, 2017
 *      Author: ramiz
 */

#ifndef VEHICLE_H_
#define VEHICLE_H_

#include <iostream>
#include <vector>
#include "Eigen/Dense"

using namespace std;
using Eigen::VectorXd;


class Vehicle {
public:
  /**
   *  start_state = [s, s_dot, s_ddot, d, d_dot, d_ddot]
   */
  Vehicle(VectorXd start_state);
  /**
   * [ id, x, y, vx, vy, s, d]
   */
  Vehicle(const vector<double> &sensor_fusion_data);

  virtual ~Vehicle();

  /**
   * Predict and returns the state of the vehicle at given timestep
   */
  VectorXd state_at(double t) const;

  double get_d() const;
  double get_s() const;

  void set_s(double s) {
    start_state_[0] = s;
  }

private:
  VectorXd start_state_;

  /**
   * predict new state (d, v, a) at timestep t
   * using equations of motion
   * given old state (d, v, a)
   */
  Eigen::VectorXd predict_coordinate_state(const Eigen::VectorXd& s, double t) const;
};

#endif /* VEHICLE_H_ */
