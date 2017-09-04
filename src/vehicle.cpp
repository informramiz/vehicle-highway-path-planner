/*
 * vehicle.cpp
 *
 *  Created on: Aug 25, 2017
 *      Author: ramiz
 */

#include "vehicle.h"

/**
 *  start_state = [s, s_dot, s_ddot, d, d_dot, d_ddot]
 */
Vehicle::Vehicle(VectorXd start_state) {
  this->start_state_ = start_state;
}

/**
 * [ id, x, y, vx, vy, s, d]
 */
Vehicle::Vehicle(const vector<double> &sensor_fusion_data) {
  //access vx, vy which are at indexes (3, 4)
  double vx = sensor_fusion_data[3];
  double vy = sensor_fusion_data[4];
  //s-coordinate is at index 5
  double s = sensor_fusion_data[5];
  //d-coordinate is at index 6
  double d = sensor_fusion_data[6];

  //calculate total speed (s_dot)
  double s_dot = sqrt(vx*vx + vy*vy);

  VectorXd state(6);

  //start_state = [s, s_dot, s_ddot, d, d_dot, d_ddot]
  state << s, s_dot, 0, d, 0, 0;

  start_state_ = state;
}

Vehicle::~Vehicle() {

}
/**
 * predict new state (d, v, a) at timestep t
 * using equations of motion
 * given old state (d, v, a)
 */
VectorXd Vehicle::predict_coordinate_state(const VectorXd& values, double t) const {
  //predict distance travelled based on velocity and acceleration
  //S_t+1 = S_t + Vt * t + (a * t^2)/2
  double d = values[0] + (values[1] * t) + ((1.0 / 2.0) * values[2] * t*t);

  //predict new v
  //v = v + a*t
  double v = values[1] + values[2] * t;
  //predict new acceleration
  //as acceleration is assumed constant so same as old one
  double a = values[2];

  VectorXd new_state(3);
  new_state << d, v, a;

  return new_state;
}

/**
 * predict state of vehicle at timestep t
 * using equations of motion
 */
VectorXd Vehicle::state_at(double t) const {
  //extract s-values
  VectorXd s = start_state_.head(3);
  //extract d-values
  VectorXd d = start_state_.tail(3);

  //predict state of vehicle at timestep t
  //using equations of motion

  //predict s-coordinate values (s, v, a)
  VectorXd new_s_state = predict_coordinate_state(s, t);
  //similarly for d-coordinate
  VectorXd new_d_state = predict_coordinate_state(d, t);

  //concatenate s and d-coordinate values into one vector
  VectorXd new_state (this->start_state_.rows());
  new_state << new_s_state, new_d_state;

  return new_state;
}

