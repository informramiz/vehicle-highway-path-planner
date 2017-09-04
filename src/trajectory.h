/*
 * Trajectory.h
 *
 *  Created on: Aug 25, 2017
 *      Author: ramiz
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include "Eigen/Dense"

using Eigen::VectorXd;

/**
 * A passive object to contain Jerk Minimized Trajectory information
 */

struct Trajectory {
  ///duration of of trajectory
  double t;

  ///s-coordinate polynomial trajectory coefficients
  VectorXd s_coeffs;

  ///d-coordinate polynomial trajectory coefficients
  VectorXd d_coeffs;

  Trajectory(VectorXd &s_coeffs, VectorXd &d_coeffs, double t) {
    this->s_coeffs = s_coeffs;
    this->d_coeffs = d_coeffs;
    this->t = t;
  }
};



#endif /* TRAJECTORY_H_ */
