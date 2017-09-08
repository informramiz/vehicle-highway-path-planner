/*
 * Trajectory.h
 *
 *  Created on: Aug 25, 2017
 *      Author: ramiz
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <vector>

using namespace std;

/**
 * A passive object to contain Jerk Minimized Trajectory information
 */

struct CartesianTrajectory {
  //cartesian coordinates
  vector<double> x_values;
  vector<double> y_values;

  CartesianTrajectory(const vector<double> &x_values, const vector<double> &y_values) {
    this->x_values = x_values;
    this->y_values = y_values;
  }
};

struct FrenetTrajectory {
  //Frenet coordinates
  vector<double> s_values;
  vector<double> d_values;

  FrenetTrajectory(const vector<double> &s_values, const vector<double> &d_values) {
    this->s_values = s_values;
    this->d_values = d_values;
  }
};


#endif /* TRAJECTORY_H_ */
