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

struct Trajectory {
  //cartesian coordinates
  vector<double> x_values;
  vector<double> y_values;

  Trajectory(const vector<double> &x_values, const vector<double> &y_values) {
    this->x_values = x_values;
    this->y_values = y_values;
  }
};



#endif /* TRAJECTORY_H_ */
