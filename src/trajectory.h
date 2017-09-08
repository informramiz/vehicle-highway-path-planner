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

  CartesianTrajectory ExtractTrajectory(int points_count=50) {
    vector<double> new_x_values(this->x_values.begin(), this->x_values.begin() + points_count);
    vector<double> new_y_values(this->y_values.begin(), this->y_values.begin() + points_count);

    return CartesianTrajectory(new_x_values, new_y_values);
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

  FrenetTrajectory ExtractTrajectory(int points_count=50) {
    vector<double> new_s_values(this->s_values.begin(), this->s_values.begin() + points_count);
    vector<double> new_d_values(this->d_values.begin(), this->d_values.begin() + points_count);

    return FrenetTrajectory(new_s_values, new_d_values);
  }
};


#endif /* TRAJECTORY_H_ */
