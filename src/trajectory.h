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
  double reference_velocity;
  int lane;

  CartesianTrajectory(const vector<double> &x_values,
                      const vector<double> &y_values,
                      double reference_velocity,
                      int lane) {
    this->x_values = x_values;
    this->y_values = y_values;
    this->reference_velocity = reference_velocity;
    this->lane = lane;
  }

  CartesianTrajectory ExtractTrajectory(int points_count=50) {
    vector<double> new_x_values(this->x_values.begin(), this->x_values.begin() + points_count);
    vector<double> new_y_values(this->y_values.begin(), this->y_values.begin() + points_count);

    return CartesianTrajectory(new_x_values, new_y_values, reference_velocity, lane);
  }
};

struct FrenetTrajectory {
  //Frenet coordinates
  vector<double> s_values;
  vector<double> d_values;
  double reference_velocity;
  int lane;

  FrenetTrajectory(const vector<double> &s_values,
                   const vector<double> &d_values,
                   double reference_velocity,
                   int lane) {
    this->s_values = s_values;
    this->d_values = d_values;
    this->reference_velocity = reference_velocity;
    this->lane = lane;
  }

  FrenetTrajectory ExtractTrajectory(int points_count=50) {
    vector<double> new_s_values(this->s_values.begin(), this->s_values.begin() + points_count);
    vector<double> new_d_values(this->d_values.begin(), this->d_values.begin() + points_count);

    return FrenetTrajectory(new_s_values, new_d_values, reference_velocity, lane);
  }
};


#endif /* TRAJECTORY_H_ */
