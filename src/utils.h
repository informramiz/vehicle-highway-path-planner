/*
 * utils.h
 *
 *  Created on: Aug 21, 2017
 *      Author: ramiz
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <iostream>
#include <vector>
#include "Eigen/Dense"
#include "trajectory.h"
#include "vehicle.h"

using namespace std;
using Eigen::VectorXd;

class Utils {
public:
  Utils();
  virtual ~Utils();


  static int find_nearest_vehicle_ahead(const vector<Vehicle> &vehicles, double s, double d);
  static vector<vector<double> > get_trajectory_points(const CartesianTrajectory &trajectory, int points_count);


  /**
   * Method to print a 2D vector. Vector type can be either primitive or
   * a custom type that has overloaded stream insertion operator (<<).
   */
  template<class T>
  static void print_grid(const vector<vector<T> > &grid) {
    for(int i = 0; i < grid.size(); i++)
    {
      cout << grid[i][0];
      for(int j = 1; j < grid[0].size(); j++)
      {
        cout << "," << grid[i][j];
      }
      cout << endl;
    }
  }

  /**
   * Method to print a 1D vector. Vector type can be either primitive or
   * a custom type that has overloaded stream insertion operator (<<).
   */
  template<class T>
  static void print_vector(const vector<T> &values) {
    for (int i = 0; i < values.size(); ++i) {
      cout << values[i];

      if (i != values.size() - 1) {
        cout << ",";
      }
    }

    cout << endl;
  }

  /**
   * Merge two vectors
   */
  template<class T>
  static void merge_vectors(vector<T> &target, vector<T> &source) {
    target.insert(target.end(), source.begin(), source.end());
  }

  /**
   * Calculates Euclidean distance
   */
  static double euclidean(double x1, double y1, double x2, double y2);

  /**
   * Converts angle from degree to radians
   */
  static double deg2rad(double delta_i);
  /**
   * Converts angle from radians to degrees
   */
  static double rad2deg(double x);

  static Eigen::VectorXd vectorToVectorXd(vector<double> &v) {
    Eigen::Map<Eigen::VectorXd> new_vector(v.data(), v.size());
    return new_vector;
  }

  static vector<double> VectorXdToVector(VectorXd &v) {
    return vector<double>(v.data(), v.data() + v.rows() * v.cols());
  }

  /**
   * A function that returns a value between 0 and 1 for x in the
   * range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].
   *
   * Useful for cost functions.
   */
  static double logistic(double x);
};

#endif /* UTILS_H_ */
