/*
 * utils.cpp
 *
 *  Created on: Aug 21, 2017
 *      Author: ramiz
 */

#include <math.h>
//#include "matplotlibcpp.h"
#include "utils.h"

//namespace plt = matplotlibcpp;

Utils::Utils() {
  // TODO Auto-generated constructor stub

}

Utils::~Utils() {
  // TODO Auto-generated destructor stub
}

double Utils::euclidean(double x1, double y1, double x2, double y2) {
  double dist_x = (x1 - x2);
  double dist_y =  (y1 - y2);
  double squared_dist = pow(dist_x, 2) + pow(dist_y, 2);

  return sqrt(squared_dist);
}

double Utils::deg2rad(double delta_i) {
  return M_PI / 180.0 * delta_i;
}

double Utils::rad2deg(double x) {
  return x * 180 / M_PI;
}

double Utils::logistic(double x) {
  return (2.0 / (1 + exp(-x))) - 1.0;
}

int Utils::get_lane(double d) {
  if (d >= 0 && d <= 4) {
    return 0;
  } else if (d > 4 && d <= 8) {
    return 1;
  } else if (d > 8 && d <= 12) {
    return 2;
  }

  return -1;
}

double Utils::get_d_value_for_lane_center(int lane) {
  //as car drives in center so add 2m for to count current lane
  //and as each lane is 4m so multiply by 4 to count other lanes before on left
  return 2 + lane * 4;
}




