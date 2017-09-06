/*
 * map_utils.h
 *
 *  Created on: Sep 6, 2017
 *      Author: ramiz
 */

#ifndef MAP_UTILS_H_
#define MAP_UTILS_H_

#include <iostream>
#include <vector>
#include <string>

using namespace std;

class MapUtils {
public:
  static void Initialize(const string &map_file);
  static int ClosestWaypoint(double x, double y);
  static int NextWaypoint(double x, double y, double theta);
  static vector<double> getFrenet(double x, double y, double theta);
  static vector<double> getXY(double s, double d);

  static void TransformToVehicleCoordinates(double ref_x, double ref_y, double ref_yaw, double &x, double &y);
  static void TransformFromVehicleToMapCoordinates(double ref_x, double ref_y, double ref_yaw, double &x, double &y);
private:
  static void CheckInitialization();

  static bool is_initialized_;
  static vector<double> map_waypoints_x_;
  static vector<double> map_waypoints_y_;
  static vector<double> map_waypoints_s_;
  static vector<double> map_waypoints_dx_;
  static vector<double> map_waypoints_dy_;
};

#endif /* MAP_UTILS_H_ */
