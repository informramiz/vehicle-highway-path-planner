/*
 * map_utils.cpp
 *
 *  Created on: Sep 6, 2017
 *      Author: ramiz
 */

#include <fstream>
#include "utils.h"
#include "map_utils.h"

bool MapUtils::is_initialized_ = false;
vector<double> MapUtils::map_waypoints_x_;
vector<double> MapUtils::map_waypoints_y_;
vector<double> MapUtils::map_waypoints_s_;
vector<double> MapUtils::map_waypoints_dx_;
vector<double> MapUtils::map_waypoints_dy_;

void MapUtils::Initialize(const string &map_file) {
  // Load up map values for waypoint's x,y,s and d normalized normal vectors

  ifstream in_map_(map_file.c_str(), ifstream::in);

  if (!in_map_.is_open()) {
    cerr << "Unable to open map file: " << map_file << endl;
    exit(-1);
  }

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x_.push_back(x);
    map_waypoints_y_.push_back(y);
    map_waypoints_s_.push_back(s);
    map_waypoints_dx_.push_back(d_x);
    map_waypoints_dy_.push_back(d_y);
  }

  is_initialized_ = true;
  cout << "map reading complete" << endl;
}

int MapUtils::ClosestWaypoint(double x, double y) {
  CheckInitialization();

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < map_waypoints_x_.size(); i++) {
    double map_x = map_waypoints_x_[i];
    double map_y = map_waypoints_y_[i];
    double dist = Utils::euclidean(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;

}

int MapUtils::NextWaypoint(double x, double y, double theta) {
  CheckInitialization();
  int closestWaypoint = ClosestWaypoint(x, y);

  double map_x = map_waypoints_x_[closestWaypoint];
  double map_y = map_waypoints_y_[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = abs(theta - heading);

  if (angle > M_PI / 4) {
    closestWaypoint++;
  }

  return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> MapUtils::getFrenet(double x, double y, double theta) {
  CheckInitialization();

  int next_wp = NextWaypoint(x, y, theta);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = map_waypoints_x_.size() - 1;
  }

  double n_x = map_waypoints_x_[next_wp] - map_waypoints_x_[prev_wp];
  double n_y = map_waypoints_y_[next_wp] - map_waypoints_y_[prev_wp];
  double x_x = x - map_waypoints_x_[prev_wp];
  double x_y = y - map_waypoints_y_[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = Utils::euclidean(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000 - map_waypoints_x_[prev_wp];
  double center_y = 2000 - map_waypoints_y_[prev_wp];
  double centerToPos = Utils::euclidean(center_x, center_y, x_x, x_y);
  double centerToRef = Utils::euclidean(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += Utils::euclidean(map_waypoints_x_[i], map_waypoints_y_[i],
        map_waypoints_x_[i + 1], map_waypoints_y_[i + 1]);
  }

  frenet_s += Utils::euclidean(0, 0, proj_x, proj_y);

  return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> MapUtils::getXY(double s, double d) {
  CheckInitialization();

  int prev_wp = -1;

  while (s > map_waypoints_s_[prev_wp + 1]
      && (prev_wp < (int) (map_waypoints_s_.size() - 1))) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % map_waypoints_x_.size();

  double heading = atan2((map_waypoints_y_[wp2] - map_waypoints_y_[prev_wp]),
      (map_waypoints_x_[wp2] - map_waypoints_x_[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - map_waypoints_s_[prev_wp]);

  double seg_x = map_waypoints_x_[prev_wp] + seg_s * cos(heading);
  double seg_y = map_waypoints_y_[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - M_PI / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x,y};

}

void MapUtils::CheckInitialization() {
  if (!is_initialized_) {
    cerr << "Map not initialized" << endl;
    exit(-1);
  }
}

void MapUtils::TransformToVehicleCoordinates(double ref_x, double ref_y, double ref_yaw, double &x, double &y) {
  //translate (x, y) to where vehicle is right now (ref_x, ref_y)
  double shift_x = x - ref_x;
  double shift_y = y - ref_y;

  //rotate (x, y) by ref_yaw in clockwise so that vehicle is at angle 0
  double new_x = shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw);
  double new_y = shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw);

  //update passed params (x, y)
  x = new_x;
  y = new_y;
}

void MapUtils::TransformFromVehicleToMapCoordinates(double ref_x, double ref_y, double ref_yaw, double &x, double &y) {
  //rotate (x, y) by ref_yaw in counter-clockwise so that vehicle is at angle ref_yaw
  double rotated_x = x * cos(ref_yaw) - y * sin(ref_yaw);
  double rotated_y = x * sin(ref_yaw) + y * cos(ref_yaw);

  //translate (x, y) to where vehicle is right now (ref_x, ref_y) in map coordinates
  double new_x = rotated_x + ref_x;
  double new_y = rotated_y + ref_y;

  //update passed params (x, y)
  x = new_x;
  y = new_y;
}


