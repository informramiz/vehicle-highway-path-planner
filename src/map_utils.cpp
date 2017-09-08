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

FrenetTrajectory MapUtils::CartesianToFrenet(const CartesianTrajectory &cartesian_trajectory,
                                                    const double ref_yaw) {

  vector<double> s_values;
  vector<double> d_values;

  //get first point as it will be from previous path
  //we will use it for angle calculation which is
  //tangent line (slope) between two points
  double prev_x = cartesian_trajectory.x_values[0];
  double prev_y = cartesian_trajectory.y_values[0];

  //convert this point to Frenet
  vector<double> frenet = getFrenet(prev_x, prev_y, ref_yaw);
  s_values.push_back(frenet[0]);
  d_values.push_back(frenet[1]);

  //for each point in trajectory predict where other vehicle
  //will be at that point in time to see if there is a collision
  const int num_timesteps = cartesian_trajectory.x_values.size();
  for (int i = 1; i < num_timesteps; ++i) {
    double next_x = cartesian_trajectory.x_values[i];
    double next_y = cartesian_trajectory.y_values[i];

    //calculate angle of vehicle at this point in time
    //which is slope (tangent) between this and previous point
    double yaw = atan2(next_y - prev_y, next_x - prev_x);
    //convert current trajectory point to Frenet coordinate system
    vector<double> sd = MapUtils::getFrenet(next_x, next_y, yaw);

    s_values.push_back(sd[0]);
    d_values.push_back(sd[1]);

    prev_x = next_x;
    prev_y = next_y;
  }

  return FrenetTrajectory(s_values, d_values);
}

CartesianTrajectory MapUtils::FrenetToCartesian(const FrenetTrajectory &frenet_trajectory) {
  vector<double> x_values;
  vector<double> y_values;

  const int points_count = frenet_trajectory.s_values.size();
  for (int i = 0; i < points_count; ++i) {
    vector<double> xy = getXY(frenet_trajectory.s_values[i], frenet_trajectory.d_values[i]);

    x_values.push_back(xy[0]);
    y_values.push_back(xy[1]);
  }

  return CartesianTrajectory(x_values, y_values);
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

int MapUtils::GetLane(double d) {
  if (d >= 0 && d <= 4) {
    return 0;
  } else if (d > 4 && d <= 8) {
    return 1;
  } else if (d > 8 && d <= 12) {
    return 2;
  }

  return -1;
}

double MapUtils::GetdValueForLaneCenter(int lane) {
  //as car drives in center so add 2m for to count current lane
  //and as each lane is 4m so multiply by 4 to count other lanes before on left
  return 2 + lane * 4;
}

