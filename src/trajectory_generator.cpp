/*
 * trajectory_generator.cpp
 *
 *  Created on: Sep 6, 2017
 *      Author: ramiz
 */

#include <math.h>
#include "utils.h"
#include "map_utils.h"
#include "spline.h"
#include "trajectory_generator.h"

TrajectoryGenerator::TrajectoryGenerator() {
  // TODO Auto-generated constructor stub

}

TrajectoryGenerator::~TrajectoryGenerator() {
  // TODO Auto-generated destructor stub
}

CartesianTrajectory TrajectoryGenerator::GenerateTrajectory(const Vehicle &ego_vehicle,
                                     const vector<double> &prev_path_x,
                                     const vector<double> &prev_path_y,
                                     double prev_path_last_s,
                                     double prev_path_last_d,
                                     int proposed_lane,
                                     double ref_velocity) {

  const int prev_path_size = prev_path_x.size();

  double ref_x = ego_vehicle.x;
  double ref_y = ego_vehicle.y;
  double ref_yaw = ego_vehicle.yaw;

  double ref_s = ego_vehicle.s;
  double ref_d = ego_vehicle.d;

  //previous path is not empty that means simulator has
  //not traversed it yet and car is still in somewhere on that path
  //we have to provide new path so start from previous path end
  //hence previous path end point will become the new reference point to start
  if (prev_path_size > 0) {
    ref_s = prev_path_last_s;
    ref_d = prev_path_last_d;
  }

  //make vectors of temporary points first
  //from which we will extrapolate actual points
  vector<double> points_x;
  vector<double> points_y;

  if (prev_path_size < 2) {
    //predict x,y before ref_x, ref_y
    //as delta_t = 1 so
    double prev_x = ref_x - cos(ref_yaw);
    double prev_y = ref_y - sin(ref_yaw);

    //add (ref_x, ref_y) and (prev_x, prev_y) to points list
    points_x.push_back(prev_x);
    points_y.push_back(prev_y);

    points_x.push_back(ref_x);
    points_y.push_back(ref_y);
  } else {
    //previous path is not empty that means simulator has
    //not traversed it yet and car is still in somewhere on that path
    //we have to provide new path so start from previous path end
    //hence previous path end point will become the new reference point to start
    ref_x = prev_path_x[prev_path_size-1];
    ref_y = prev_path_y[prev_path_size-1];

    //we need to calculate ref_yaw which is tangent between last and second
    //last point of the previous path so get the second last point
    double x_before_ref_x = prev_path_x[prev_path_size-2];
    double y_before_ref_y = prev_path_y[prev_path_size-2];

    //now take tangent
    ref_yaw = atan2(ref_y - y_before_ref_y, ref_x - x_before_ref_x);

    //add these 2 points to list of points as well
    points_x.push_back(x_before_ref_x);
    points_y.push_back(y_before_ref_y);

    points_x.push_back(ref_x);
    points_y.push_back(ref_y);
  }

  //add 3 more equally distant (30 meters) points (from each other) for better extrapolation
  //AND to also consider LANE CHANGE
  //for ease we will add them as Frenet coordinates
  double d_value_for_proposed_lane = MapUtils::GetdValueForLaneCenter(proposed_lane);
  vector<double> wp0 = MapUtils::getXY(ref_s + 30, d_value_for_proposed_lane);
  vector<double> wp1 = MapUtils::getXY(ref_s + 60, d_value_for_proposed_lane);
  vector<double> wp2 = MapUtils::getXY(ref_s + 90, d_value_for_proposed_lane);

  //add these 3 points to way points list
  points_x.push_back(wp0[0]);
  points_y.push_back(wp0[1]);

  points_x.push_back(wp1[0]);
  points_y.push_back(wp1[1]);

  points_x.push_back(wp2[0]);
  points_y.push_back(wp2[1]);

  //to make our math easier let's convert these points from
  //Cartesian/Map coordinates to Vehicle coordinates
  //for example, when x-axis is verticle (high slope roads)
  //you will get same y-values for different x-axis.
  //To avoid that we convert to vehicle coordinates which
  //don't have these issues
  for (int i = 0; i < points_x.size(); ++i) {
    MapUtils::TransformToVehicleCoordinates(ref_x, ref_y, ref_yaw, points_x[i], points_y[i]);
  }

  //fit a spline function which makes sure the curve/line passes through
  //each given point
  tk::spline spline;
  spline.set_points(points_x, points_y);

  //our reference velocity is in miles/hour we need to
  //convert our desired/reference velocity in meters/second for ease
  //Remember 1 mile = 1.69 km = 1609.34 meters
  //and 1 hours = 60 mins * 60 secs = 3600
  const double ref_velocity_meters_per_second = ref_velocity * (1609.34 / 3600);

  //as we need to find the space between points on spline to
  //to keep our desired velocity, to achieve that
  //we can define some target on x-axis, target_x,
  //and then find how many points should be there in between
  //x-axis=0 and x-axis=target_x so that if we keep our desired
  //velocity and each time step is 0.02 (20 ms) long then we achieve
  //our target distance between 0 to target_x, target_dist
  //
  //formula: V = d / t
  //as each timestep = 0.02 secs so
  //formula: V = d / (0.02 * N)
  //--> ref_v = target_dist / (0.02 * N)
  //--> N = target_dist / (0.02 * ref_v)
  double target_x = 30;
  double target_y = spline(target_x);
  double target_distance = Utils::euclidean(0, 0, target_x, target_y);

  //here N is: number of points from 0 to target_dist_x
  //
  //--> point_space = target_x / N
  const double N = target_distance / (0.02 * ref_velocity_meters_per_second);

  //divide the range between 0 to target_x to get
  //the space each point in this range have in between
  //to reach target_x with desired velocity
  const double point_space = target_x / N;

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  //now we are ready to generate points from spline
  //but first let's add points of previous path that are
  //not yet traversed by the simulator
  for (int i = 0; i < prev_path_size; ++i) {
    next_x_vals.push_back(prev_path_x[i]);
    next_y_vals.push_back(prev_path_y[i]);
  }

  //now generate remaining points
  //in total we need 50 points (50 frames (each frame of 20ms) per second
  //and in each (after 20ms) simulator moves to next point in list)
  //so providing 50 points will cover 1 second window.
  //As we have already added left-over points from previous path that
  //simulator have not yet traversed so we will only add (50 - prev_path_size)
  //points. All points will be generated starting from ref_x, which is 0
  //because we are in vehicle coordinates so (ref_x, ref_y) = (0, 0),
  //and will have `point_space` gap between them
  double start_x = 0;
  for (int i = 1; i <= 50 - prev_path_size; ++i) {
    double x = start_x + (i * point_space);
    //get corresponding y-value on spline
    double y = spline(x);

    //convert each point back to Map-Coordinates as
    //Simulator expects points in Map-Coordinates
    MapUtils::TransformFromVehicleToMapCoordinates(ref_x, ref_y, ref_yaw, x, y);

    //add this point to the list of points
    next_x_vals.push_back(x);
    next_y_vals.push_back(y);
  }

  return CartesianTrajectory(next_x_vals, next_y_vals);
}

