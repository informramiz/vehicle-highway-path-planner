/*
 * path_planner.cpp
 *
 *  Created on: Sep 7, 2017
 *      Author: ramiz
 */

#include "map_utils.h"
#include "path_planner.h"

// Sensor Fusion Data, a list of all other cars on the same side of the road.
//The data format for each car is: [ id, x, y, vx, vy, s, d]
PathPlanner::PathPlanner() {
  this->lane_ = 1;
  this->reference_velocity_ = 0.0;
}

//Sensor Fusion Data, a list of all other cars on the same side of the road.
//The data format for each car is: [ id, x, y, vx, vy, s, d]'
vector<Vehicle> PathPlanner::ExtractSensorFusionData(const vector<vector<double> > &sensor_fusion_data,
                                                     const int previous_path_size) {

  vector<Vehicle> vehicles;

  for (int i = 0; i < sensor_fusion_data.size(); ++i) {
    int id = sensor_fusion_data[i][0];
    double x = sensor_fusion_data[i][1];
    double y = sensor_fusion_data[i][2];

    //access vx, vy which are at indexes (3, 4)
    double vx = sensor_fusion_data[i][3];
    double vy = sensor_fusion_data[i][4];

    //s-coordinate is at index 5
    double s = sensor_fusion_data[i][5];
    //d-coordinate is at index 6
    double d = sensor_fusion_data[i][6];

    //calculate total velocity
    double total_v = sqrt(vx*vx + vy*vy);

    //if previous_path size is not 0 that means
    //the Simulator has not yet traversed complete prev path and
    //the vehicle is not in perspective with the
    //new path/trajectory we are going to build so
    //we should predict its s-coordinate like if
    //we previous path was already traversed
    //REMEMBER: 1 timestep = 0.02 secs (or 20ms)
    double predicted_s = s + 0.02 * total_v * previous_path_size;

    Vehicle v(id, x, y, predicted_s, d, 0, total_v, 0);
    vehicles.push_back(v);
  }

  return vehicles;
}

void PathPlanner::UpdateEgoVehicleStateWithRespectToPreviousPath(const vector<double> &previous_path_x,
                                                                 const vector<double> &previous_path_y,
                                                                 const double previous_path_last_s,
                                                                 const double previous_path_last_d) {
  //previous path is not empty that means simulator has
  //not traversed it yet and car is still in somewhere on that path
  //we have to provide new path so start from previous path end
  //hence previous path end point will become the new reference point to start
  int prev_path_size = previous_path_x.size();

  if (prev_path_size > 2) {
    ego_vehicle_.s = previous_path_last_s;
    ego_vehicle_.d = previous_path_last_d;

    ego_vehicle_.x = previous_path_x[prev_path_size - 1];
    ego_vehicle_.y = previous_path_y[prev_path_size - 1];

    //we need to calculate ref_yaw because provided ego_vehicle_.yaw is of
    //where the vehicle is right now (which is somewhere before previous_path end)
    //so ego_vehicle_.yaw does not represent angle that will be when ego vehicle arrives
    //at the end of previous path
    //Angle is tangent between last and second
    //last point of the previous path so get the second last point
    double prev_x = previous_path_x[prev_path_size - 2];
    double prev_y = previous_path_y[prev_path_size - 2];

    //calculate tangent
    ego_vehicle_.yaw = atan2(ego_vehicle_.y - prev_y, ego_vehicle_.x - prev_x);
  }
}

PathPlanner::~PathPlanner() {
  // TODO Auto-generated destructor stub
}

double PathPlanner::FindDistanceFromVehicleAhead() {
//  int ego_vehicle_lane = MapUtils::GetLane(ego_vehicle_.d);
  double min_distance = 999999;

  for (int i = 0; i < vehicles_.size(); ++i) {
    int vehicle_lane = vehicles_[i].lane;
    double s = vehicles_[i].s;
    //discard this iteration if vehicle is not in ego_vehicle's lane
    //and is not leading vehicle

    if (vehicle_lane != this->lane_
        || s <= ego_vehicle_.s) {
      //we are only interested in leading vehicles in same lane
      continue;
    }

    //vehicle is leading vehicle and in same lane as ego vehicle
    //check distance to leading vehicle
    double distance = s - ego_vehicle_.s;
    if (distance < min_distance) {
      min_distance = distance;
    }
  }

  return min_distance;
}

bool PathPlanner::IsTooCloseToVehicleAhead() {
  return FindDistanceFromVehicleAhead() < BUFFER_DISTANCE;
}

Trajectory PathPlanner::GenerateTrajectory(const Vehicle &ego_vehicle,
                                           const vector<vector<double> > &sensor_fusion_data,
                                           const vector<double> &previous_path_x,
                                           const vector<double> &previous_path_y,
                                           const double previous_path_last_s,
                                           const double previous_path_last_d) {
  this->ego_vehicle_ = ego_vehicle;
  this->vehicles_ = ExtractSensorFusionData(sensor_fusion_data, previous_path_x.size());

  //we need to consider whether Simulator has traversed previous path
  //completely or some points till left. This will affect ego vehicle
  //state as well as new path so let's update ego vehicle state accordingly
  UpdateEgoVehicleStateWithRespectToPreviousPath(previous_path_x, previous_path_y,
                                                 previous_path_last_s, previous_path_last_d);

  bool is_too_close_to_leading_vehicle = IsTooCloseToVehicleAhead();

  if (is_too_close_to_leading_vehicle) {
    //decrease speed to avoid collision
    this->reference_velocity_ -= 0.224;

    //also consider changing lanes
    this->lane_ = (this->lane_ + 1) % 3;
  } else if (reference_velocity_ < SPEED_LIMIT) {
    //we have less than desired speed and
    //are more than buffer distance from leading vehicle
    //so that means we can increase speed
    this->reference_velocity_ += 0.224;
  }

  TrajectoryGenerator trajectory_generator;
  return trajectory_generator.GenerateTrajectory(ego_vehicle_, previous_path_x, previous_path_y, previous_path_last_s,
      previous_path_last_d, lane_, reference_velocity_);
}



