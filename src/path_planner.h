/*
 * path_planner.h
 *
 *  Created on: Sep 7, 2017
 *      Author: ramiz
 */

#ifndef PATH_PLANNER_H_
#define PATH_PLANNER_H_

#include <iostream>
#include <vector>
#include "vehicle.h"
#include "trajectory_generator.h"
#include "trajectory.h"
#include "cost_functions.h"

using namespace std;

class PathPlanner {
public:
  PathPlanner();

  virtual ~PathPlanner();

  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  //The data format for each car is: [ id, x, y, vx, vy, s, d]
  Trajectory GenerateTrajectory(const Vehicle &ego_vehicle,
                                const vector<vector<double> > &sensor_fusion_data,
                                const vector<double> &previous_path_x,
                                const vector<double> &previous_path_y,
                                const double previous_path_last_s,
                                const double previous_path_last_d);

private:
  vector<Vehicle> ExtractSensorFusionData(const vector<vector<double> > &sensor_fusion_data, const int previous_path_size);
  void UpdateEgoVehicleStateWithRespectToPreviousPath();
  Trajectory FindBestTrajectory();

  double FindDistanceFromVehicleAhead();
  bool IsTooCloseToVehicleAhead();
  vector<Trajectory> GeneratePossibleTrajectories(const vector<int> &valid_lanes);
  std::__1::vector<int> GetPossibleLanesToGo();

  TrajectoryGenerator trajectory_generator_;
  CostFunctions cost_functions_;

  vector<Vehicle> vehicles_;
  Vehicle ego_vehicle_;
  vector<double> previous_path_x_;
  vector<double> previous_path_y_;
  double previous_path_last_s_;
  double previous_path_last_d_;

  int lane_;
  double reference_velocity_;

  const vector<int> possible_lanes_ = {0, 1, 2};
  const double BUFFER_DISTANCE = 30;
  const double SPEED_LIMIT = 49.5;
  // The max s value before wrapping around the track back to 0
  const double MAX_S = 6945.554;
};

#endif /* PATH_PLANNER_H_ */
