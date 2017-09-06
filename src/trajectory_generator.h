/*
 * trajectory_generator.h
 *
 *  Created on: Sep 6, 2017
 *      Author: ramiz
 */

#ifndef TRAJECTORY_GENERATOR_H_
#define TRAJECTORY_GENERATOR_H_

#include <vector>
#include "vehicle.h"
#include "trajectory.h"

using namespace std;

class TrajectoryGenerator {
public:
  TrajectoryGenerator();
  virtual ~TrajectoryGenerator();

  static Trajectory GenerateTrajectory(const Vehicle &ego_vehicle,
                                       const vector<double> &prev_path_x,
                                       const vector<double> &prev_path_y,
                                       double prev_path_last_s,
                                       double prev_path_last_d,
                                       int proposed_lane,
                                       double ref_velocity);
};

#endif /* TRAJECTORY_GENERATOR_H_ */
