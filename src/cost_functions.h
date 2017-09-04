/*
 * cost_functions.h
 *
 *  Created on: Aug 25, 2017
 *      Author: ramiz
 */

#ifndef COST_FUNCTIONS_H_
#define COST_FUNCTIONS_H_

#include <vector>
#include "trajectory.h"
#include "vehicle.h"

using namespace std;

class CostFunctions {
public:
  CostFunctions();
  virtual ~CostFunctions();

  /**
   * Penalizes trajectories that span a duration which is longer or
   * shorter than the duration requested.
   *
   * @param delta,
   * delta - a length 6 array indicating the offset we are aiming for between us
   *    and the target_vehicle. So if at time 5 the target vehicle will be at
   *   [100, 10, 0, 0, 0, 0] and delta is [-10, 0, 0, 4, 0, 0], then our goal
   *   state for t = 5 will be [90, 10, 0, 4, 0, 0]. This would correspond to a
   *   goal of "follow 10 meters behind and 4 meters to the right of target vehicle"
   *
   */
  double time_diff_cost(const Trajectory &trajectory,
                        int target_vehicle_id,
                        const VectorXd &delta,
                        double T,
                        const vector<Vehicle> &predictions);


  /**
   * Penalizes trajectories whose d coordinate (and derivatives)
   * differ from the goal.
   *
   * @param delta,
   * delta - a length 6 array indicating the offset we are aiming for between us
   *    and the target_vehicle. So if at time 5 the target vehicle will be at
   *   [100, 10, 0, 0, 0, 0] and delta is [-10, 0, 0, 4, 0, 0], then our goal
   *   state for t = 5 will be [90, 10, 0, 4, 0, 0]. This would correspond to a
   *   goal of "follow 10 meters behind and 4 meters to the right of target vehicle"
   */
  double d_diff_cost(const Trajectory &trajectory,
                     int target_vehicle_id,
                     const VectorXd &delta,
                     double T,
                     const vector<Vehicle> &predictions);

  /**
   * Penalizes trajectories whose s coordinate (and derivatives)
   * differ from the goal.
   *
   * @param delta,
   * delta - a length 6 array indicating the offset we are aiming for between us
   *    and the target_vehicle. So if at time 5 the target vehicle will be at
   *   [100, 10, 0, 0, 0, 0] and delta is [-10, 0, 0, 4, 0, 0], then our goal
   *   state for t = 5 will be [90, 10, 0, 4, 0, 0]. This would correspond to a
   *   goal of "follow 10 meters behind and 4 meters to the right of target vehicle"
   */
  double s_diff_cost(const Trajectory &trajectory,
                     int target_vehicle_id,
                     const VectorXd &delta,
                     double T,
                     const vector<Vehicle> &predictions);

  /**
   * Binary cost function which penalizes collisions.
   */
  double collision_cost(const Trajectory &trajectory,
                     int target_vehicle_id,
                     const VectorXd &delta,
                     double T,
                     const vector<Vehicle> &predictions);

  /**
   * Penalizes getting close to other vehicles.
   */
  double buffer_cost(const Trajectory &trajectory,
                     int target_vehicle_id,
                     const VectorXd &delta,
                     double T,
                     const vector<Vehicle> &predictions);

  /**
   * Penalizes getting close to other vehicles.
   */
  double exceeds_speed_limit_cost(const Trajectory &trajectory,
                                  int target_vehicle_id,
                                  const VectorXd &delta,
                                  double T,
                                  const vector<Vehicle> &predictions);

  /**
   * Penalizes getting out of road
   */
  double stays_on_road_cost(const Trajectory &trajectory,
                                  int target_vehicle_id,
                                  const VectorXd &delta,
                                  double T,
                                  const vector<Vehicle> &predictions);

  /**
   * Rewards high average speeds.
   */
  double efficiency_cost(const Trajectory &trajectory,
                                  int target_vehicle_id,
                                  const VectorXd &delta,
                                  double T,
                                  const vector<Vehicle> &predictions);

  /**
   * Binary function for penalizing trajectories that cross max acceleration limit
   */
  double max_acceleration_cost(const Trajectory &trajectory,
                               int target_vehicle_id,
                               const VectorXd &delta,
                               double T,
                               const vector<Vehicle> &predictions);

  /**
   * Binary function for penalizing trajectories that cross max jerk limit
   */
  double max_jerk_cost(const Trajectory &trajectory,
                       int target_vehicle_id,
                       const VectorXd &delta,
                       double T,
                       const vector<Vehicle> &predictions);

  /**
   * Binary function for penalizing trajectories that cross acceleration in one second limit
   */
  double total_acceleration_cost(const Trajectory &trajectory,
                               int target_vehicle_id,
                               const VectorXd &delta,
                               double T,
                               const vector<Vehicle> &predictions);

  /**
   * Binary function for penalizing trajectories that cross jerk in one second limit
   */
  double total_jerk_cost(const Trajectory &trajectory,
                         int target_vehicle_id,
                         const VectorXd &delta,
                         double T,
                         const vector<Vehicle> &predictions);

  /**
   * Calculates cost for given trajectory and based on given predictions
   */
  double calculate_cost(const Trajectory &trajectory,
                        int target_vehicle_id,
                        const VectorXd &delta,
                        double T,
                        const vector<Vehicle> &predictions,
                        bool verbose = false);

  //***************Data members*******************//

  ///define an alias type name for function pointer
  typedef double (CostFunctions::* cost_function_ptr) (const Trajectory &trajectory,
                                                      int target_vehicle_id,
                                                      const VectorXd &delta,
                                                      double T,
                                                      const vector<Vehicle> &predictions);

  const vector<cost_function_ptr> cost_functions_pointers = {
      &CostFunctions::time_diff_cost,
      &CostFunctions::s_diff_cost,
      &CostFunctions::d_diff_cost,
      &CostFunctions::collision_cost,
      &CostFunctions::efficiency_cost,
      &CostFunctions::exceeds_speed_limit_cost,
      &CostFunctions::stays_on_road_cost,
      &CostFunctions::buffer_cost,
      &CostFunctions::max_acceleration_cost,
      &CostFunctions::max_jerk_cost,
      &CostFunctions::total_acceleration_cost,
      &CostFunctions::total_jerk_cost,
  };

  const vector<double> cost_functions_weights = {
      5, //time_diff_cost
      15, //s_diff_cost
      15, //d_diff_cost
      20, //collision_cost
      1,  //efficiency_cost
      10, //exceeds_speed_limit_cost
      1,  //stays_on_road_cost
      15, //buffer_cost
      1, //max_acceleration_cost
      5, //max_jerk_cost
      1, //total_acceleration_cost
      5, //total_jerk_cost
  };

  const vector<string> cost_functions_names = {
        "time_diff_cost",
        "s_diff_cost",
        "d_diff_cost",
        "collision_cost",
        "efficiency_cost",
        "exceeds_speed_limit_cost",
        "stays_on_road_cost",
        "buffer_cost",
        "max_acceleration_cost",
        "max_jerk_cost",
        "total_acceleration_cost",
        "total_jerk_cost",
    };
};

#endif /* COST_FUNCTIONS_H_ */
