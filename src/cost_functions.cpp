/*
 * cost_functions.cpp
 *
 *  Created on: Aug 25, 2017
 *      Author: ramiz
 */

#include <math.h>
#include "cost_functions.h"
#include "utils.h"
#include "constants.h"

CostFunctions::CostFunctions() {
  // TODO Auto-generated constructor stub

}

CostFunctions::~CostFunctions() {
  // TODO Auto-generated destructor stub
}

double CostFunctions::time_diff_cost(const Trajectory &trajectory,
                                     int target_vehicle_id,
                                     const VectorXd &delta,
                                     double T,
                                     const vector<Vehicle> &predictions) {

  return Utils::logistic(abs(T - trajectory.t) / T);
}

double CostFunctions::d_diff_cost(const Trajectory &trajectory,
                                  int target_vehicle_id,
                                  const VectorXd &delta,
                                  double T,
                                  const vector<Vehicle> &predictions) {

  //as there is separate function for time_diff cost
  //so in this function we will consider time of trajectory
  //as the right time so
  T = trajectory.t;

  VectorXd d_coeffs = trajectory.d_coeffs;
  //calculate value of d-coordinate at time T
  double d_at_t = Utils::solve_polynomial(d_coeffs, T);

  //we also need speed and acceleration values to complete tuple (d, v, a) so
  //make polynomial function for speed which is differentiation of distance (d_coeffs) function
  VectorXd d_dot_coeffs = Utils::differentiate(d_coeffs);
  //calculate speed a time T
  double v_at_t = Utils::solve_polynomial(d_dot_coeffs, T);

  //make polynomial function for acceleration which is differentiation of speed function
  VectorXd d_dot_dot_coeffs = Utils::differentiate(d_dot_coeffs);
  double a_at_t = Utils::solve_polynomial(d_dot_dot_coeffs, T);

  //combine d, v, a to make state of ego vehicle
  VectorXd actual_state(3);
  actual_state << d_at_t, v_at_t, a_at_t;

  //get the target vehicle from predictions using target_vehicle_id
  Vehicle target_vehicle = predictions[target_vehicle_id];
  //predict the state of the target vehicle
  VectorXd target_vehicle_state = target_vehicle.state_at(T);
  //add the required difference in (d, v, a) values that ego vehicle has to keep from target vehicle
  VectorXd expected_state = target_vehicle_state + delta;

  //for each value in ego-vehicle state and target state, penalize if it exceeds allowed variance sigma
  double cost = 0.0;
  for (int i = 0; i < actual_state.rows(); ++i) {
    double diff = abs(actual_state[i] - expected_state[i]);
    cost += Utils::logistic(diff/Constants::SIGMA_D[i]);
  }

  return cost;
}

double CostFunctions::s_diff_cost(const Trajectory &trajectory,
                                  int target_vehicle_id,
                                  const VectorXd &delta,
                                  double T,
                                  const vector<Vehicle> &predictions) {

  //as there is separate function for time_diff cost
  //so in this function we will consider time of trajectory
  //as the right time so
  T = trajectory.t;

  VectorXd s_coeffs = trajectory.s_coeffs;
  //calculate value of s-coordinate at time T
  double s_at_t = Utils::solve_polynomial(s_coeffs, T);

  //we also need speed and acceleration values to complete tuple (s, v, a) so
  //make polynomial function for speed which is differentiation of distance (d_coeffs) function
  VectorXd s_dot_coeffs = Utils::differentiate(s_coeffs);
  //calculate speed a time T
  double v_at_t = Utils::solve_polynomial(s_dot_coeffs, T);

  //make polynomial function for acceleration which is differentiation of speed function
  VectorXd s_dot_dot_coeffs = Utils::differentiate(s_dot_coeffs);
  double a_at_t = Utils::solve_polynomial(s_dot_dot_coeffs, T);

  //combine s, v, a to make state of ego vehicle
  VectorXd actual_state(3);
  actual_state << s_at_t, v_at_t, a_at_t;

  //get the target vehicle from predictions using target_vehicle_id
  Vehicle target_vehicle = predictions[target_vehicle_id];
  //predict the state of the target vehicle
  VectorXd target_vehicle_state = target_vehicle.state_at(T);
  //add the required difference in (s, v, a) values that ego vehicle has to keep from target vehicle
  VectorXd expected_state = target_vehicle_state + delta;

  //for each value in ego-vehicle state and target state, penalize if it exceeds allowed variance/difference sigma
  double cost = 0.0;
  for (int i = 0; i < actual_state.rows(); ++i) {
    double diff = abs(actual_state[i] - expected_state[i]);
    cost += Utils::logistic(diff/Constants::SIGMA_S[i]);
  }

  return cost;
}

double CostFunctions::collision_cost(const Trajectory &trajectory,
                                     int target_vehicle_id,
                                     const VectorXd &delta,
                                     double T,
                                     const vector<Vehicle> &predictions) {
  //calculate nearest approach of trajectory to any vehicle in predictions
  double nearest_approach = Utils::nearest_approach_to_any_vehicle(trajectory, predictions);

  //check if this is less than 2 * VEHICLE_RADIUS (diameter of vehicle)
  //then we consider it a collision
  if (nearest_approach < 2 * Constants::VEHICLE_RADIUS) {
    return 1.0;
  }

  return 0.0;
}

double CostFunctions::buffer_cost(const Trajectory &trajectory,
                                  int target_vehicle_id,
                                  const VectorXd &delta,
                                  double T,
                                  const vector<Vehicle> &predictions) {
  //calculate nearest approach of trajectory to any vehicle in predictions
  double nearest_approach = Utils::nearest_approach_to_any_vehicle(trajectory, predictions);

  return Utils::logistic(2*Constants::VEHICLE_RADIUS / nearest_approach);
}

double CostFunctions::exceeds_speed_limit_cost(const Trajectory &trajectory,
                                               int target_vehicle_id,
                                               const VectorXd &delta,
                                               double T,
                                               const vector<Vehicle> &predictions) {
  //get polynomial function for longitudinal-speed
  //which is differentiation of s-coordinate function
  VectorXd s_dot_coeffs = Utils::differentiate(trajectory.s_coeffs);

  //we will divide total time in 100 steps and check for each timestep in T duration to find
  //out speed (s_dot) for trajectory at each timestep and for given vehicle at that time step
  double max_v = 0;
  for (int i = 0; i < 100; ++i) {
    //consider i% of total time for each iteration
    double t = (T/100) * i;

    //calculate speed at time step t
    double v_at_t = abs(Utils::solve_polynomial(s_dot_coeffs, t));

    //we are only interested in absolute max velocity
    if (v_at_t > max_v) {
      max_v = v_at_t;
    }
  }

  if (max_v > Constants::SPEED_LIMIT) {
    return 1.0;
  }

  return 0.0;
}

double CostFunctions::stays_on_road_cost(const Trajectory &trajectory,
                                         int target_vehicle_id,
                                         const VectorXd &delta,
                                         double T,
                                         const vector<Vehicle> &predictions) {
  //TODO
  return 0.0;
}

/**
 * Rewards high average speeds.
 */
double CostFunctions::efficiency_cost(const Trajectory &trajectory,
                                      int target_vehicle_id,
                                      const VectorXd &delta,
                                      double T,
                                      const vector<Vehicle> &predictions) {
  //calculate avg_speed (v = s/t) of trajectory
  double avg_v = Utils::solve_polynomial(trajectory.s_coeffs, trajectory.t) / trajectory.t;

  //calculate speed of target vehicle
  double target_v = predictions[target_vehicle_id].state_at(trajectory.t)[0] / trajectory.t;

  //multiply by 2 to make cost negative (reward) for trajectories
  //that stay close to target velocity
  return Utils::logistic(2 * (target_v - avg_v) / avg_v);
}

double CostFunctions::max_acceleration_cost(const Trajectory &trajectory,
                             int target_vehicle_id,
                             const VectorXd &delta,
                             double T,
                             const vector<Vehicle> &predictions) {
  //get polynomial function for longitudinal-speed
  //which is differentiation of s-coordinate function
  VectorXd s_dot_coeffs = Utils::differentiate(trajectory.s_coeffs);

  //get polynomial function for longitudinal-acceleration
  //which is differentiation of longitudinal-speed function
  VectorXd s_dot_dot_coeffs = Utils::differentiate(s_dot_coeffs);

  //we will divide total time in 100 steps and check for each timestep in T duration to find
  //out acceleration (s_dot_dot) for trajectory at each timestep and for given vehicle at that time step
  double max_a = 0;
  for (int i = 0; i < 100; ++i) {
    //consider i% of total time for each iteration
    double t = (T/100) * i;

    //calculate acceleration at time step t
    double a_at_t = abs(Utils::solve_polynomial(s_dot_dot_coeffs, t));

    //we are only interested in absolute max acceleration
    if (a_at_t > max_a) {
      max_a = a_at_t;
    }
  }

  if (max_a > Constants::MAX_ACCELERATION) {
    return 1.0;
  }

  return 0.0;
}

/**
 * Binary function for penalizing trajectories that cross max jerk limit
 */
double CostFunctions::max_jerk_cost(const Trajectory &trajectory,
                     int target_vehicle_id,
                     const VectorXd &delta,
                     double T,
                     const vector<Vehicle> &predictions) {
  //get polynomial function for longitudinal-speed
  //which is differentiation of s-coordinate function
  VectorXd s_dot_coeffs = Utils::differentiate(trajectory.s_coeffs);

  //get polynomial function for longitudinal-acceleration
  //which is differentiation of longitudinal-speed function
  VectorXd s_dot_dot_coeffs = Utils::differentiate(s_dot_coeffs);

  //get polynomial function for longitudinal-jerk
  //which is differentiation of longitudinal-acceleration function
  VectorXd s_dot_dot_dot_coeffs = Utils::differentiate(s_dot_dot_coeffs);

  //we will divide total time in 100 steps and check for each timestep in T duration to find
  //out jerk (s_dot_dot_dot) for trajectory at each timestep and for given vehicle at that time step
  double max_jerk = 0;
  for (int i = 0; i < 100; ++i) {
    //consider i% of total time for each iteration
    double t = (T/100) * i;

    //calculate jerk at time step t
    double jerk_at_t = abs(Utils::solve_polynomial(s_dot_dot_dot_coeffs, t));

    //we are only interested in absolute max jerk
    if (jerk_at_t > max_jerk) {
      max_jerk = jerk_at_t;
    }
  }

  if (max_jerk > Constants::MAX_JERK) {
    return 1.0;
  }

  return 0.0;
}

double CostFunctions::total_acceleration_cost(const Trajectory &trajectory,
                                              int target_vehicle_id,
                                              const VectorXd &delta,
                                              double T,
                                              const vector<Vehicle> &predictions) {
  //get polynomial function for longitudinal-speed
  //which is differentiation of s-coordinate function
  VectorXd s_dot_coeffs = Utils::differentiate(trajectory.s_coeffs);

  //get polynomial function for longitudinal-acceleration
  //which is differentiation of longitudinal-speed function
  VectorXd s_dot_dot_coeffs = Utils::differentiate(s_dot_coeffs);

  //we will divide total time in 100 steps and check for each timestep in T duration to find
  //out acceleration (s_dot_dot) for trajectory at each timestep and for given vehicle at that time step
  double total_a = 0;
  double dt = (T/100);
  for (int i = 0; i < 100; ++i) {
    //consider i% of total time for each iteration
    double t = dt * i;

    //calculate acceleration at time step t
    double a = abs(Utils::solve_polynomial(s_dot_dot_coeffs, t));
    //TODO: What does (a * dt) means here? I think its converting
    //acceleration into velocity so that later acceleration can be calculated
    a = a * dt;

    total_a += a;
  }

  //calculate acceleration per second
  double acceleration_in_one_second = total_a / T;

  return Utils::logistic(acceleration_in_one_second / Constants::EXPECTED_ACCELERATION_IN_ONE_SEC);
}

double CostFunctions::total_jerk_cost(const Trajectory &trajectory,
                                      int target_vehicle_id,
                                      const VectorXd &delta,
                                      double T,
                                      const vector<Vehicle> &predictions) {
  //get polynomial function for longitudinal-speed
  //which is differentiation of s-coordinate function
  VectorXd s_dot_coeffs = Utils::differentiate(trajectory.s_coeffs);

  //get polynomial function for longitudinal-acceleration
  //which is differentiation of longitudinal-speed function
  VectorXd s_dot_dot_coeffs = Utils::differentiate(s_dot_coeffs);

  //get polynomial function for longitudinal-jerk
  //which is differentiation of longitudinal-acceleration function
  VectorXd s_dot_dot_dot_coeffs = Utils::differentiate(s_dot_dot_coeffs);

  //we will divide total time in 100 steps and check for each timestep in T duration to find
  //out jerk (s_dot_dot_dot) for trajectory at each timestep and for given vehicle at that time step
  double total_jerk = 0;
  double dt = (T/100);
  for (int i = 0; i < 100; ++i) {
    //consider i% of total time for each iteration
    double t = dt * i;

    //calculate jerk at time step t
    double jerk = abs(Utils::solve_polynomial(s_dot_dot_dot_coeffs, t));
    //TODO: What does (jerk * dt) means here? I think its converting
    //jerk into acceleration so that later jerk can be calculated by
    //dividing acceleration by time T
    jerk = jerk * dt;

    total_jerk += jerk;
  }

  //calculate jerk per second
  double jerk_in_one_second = total_jerk / T;

  return Utils::logistic(jerk_in_one_second / Constants::EXPECTED_JERK_IN_ONE_SEC);
}

double CostFunctions::calculate_cost(const Trajectory &trajectory,
                                     int target_vehicle_id,
                                     const VectorXd &delta,
                                     double T,
                                     const vector<Vehicle> &predictions,
                                     bool verbose) {
  double total_cost = 0.0;
  for (int i = 0; i < cost_functions_pointers.size(); ++i) {
    double cost = (this->*cost_functions_pointers[i])(trajectory, target_vehicle_id, delta, T, predictions);
    cost *= cost_functions_weights[i];

    if (verbose) {
      printf("Cost for function %s is %f\n", cost_functions_names[i].c_str(), cost) ;
    }

    total_cost += cost;
  }

  return total_cost;
}
