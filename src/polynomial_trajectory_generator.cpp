/*
 * jmt.cpp
 *
 *  Created on: Aug 26, 2017
 *      Author: ramiz
 */

#include <iostream>
#include <random>
#include <math.h>
#include "polynomial_trajectory_generator.h"
#include "constants.h"
#include "cost_functions.h"
#include "utils.h"

PolynomialTrajectoryGenerator::PolynomialTrajectoryGenerator() {
  // TODO Auto-generated constructor stub

}

PolynomialTrajectoryGenerator::~PolynomialTrajectoryGenerator() {
  // TODO Auto-generated destructor stub
}

Trajectory PolynomialTrajectoryGenerator::generate_trajectory(const VectorXd &start_s,
                                 const VectorXd &start_d,
                                 int target_vehicle_id,
                                 const VectorXd &delta,
                                 double T,
                                 const vector<Vehicle> &predictions) {
  Vehicle target_vehicle = predictions[target_vehicle_id];
  double timestep = 0.5;

  vector<Goal> all_goals;
  //account for noise in duration
  double t = T - Constants::SIGMA_T * timestep;
  while (t < T + Constants::SIGMA_T * timestep) {
    //predict the sate of target vehicle at timstep t
    //and add offset delta to target vehicle state
    //which is the sate we want to achieve for ego vehicle
    VectorXd target_state = target_vehicle.state_at(t) + delta;
    Goal goal(target_state.head(3), target_state.tail(3), t);

    //add this goal to list of goals
    all_goals.push_back(goal);
    vector<Goal> perturbed_goals = generate_perturbed_goals(goal);

    //add these perturbed goals to list of goals
    Utils::merge_vectors(all_goals, perturbed_goals);

    t += timestep;
  }

  vector<Trajectory> trajectories = generate_trajectories(start_s, start_d, all_goals);
  Trajectory best = find_best_trajectory(trajectories, target_vehicle_id, delta, T, predictions);

  return best;
}

Trajectory PolynomialTrajectoryGenerator::generate_trajectory(const VectorXd &start_s,
                                 const VectorXd &start_d,
                                 const Goal &goal,
                                 double T,
                                 const vector<Vehicle> &predictions) {
  double timestep = 0.5;

  vector<Goal> all_goals;
  vector<Goal> perturbed_goals = generate_perturbed_goals(goal);

  vector<Trajectory> trajectories = generate_trajectories(start_s, start_d, all_goals);
//  Trajectory best = find_best_trajectory(trajectories, target_vehicle_id, delta, T, predictions);
  Trajectory best = find_best_trajectory(trajectories, 0, start_d, T, predictions);

  return best;
}



/**
 * @Input: ([0, 10, 0], [10, 10, 0], 1)
 * @output: [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
 */
VectorXd PolynomialTrajectoryGenerator::generate_jerk_minimized_trajectory(VectorXd start, VectorXd end, double T) {

  /*
   Calculate the Jerk Minimizing Trajectory that connects the initial state
   to the final state in time T.

   INPUTS

   start - the vehicles start location given as a length three array
   corresponding to initial values of [s, s_dot, s_double_dot]

   end   - the desired end state for vehicle. Like "start" this is a
   length three array.

   T     - The duration, in seconds, over which this maneuver should occur.

   OUTPUT
   an array of length 6, each value corresponding to a coefficent in the polynomial
   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

   EXAMPLE

   > JMT( [0, 10, 0], [10, 10, 0], 1)
   [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */

  double Si = start[0];
  double Si_dot = start[1];
  double Si_dot_dot = start[2];

  double Sf = end[0];
  double Sf_dot = end[1];
  double Sf_dot_dot = end[2];

  //a0 = Si
  double a0 = Si;
  //a1 = Si_dot
  double a1 = Si_dot;
  //a2 = Si_dot_dot/2
  double a2 = Si_dot_dot / 2;

  //C1 = Si + Si_dot*T + Si_dot_dot/2 * T^2
  double C1 = Si + Si_dot * T + (Si_dot_dot/2) * pow(T, 2);

  //C2 = Si_dot + Si_dot_dot*T
  double C2 = Si_dot + Si_dot_dot * T;

  //C3 = Si_dot_dot
  double C3 = Si_dot_dot;

  MatrixXd B(3, 3);
  B << pow(T, 3), pow(T, 4), pow(T, 5),
      3* pow(T, 2), 4 * pow(T, 3), 5 * pow(T, 4),
      6 * T, 12 * pow(T, 2), 20 * pow(T, 3);

  VectorXd s(3);
  s << Sf-C1,
      Sf_dot-C2,
      Sf_dot_dot-C3;

  VectorXd a = B.inverse() * s;

  VectorXd answer(6);
  answer << a0, a1, a2, a[0], a[1], a[2];
  return answer;
}

Goal PolynomialTrajectoryGenerator::perturb_goal(const Goal &goal) {
  VectorXd goal_s = goal.s;
  VectorXd goal_d = goal.d;

  VectorXd perturbed_goal_s(3);
  VectorXd perturbed_goal_d(3);

  std::default_random_engine random_number_generator;

  for (int i = 0; i < goal_s.rows(); ++i) {
    //create a normal distribution with goal_s[i] as mean (mu) and SIGMA_S[i]
    //as variance to account for noise
    std::normal_distribution<double> s_normal_distribution(goal_s[i], Constants::SIGMA_S[i]);
    //generate a s-number/goal_s using gaussian distribution
    perturbed_goal_s[i] = s_normal_distribution(random_number_generator);

    //similarly for d-coordinate
    std::normal_distribution<double> d_normal_distribution(goal_d[i], Constants::SIGMA_D[i]);
    perturbed_goal_d[i] = d_normal_distribution(random_number_generator);
  }

  Goal perturbed_goal(perturbed_goal_s, perturbed_goal_d, goal.t);
  return perturbed_goal;
}

vector<Goal> PolynomialTrajectoryGenerator::generate_perturbed_goals(const Goal &actual_goal) {
  vector<Goal> perturbed_goals;
  for (int i = 0; i < Constants::N_SAMPLES; ++i) {
    Goal perturbed_goal = perturb_goal(actual_goal);
    perturbed_goals.push_back(perturbed_goal);
  }

  return perturbed_goals;
}

vector<Trajectory> PolynomialTrajectoryGenerator::generate_trajectories(const VectorXd &start_s,
                                                                        const VectorXd &start_d,
                                                                        const vector<Goal> &goals) {
  vector<Trajectory> trajectories;
  for(int i = 0; i < goals.size(); ++i) {
    VectorXd s_coeffs = generate_jerk_minimized_trajectory(start_s, goals[i].s, goals[i].t);
    VectorXd d_coeffs = generate_jerk_minimized_trajectory(start_d, goals[i].d, goals[i].t);

    Trajectory trajectory(s_coeffs, d_coeffs, goals[i].t);
    trajectories.push_back(trajectory);
  }

  return trajectories;
}


Trajectory PolynomialTrajectoryGenerator::find_best_trajectory(const vector<Trajectory> &trajectories,
                                                               int target_vehicle_id,
                                                               const VectorXd &delta,
                                                               double T,
                                                               const vector<Vehicle> &predictions) {
  int min_trajectory_index = -1;
  double min_cost = 999999;

  CostFunctions cost_functions;
  for (int i = 0; i < trajectories.size(); ++i) {
    double cost = cost_functions.calculate_cost(trajectories[i], target_vehicle_id, delta, T, predictions);

    if (cost < min_cost) {
      min_cost = cost;
      min_trajectory_index = i;
    }
  }

  return trajectories[min_trajectory_index];
}

