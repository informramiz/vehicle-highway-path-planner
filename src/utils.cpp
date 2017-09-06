/*
 * utils.cpp
 *
 *  Created on: Aug 21, 2017
 *      Author: ramiz
 */

#include <math.h>
//#include "matplotlibcpp.h"
#include "utils.h"

//namespace plt = matplotlibcpp;

Utils::Utils() {
  // TODO Auto-generated constructor stub

}

Utils::~Utils() {
  // TODO Auto-generated destructor stub
}

double Utils::euclidean(double x1, double y1, double x2, double y2) {
  double dist_x = (x1 - x2);
  double dist_y =  (y1 - y2);
  double squared_dist = pow(dist_x, 2) + pow(dist_y, 2);

  return sqrt(squared_dist);
}

double Utils::euclidean_3d(int x1, int y1, double theta_rad1, int x2, int y2, double theta_rad2) {
  int dist_x = (x1 - x2);
  int dist_y =  (y1 - y2);
  double dist_theta = (theta_rad1 - theta_rad2);
  double squared_dist = pow(dist_x, 2) + pow(dist_y, 2) + pow(dist_theta, 2);

  return sqrt(squared_dist);
}

double Utils::deg2rad(double delta_i) {
  return M_PI / 180.0 * delta_i;
}

double Utils::logistic(double x) {
  return (2.0 / (1 + exp(-x))) - 1.0;
}

double Utils::solve_polynomial(const VectorXd &coeffs, double x) {
  double total = 0.0;
  for (int i = 0; i < coeffs.rows(); ++i) {
    total += coeffs[i] * pow(x, i);
  }

  return total;
}

VectorXd Utils::differentiate(const VectorXd &coeffs) {
  vector<double> new_coeffs;
  //ignore the first coefficient as it will always be 0 after
  //differentiation. Multiply power/degree of each term with with its coefficient
  for (int i = 1; i < coeffs.rows(); ++i) {
    double new_coeff = i * coeffs[i];
    new_coeffs.push_back(new_coeff);
  }

  return Utils::vectorToVectorXd(new_coeffs);
}

double Utils::nearest_approach_to_vehicle(const Trajectory &trajectory, const Vehicle &vehicle) {

  //assign a very high value initially
  double closest = 999999;

  //we will divide total time in 100 steps and check for each timestep in trajectory.t duration to find
  //out value of (s, d) for trajectory and for given vehicle at that time step
  for (int i = 0; i < 100; ++i) {
    //consider i% of total timestep for eachc iteration
    double t = trajectory.t/100 * i;

    //get s-coordinate value at time t
    //by solving trajectory.s_coeffs polynomial function
    //at time t
    double trajectory_s = Utils::solve_polynomial(trajectory.s_coeffs, t);

    //get d-coordinate value at time t
    //by solving trajectory.s_coeffs polynomial function
    //at time t
    double trajectory_d = Utils::solve_polynomial(trajectory.d_coeffs, t);

    //now predict the state of target vehicle at time t
    VectorXd target_vehicle_state = vehicle.state_at(t);
    //extract (s, d)
    double target_vehicle_s = target_vehicle_state[0];
    double target_vehicle_d = target_vehicle_state[3];

    //calculate the euclidean distance between trajectory's (s, d)
    //and target vehicle's (s, d) at time t
    double distance = euclidean(trajectory_s, trajectory_d, target_vehicle_s, target_vehicle_d);

    if (distance < closest) {
      closest = distance;
    }
  }

  return closest;
}

double Utils::nearest_approach_to_any_vehicle(const Trajectory &trajectory, const vector<Vehicle> &vehicles) {
  double closest_to_any_vehicle = 999999;

  for (int i = 0; i < vehicles.size(); ++i) {
    //calculate closest approach of trajectory to current vehicle
    double closest_to_current_vehicle = nearest_approach_to_vehicle(trajectory, vehicles[i]);

    if (closest_to_current_vehicle < closest_to_any_vehicle) {
      closest_to_any_vehicle = closest_to_current_vehicle;
    }
  }

  return closest_to_any_vehicle;
}

void Utils::plot_trajectory(const Trajectory &trajectory, const Vehicle &vehicle, bool plot_vehicle) {
  //vectors to hold (s, d) values for given trajectory
  vector<double> trajectory_s_values;
  vector<double> trajectory_d_values;

  //vectors to hold (s, d) values for given vehicle
  vector<double> vehicle_s_values;
  vector<double> vehicle_d_values;

  //for each timestep of 0.25 till total time/duration of trajectory.t
  double t = 0;
  while (t <= trajectory.t + 0.01) {
    //use trajectory s_coefficients to solve polynomial function of time at time t
    //to get the value of s-coordinate at timestep t
    double trajectory_s = solve_polynomial(trajectory.s_coeffs, t);
    //similary for d-coordinate
    double trajectory_d = solve_polynomial(trajectory.d_coeffs, t);

    //append these values to list of (s, d) values
    trajectory_s_values.push_back(trajectory_s);
    trajectory_d_values.push_back(trajectory_d);

    //if need to plot target vehicle as vehicle then
    //calculate (s, d) values for its trajectory as well
    if (plot_vehicle) {
      //predict vehicle state at time step t
       VectorXd vehicle_predicted_state = vehicle.state_at(t);

       //add predicted (s, d) to list
       vehicle_s_values.push_back(vehicle_predicted_state[0]);
       vehicle_d_values.push_back(vehicle_predicted_state[3]);
    }

    //increment time
    t += 0.25;
  }

//  plt::named_plot("Self-Driving-Car", trajectory_s_values, trajectory_d_values, "b--");
//  if (plot_vehicle) {
//    plt::named_plot("Target Vehicle", vehicle_s_values, vehicle_d_values, "r--");
//  }
//
//  plt::xlim(0, 100);
//  plt::ylim(0, 6);
//  plt::legend();
//  plt::show();
}

vector<vector<double> > Utils::get_trajectory_points(const Trajectory &trajectory, int points_count) {
  //vectors to hold (s, d) values for given trajectory
  vector<double> trajectory_s_values;
  vector<double> trajectory_d_values;

  //vectors to hold (s, d) values for given vehicle
  vector<double> vehicle_s_values;
  vector<double> vehicle_d_values;

  int count = 0;
  //for each timestep of 0.25 till total time/duration of trajectory.t
  double t = 0;
  while (t <= trajectory.t + 0.01) {
    //use trajectory s_coefficients to solve polynomial function of time at time t
    //to get the value of s-coordinate at timestep t
    double trajectory_s = solve_polynomial(trajectory.s_coeffs, t);
    //similary for d-coordinate
    double trajectory_d = solve_polynomial(trajectory.d_coeffs, t);

    //append these values to list of (s, d) values
    trajectory_s_values.push_back(trajectory_s);
    trajectory_d_values.push_back(trajectory_d);

    //increment time
    t += 0.25;
    count += 1;

    if (count >= points_count) {
      break;
    }
  }

  vector<vector<double> > pts;
  pts.push_back(trajectory_s_values);
  pts.push_back(trajectory_d_values);

  return pts;
}

int get_lane(double d) {
  if (d >= 0 && d <= 4) {
    return 0;
  } else if (d > 4 && d <= 8) {
    return 1;
  } else if (d > 8 && d <= 12) {
    return 2;
  }
}

int Utils::find_nearest_vehicle_ahead(const vector<Vehicle> &vehicles, double s, double d) {
  int nearest_distance = 999999;
  int index = -1;
  for (int i = 0; i < vehicles.size(); ++i) {
    //check if in same lane and ahead of given s
    if (get_lane(vehicles[i].get_d()) == get_lane(d) && vehicles[i].get_s() > s) {
      double distance = vehicles[i].get_s() - s;

      if (distance < nearest_distance) {
        nearest_distance = distance;
        index = i;
      }
    }
  }

  return index;
}

double Utils::get_d_value_for_lane_center(int lane) {
  //as car drives in center so add 2m for to count current lane
  //and as each lane is 4m so multiply by 4 to count other lanes before on left
  return 2 + lane * 4;
}




