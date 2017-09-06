/*
 * Vehicle.h
 *
 *  Created on: Jul 29, 2017
 *      Author: ramiz
 */

#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include "snapshot.h"

using namespace std;

class Vehicle {
public:

  struct collider {

    bool collision; // is there a collision?
    int time; // time collision happens

  };

  int L = 1;

  double preferred_buffer = 6; // impacts "keep lane" behavior.

  int id;
  double x;
  double y;
  int lane;
  double s;
  double d;
  double yaw;
  double v;
  double a;

  /**
   * Constructor
   */
  Vehicle(int id, double x, double y, double s, double d, double yaw_radians, double v, double a);

  /**
   * Destructor
   */
  virtual ~Vehicle();

  string display() const;

  void increment(int dt=1);

  vector<double> state_at(double t);

  vector<vector<double> > generate_predictions(double horizon=1);

private:
  /**
   * Takes snapshot of current state of vehicle and saves it into snapshot object
   * @returns  Snapshot object containing current state of vehicle
   */
  Snapshot take_current_state_snapshot();
  /**
   * Sets current state of vehicle to that of saved in passed snapshot
   * @param snapshot  snapshot containing state of vehicle to restore from
   */
  void restore_state_from_snapshot(const Snapshot& snapshot);

  /**
   * @param state, state for which trajectory to calculate
   * @param predictions, map of predictions of other vehicle's predicted trajectories
   * @returns Returns trajectory to for reaching given state
   */
  void remove_first_prediction_for_each(map<int, vector<vector<int> > > &predictions);

};

#endif
