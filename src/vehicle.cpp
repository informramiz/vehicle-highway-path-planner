/*
 * Vehicle.cpp
 *
 *  Created on: Jul 29, 2017
 *      Author: ramiz
 */

#include <cassert>
#include <iostream>
#include <algorithm>
#include "vehicle.h"
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include "utils.h"

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(double s, double d, double v, double a) {

  this->lane = Utils::get_lane(d);
  this->s = s;
  this->d = d;
  this->v = v;
  this->a = a;
}

Vehicle::~Vehicle() {
}

void Vehicle::remove_first_prediction_for_each(map<int, vector<vector<int> > > &predictions) {
  map<int, vector<vector<int> > >::iterator iter = predictions.begin();

  while (iter != predictions.end()) {
    //map value (second element in iterator) is vector with each vector value a vector: [s, lane]
    //take a reference to this second vector object so that we can update it
    vector<vector<int> > &v_preds = iter->second;

    //remove first vector of this prediction
    v_preds.erase(v_preds.begin());

    //move to next vehicle predictions
    iter++;
  }
}

/**
 * Takes snapshot of current state of vehicle and saves it into snapshot object
 * @return  Snapshot object containing current state of vehicle
 */
Snapshot Vehicle::take_current_state_snapshot() {
  return Snapshot(this->lane,
      this->s,
      this->v,
      this->a);
}
/**
 * Sets current state of vehicle to that of saved in passed snapshot
 * @param snapshot  snapshot containing state of vehicle to restore from
 */
void Vehicle::restore_state_from_snapshot(const Snapshot& snapshot) {
  this->lane = snapshot.lane;
  this->s = snapshot.s;
  this->v = snapshot.v;
  this->a = snapshot.a;
}

string Vehicle::display() const{

  ostringstream oss;

  oss << "s:    " << this->s << "\n";
  oss << "d:    " << this->d << "\n";
  oss << "lane: " << this->lane << "\n";
  oss << "v:    " << this->v << "\n";
  oss << "a:    " << this->a << "\n";

  return oss.str();
}

void Vehicle::increment(int dt) {

  this->s += this->v * dt;
  this->v += this->a * dt;
}

vector<double> Vehicle::state_at(double t) {

  /*
   Predicts state of vehicle in t seconds (assuming constant acceleration)
   */
  double s = this->s + this->v * t + this->a * t * t / 2;
  double v = this->v + this->a * t;
  return {this->lane, s, v, this->a};
}

vector<vector<double> > Vehicle::generate_predictions(double horizon = 1) {

  vector<vector<int> > predictions;
  double t = 0;
  while (t < horizon + 0.001) {
    vector<int> check1 = state_at(t);
    vector<int> lane_s = { check1[0], check1[1] };
    predictions.push_back(lane_s);
  }
  return predictions;

}

