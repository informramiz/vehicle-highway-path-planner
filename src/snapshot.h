/*
 * snapshot.h
 *
 *  Created on: Aug 10, 2017
 *      Author: ramiz
 */

#ifndef SNAPSHOT_H_
#define SNAPSHOT_H_

#include <string>
using namespace std;

/**
 * A passive object to save a snapshot of state of vehicle
 * at any given time
 */
struct Snapshot {
  int lane;
  double s;
  double d;
  double v;
  double a;

 Snapshot(int lane, double s, double d, double v, double a) {
   this->lane = lane;
   this->s = s;
   this->d = d;
   this->v = v;
   this->a = a;
 }

 void print() const{
   printf("Lane %d, s %f, d %f, v %f, a %f, state %s \n", lane, s, d, v, a);
 }
};



#endif /* SNAPSHOT_H_ */
