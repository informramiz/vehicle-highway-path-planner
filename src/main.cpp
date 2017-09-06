#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen/Core"
#include "Eigen/QR"
#include "Eigen/Dense"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"
#include "utils.h"
#include "map_utils.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int UdacityCode();
int MyCode();

int main() {
  MyCode();
  return 0;
}


int UdacityCode() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  if (!in_map_.is_open()) {
    cerr << "Unable to open map file" << endl;
    exit(-1);
  }

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  cout << "map reading complete" << endl;

  //define some initial states
  //start lane: 0 means far left lane, 1 means middle lane, 2 means right lane
  int lane = 1;

  //define SPEED LIMIT
  const int SPEED_LIMIT = 50; //mph

  //define desired velocity
  double ref_velocity = 0; //mph

  h.onMessage([&ref_velocity, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy]
               (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          //The data format for each car is: [ id, x, y, vx, vy, s, d]
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          if (previous_path_x.size() > 0) {
            car_s = end_path_s;
          }

          bool is_too_close = false;
          for (int i = 0; i < sensor_fusion.size(); ++i) {
            //d-coordinate is at index 6
            double other_vehicle_d = sensor_fusion[i][6];
            //s-coordinate is at index 5
            double other_vehicle_s = sensor_fusion[i][5];
            //access vx, vy which are at indexes (3, 4)
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];

            //check if this car is in my lane
            //I am adding +2 or -2 to (2+4*lane) formula because other
            //vehicle may not be in lane center and this formula is for lane center
            //so adding +2 or -2 makes the boundary exactly 4 meters so even if
            //vehicle is not at lane center if it is in 4 meter range of lane it
            //will be considered as in-lane vehicle
            if (other_vehicle_d < (2+4*lane+2) && other_vehicle_d > (2+4*lane-2)) {
              //as vehicle is in our lane, so let's predict its `s`
              double other_vehicle_speed = sqrt(vx*vx + vy*vy);
              //predict s = s + delta_t * v
              //also multiply (delta_t * v) term with prev_path_size
              //as simulator may not have completed previous path
              //so vehicle may not be there yet where we are expecting it to be
              double other_vehicle_predicted_s = other_vehicle_s + 0.02 * other_vehicle_speed * previous_path_x.size();

              //check if vehicle is infront of ego vehicle
              //and distance between that vehicle and our vehicle is less than 30 meters
              if (other_vehicle_predicted_s > car_s && (other_vehicle_s - car_s) < 10) {
                is_too_close = true;
                lane = (lane + 1) % 3;
              }
            }

          }


          if (is_too_close) {
            //if collision danger then decrease speed
            ref_velocity -= 0.224;
          } else if (ref_velocity < 49.5) {
            //if no collision danger and we are under speed limit
            //then increase speed
            ref_velocity += 0.244;
          }

          /***********Process Data****************/

          //reference point where the car is right now
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = Utils::deg2rad(car_yaw);

          vector<double> points_x;
          vector<double> points_y;

          //by the time we receive this data simulator may not have
          //completed our previous 50 points so we are going to consider
          //them here as well to make transition smooth
          int prev_path_size = previous_path_x.size();

          //check if there are any points in previous path
          if (prev_path_size < 2) {
            //there are not enough points so consider where the car is right now
            //and where the car was before that (current point) point
            //so predict past for delta_t = 1
            double prev_x = car_x - cos(car_yaw);
            double prev_y = car_y - sin(car_yaw);

            //add point where the car was 1 timestep before
            points_x.push_back(prev_x);
            points_y.push_back(prev_y);

            //add point where is car right now
            points_x.push_back(car_x);
            points_y.push_back(car_y);
          } else {
            //as previous path is still not completed by simulator so
            //consider end of previous path as reference point for car
            //(instead of current point which is somewhere in previous path)
            //to calculate
            ref_x = previous_path_x[prev_path_size - 1];
            ref_y = previous_path_y[prev_path_size - 1];

            //get the point before reference point (2nd last point in prev path)
            double x_before_ref_x = previous_path_x[prev_path_size - 2];
            double y_before_ref_y = previous_path_y[prev_path_size - 2];

            //as these two points make a tangent line to the car
            //so we can calculate car's yaw angle using these
            //two points
            ref_yaw = atan2(ref_y - y_before_ref_y, ref_x - x_before_ref_x);

            //add point where the car was before reference point to list of points
            points_x.push_back(x_before_ref_x);
            points_y.push_back(y_before_ref_y);

            //add the last point in previous path
            points_x.push_back(ref_x);
            points_y.push_back(ref_y);
          }

          //add 3 more points, each spaced 30m from other in Frenet coordinates
          //start from where the car is right now
          //Remember: each lane is 4m wide and we want the car to be in middle of lane
          double d_coord_for_middle_lane = (2 + 4*lane);
          vector<double> next_wp0 = MapUtils::getXY(car_s + 30, d_coord_for_middle_lane);
          vector<double> next_wp1 = MapUtils::getXY(car_s + 60, d_coord_for_middle_lane);
          vector<double> next_wp2 = MapUtils::getXY(car_s + 90, d_coord_for_middle_lane);

          //add these 3 points to way points list
          points_x.push_back(next_wp0[0]);
          points_y.push_back(next_wp0[1]);

          points_x.push_back(next_wp1[0]);
          points_y.push_back(next_wp1[1]);

          points_x.push_back(next_wp2[0]);
          points_y.push_back(next_wp2[1]);


          //to make our math easier, let's convert these points
          //from Global maps coordinates to vehicle coordinates
          for (int i = 0; i < points_x.size(); ++i) {
            MapUtils::TransformToVehicleCoordinates(ref_x, ref_y, ref_yaw, points_x[i], points_y[i]);
          }

          //spline to fit a curve through the way points we have
          //spline is fitting that makes sure that the curve passes
          //through the all the points
          tk::spline spline;

          //fit a spline through the ways points
          //we will use this fitting to get any point on
          //this line (extrapolation of points)
          spline.set_points(points_x, points_y);

          //now that we have a spline fitting we
          //can get any point on this line
          //(if given x, this spline will give corresponding y)
          //but we still have to space our points on spline
          //so that we can achieve our desired velocity

          //our reference velocity is in miles/hour we need to
          //convert our desired/reference velocity in meters/second for ease
          //Remember 1 mile = 1.69 km = 1609.34 meters
          //and 1 hours = 60 mins * 60 secs = 3600
          double ref_velocity_in_meters_per_second = ref_velocity * (1609.34 / (60*60));
          cout << "ref velocity m/s: " << ref_velocity_in_meters_per_second << endl;

          //as we need to find the space between points on spline to
          //to keep our desired velocity, to achieve that
          //we can define some target on x-axis, target_x,
          //and then find how many points should be there in between
          //x-axis=0 and x-axis=target_x so that if we keep our desired
          //velocity and each time step is 0.02 (20 ms) long then we achieve
          //our target distance between 0 to target_x, target_dist
          //
          //formula: V = d / t
          //as each timestep = 0.02 secs so
          //formula: V = d / (0.02 * N)
          //--> ref_v = target_dist / (0.02 * N)
          //--> N = target_dist / (0.02 * ref_v)

          double target_x = 30.0;
          //get the target_x's corresponding y-point on spline
          double target_y = spline(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          double N = target_dist/ (0.02 * ref_velocity_in_meters_per_second);

          //here N is: number of points from 0 to target_dist_x
          //
          //--> point_space = target_x / N
          double point_space = target_x / N;


          //now we are ready to generate trajectory points
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //but first let's add points of previous_path
          //as they have not yet been traversed by simulator
          //and we considered its end point as the reference point
          for (int i = 0; i < prev_path_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          //now we can generate the remaining points (50 - prev_path.size)
          //using the spline and point_space

          //as we are in vehicle coordinates so first x is 0
          double x_start = 0;

          for (int i = 0; i < 50 - prev_path_size; ++i) {
            double point_x = x_start + point_space;
            double point_y = spline(point_x);

            //now the current x is the new start x
            x_start = point_x;

            //convert this point to Global map coordinates which
            //is what simulator expects
            MapUtils::TransformFromVehicleToMapCoordinates(ref_x, ref_y, ref_yaw, point_x, point_y);

            //add this point to way points list
            next_x_vals.push_back(point_x);
            next_y_vals.push_back(point_y);
          }

          /***************END Processing of data***************/

          json msgJson;

          // define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
      size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
      char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();

  return 0;
}

int MyCode() {
  uWS::Hub h;

  // Waypoint map to read from
  const string map_file = "data/highway_map.csv";
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  MapUtils::Initialize(map_file);

  // The max s value before wrapping around the track back to 0
  const double max_s = 6945.554;

  //define some initial states
  //start lane: 0 means far left lane, 1 means middle lane, 2 means right lane
  int lane = 1;

  //define SPEED LIMIT
  const int SPEED_LIMIT = 50; //mph

  //define desired velocity
  double ref_velocity = 0; //mph

  h.onMessage([&ref_velocity, &lane]
               (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          //The data format for each car is: [ id, x, y, vx, vy, s, d]
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          if (previous_path_x.size() > 0) {
            car_s = end_path_s;
          }

          bool is_too_close = false;
          for (int i = 0; i < sensor_fusion.size(); ++i) {
            //d-coordinate is at index 6
            double other_vehicle_d = sensor_fusion[i][6];
            //s-coordinate is at index 5
            double other_vehicle_s = sensor_fusion[i][5];
            //access vx, vy which are at indexes (3, 4)
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];

            //check if this car is in my lane
            //I am adding +2 or -2 to (2+4*lane) formula because other
            //vehicle may not be in lane center and this formula is for lane center
            //so adding +2 or -2 makes the boundary exactly 4 meters so even if
            //vehicle is not at lane center if it is in 4 meter range of lane it
            //will be considered as in-lane vehicle
            if (other_vehicle_d < (2+4*lane+2) && other_vehicle_d > (2+4*lane-2)) {
              //as vehicle is in our lane, so let's predict its `s`
              double other_vehicle_speed = sqrt(vx*vx + vy*vy);
              //predict s = s + delta_t * v
              //also multiply (delta_t * v) term with prev_path_size
              //as simulator may not have completed previous path
              //so vehicle may not be there yet where we are expecting it to be
              double other_vehicle_predicted_s = other_vehicle_s + 0.02 * other_vehicle_speed * previous_path_x.size();

              //check if vehicle is infront of ego vehicle
              //and distance between that vehicle and our vehicle is less than 30 meters
              if (other_vehicle_predicted_s > car_s && (other_vehicle_s - car_s) < 10) {
                is_too_close = true;
                lane = (lane + 1) % 3;
              }
            }

          }


          if (is_too_close) {
            //if collision danger then decrease speed
            ref_velocity -= 0.224;
          } else if (ref_velocity < 49.5) {
            //if no collision danger and we are under speed limit
            //then increase speed
            ref_velocity += 0.244;
          }

          /***********Process Data****************/

          //reference point where the car is right now
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = Utils::deg2rad(car_yaw);

          vector<double> points_x;
          vector<double> points_y;

          //by the time we receive this data simulator may not have
          //completed our previous 50 points so we are going to consider
          //them here as well to make transition smooth
          int prev_path_size = previous_path_x.size();

          //check if there are any points in previous path
          if (prev_path_size < 2) {
            //there are not enough points so consider where the car is right now
            //and where the car was before that (current point) point
            //so predict past for delta_t = 1
            double prev_x = car_x - cos(car_yaw);
            double prev_y = car_y - sin(car_yaw);

            //add point where the car was 1 timestep before
            points_x.push_back(prev_x);
            points_y.push_back(prev_y);

            //add point where is car right now
            points_x.push_back(car_x);
            points_y.push_back(car_y);
          } else {
            //as previous path is still not completed by simulator so
            //consider end of previous path as reference point for car
            //(instead of current point which is somewhere in previous path)
            //to calculate
            ref_x = previous_path_x[prev_path_size - 1];
            ref_y = previous_path_y[prev_path_size - 1];

            //get the point before reference point (2nd last point in prev path)
            double x_before_ref_x = previous_path_x[prev_path_size - 2];
            double y_before_ref_y = previous_path_y[prev_path_size - 2];

            //as these two points make a tangent line to the car
            //so we can calculate car's yaw angle using these
            //two points
            ref_yaw = atan2(ref_y - y_before_ref_y, ref_x - x_before_ref_x);

            //add point where the car was before reference point to list of points
            points_x.push_back(x_before_ref_x);
            points_y.push_back(y_before_ref_y);

            //add the last point in previous path
            points_x.push_back(ref_x);
            points_y.push_back(ref_y);
          }

          //add 3 more points, each spaced 30m from other in Frenet coordinates
          //start from where the car is right now
          //Remember: each lane is 4m wide and we want the car to be in middle of lane
          double d_coord_for_middle_lane = (2 + 4*lane);
          vector<double> next_wp0 = MapUtils::getXY(car_s + 30, d_coord_for_middle_lane);
          vector<double> next_wp1 = MapUtils::getXY(car_s + 60, d_coord_for_middle_lane);
          vector<double> next_wp2 = MapUtils::getXY(car_s + 90, d_coord_for_middle_lane);

          //add these 3 points to way points list
          points_x.push_back(next_wp0[0]);
          points_y.push_back(next_wp0[1]);

          points_x.push_back(next_wp1[0]);
          points_y.push_back(next_wp1[1]);

          points_x.push_back(next_wp2[0]);
          points_y.push_back(next_wp2[1]);


          //to make our math easier, let's convert these points
          //from Global maps coordinates to vehicle coordinates
          for (int i = 0; i < points_x.size(); ++i) {
            MapUtils::TransformToVehicleCoordinates(ref_x, ref_y, ref_yaw, points_x[i], points_y[i]);
          }

          //spline to fit a curve through the way points we have
          //spline is fitting that makes sure that the curve passes
          //through the all the points
          tk::spline spline;

          //fit a spline through the ways points
          //we will use this fitting to get any point on
          //this line (extrapolation of points)
          spline.set_points(points_x, points_y);

          //now that we have a spline fitting we
          //can get any point on this line
          //(if given x, this spline will give corresponding y)
          //but we still have to space our points on spline
          //so that we can achieve our desired velocity

          //our reference velocity is in miles/hour we need to
          //convert our desired/reference velocity in meters/second for ease
          //Remember 1 mile = 1.69 km = 1609.34 meters
          //and 1 hours = 60 mins * 60 secs = 3600
          double ref_velocity_in_meters_per_second = ref_velocity * (1609.34 / (60*60));
          cout << "ref velocity m/s: " << ref_velocity_in_meters_per_second << endl;

          //as we need to find the space between points on spline to
          //to keep our desired velocity, to achieve that
          //we can define some target on x-axis, target_x,
          //and then find how many points should be there in between
          //x-axis=0 and x-axis=target_x so that if we keep our desired
          //velocity and each time step is 0.02 (20 ms) long then we achieve
          //our target distance between 0 to target_x, target_dist
          //
          //formula: V = d / t
          //as each timestep = 0.02 secs so
          //formula: V = d / (0.02 * N)
          //--> ref_v = target_dist / (0.02 * N)
          //--> N = target_dist / (0.02 * ref_v)

          double target_x = 30.0;
          //get the target_x's corresponding y-point on spline
          double target_y = spline(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          double N = target_dist/ (0.02 * ref_velocity_in_meters_per_second);

          //here N is: number of points from 0 to target_dist_x
          //
          //--> point_space = target_x / N
          double point_space = target_x / N;


          //now we are ready to generate trajectory points
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //but first let's add points of previous_path
          //as they have not yet been traversed by simulator
          //and we considered its end point as the reference point
          for (int i = 0; i < prev_path_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          //now we can generate the remaining points (50 - prev_path.size)
          //using the spline and point_space

          //as we are in vehicle coordinates so first x is 0
          double x_start = 0;

          for (int i = 0; i < 50 - prev_path_size; ++i) {
            double point_x = x_start + point_space;
            double point_y = spline(point_x);

            //now the current x is the new start x
            x_start = point_x;

            //convert this point to Global map coordinates which
            //is what simulator expects
            MapUtils::TransformFromVehicleToMapCoordinates(ref_x, ref_y, ref_yaw, point_x, point_y);

            //add this point to way points list
            next_x_vals.push_back(point_x);
            next_y_vals.push_back(point_y);
          }

          /***************END Processing of data***************/

          json msgJson;

          // define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
      size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
      char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();

  return 0;
}















































































