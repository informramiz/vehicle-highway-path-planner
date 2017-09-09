# Autonomous Vehicle Highway Path Planner
A path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic. 

![demo](demo/animated.gif)

### Goals
In this project goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Approach

### Trajectory Generation 
For smooth trajectory generation I have used [spline](http://kluge.in-chemnitz.de/opensource/spline/) library. 

1. Take 2 points from previous path received from the Simulator (to keep transition smooth)
2. Take 3 equally distant points in proposed lane (the lane for which trajectory is being generated)
3. Convert these points from map coordinates to vehicle coordinates for ease of math.
4. Fit a `spline` through these points using which points for trajectory will be generated.
5. To keep acceleration and velocity smooth and to avoid jerk I calculate a `point_space` (distance between each point) to make sure vehicle accelerates or decelerate smoothly. 
6. By using this `point_space` and `spline` I generate 50 points (if previous path has some points left then 50-prev_path_size).
7. Convert these points back to map coordinates.

### Trajectory Cost Calculation
For calculating cost for any trajectory, I have define 3 cost functions in `cost_functions.cpp` file.

1. **CollisionCost** function with `weight 1000` and `collision distance 20`, to avoid collision with other vehicles. Collision distance can be decreased further but I wanted to keep it hight for safe side. For current lane the CollisionCost function always consider cost 0 because `collision avoidance module` in path planner file will kick in to handle that.
2. **BufferCost** function with `weight 30` and `buffer distance 30`, to keep a buffer distance from vehicles ahead.
3. **ChangeLaneCost** function with `weight 10`, to avoid change of lanes just because there is a small benefit. Change lane cost should only happen if it benefits considerly not because we can achieve 50cm more buffer distance.

### Path Planning
Path planning consists of 2 steps.

1. **Collision Avoidance:** If `ego vehicle` is too close to vehicle ahead then decrease its smooth slowly to avoid hitting the vehicle in front.

2. **Trajectory Selection:** Generate trajectories for each lane, find cost for each trajectory and select trajectory with best cost.

### Possible Improvements

- The cost functions are not that well balanced, they can be fine tuned.
- There are not cost functions for check max acceleration or max jerk. Although trajectory generation module takes care of that but still it will be a good idea to have them just in case.
- I have intentionally kept lane changing to be not very aggresive but car speed and efficiency can be improved.

### Class Details

- **trajectory_generator.cpp** contains code for trajectory generation. It uses `spline.h` library file to generate a smooth trajectory.

- **path_planner.cpp** contains code for slowing vehicle down, selecting best trajectory and returing that trajectory back.

- **cost_functions.cpp** contains all the cost functions and their weights that calculate cost for given trajectory.
- **map_utils.cpp** contains all map and coordinates conversion related code.
- **utils.cpp** contains some utility methods


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

### Simulator.
You can download the Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

                                                           
## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
