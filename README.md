# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)

[path-planner]: ./imgs/simulator-image.png "Model Visualization"
[state-machine]: ./imgs/state-diagram.png "State machine"

![Pathdriving][path-planner]
   
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

---

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


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

--- 

## Rubric points

1) The code compiles correctly. Code must compile without errors with cmake and make. Given that we've made CMakeLists.txt as general as possible, it's recommend that you do not change it unless you can guarantee that your changes will still compile on any platform.

    Code is able to compile. Changes are made to CMakeLists.txt file in order to include newly added files in build process.

2) The car is able to drive at least 4.32 miles without incident. The top right screen of the simulator shows the current/best miles driven without incident. Incidents include exceeding acceleration/jerk/speed, collision, and driving outside of the lanes. Each incident case is also listed below in more detail.

    Car is able to drive more than specified distance without any incidents.

3) The car drives according to the speed limit. The car doesn't drive faster than the speed limit. Also the car isn't driving much slower than speed limit unless obstructed by traffic.

    Car is driving within speed limits. When it is possible car is driving near speed limit, and when traffic is dense, car is adapting it's speed to the surrounding traffic

4) Max Acceleration and Jerk are not Exceeded. The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.

    Car does not exceed total acceleration and jerk limitations.

5) Car does not have collisions. The car must not come into contact with any of the other cars on the road.

    Car is able to drive specified distance without any collisions

6) The car stays in its lane, except for the time between changing lanes. The car doesn't spend more than a 3 second length out side the lane lanes during changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road.

    The car stays in its lane in scenario when there is no car in front. If there is car in front that is driving lower than speed limit, car should try to overtake it with lane chaning. Lane change is done effectively and it does not lasts for more than 3 seconds. In addition to this, car always stays in one of 3 lanes on the right hand side of the road.

7) The car is able to change lanes. The car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic.

    Car is able to change lanes in order to overtake slow moving vehicle. Car is changing lane only when it is safe and no collision might take place.

## Reflection

Car is able to navigate full distance of the track without any incidents. Main path planning module `VehicleController` defined in `vehicle_controller.h` (function `UpdatePath()`) implements main highway navigation logic. Model is design as state machine with 3 states (shown in figure bellow)

![state-machine]


KL - Keep Lane. 
LCL - Lane Change Left
LCR - Lane Change Right

Vehicle starts in Keep Lane state and if there is no vehicle in it's current lane it accelerates to near speed limit. Velocity controll is done via `VelocityController` defined in `velocity_controller.h`. This module handles moving actions like accelerating/deaccelerating/keeping constant speed.

After all state transitions are taken, and velocity controller has been instructed on what to do (accelerate, slow down, keep speed) than trajectory is generated via `TrajectoryGenerator` class defined in `trajectory_generator.h`, function `GenerateTrajectory()`. This class handles trajectory generation, based on given inputs such as vehicle's currnet state (x/y position, x/y speed, s/d possition etc.), target lane, and target velocity. Target lane and target velocity have already been calculated at this point via VehicleController logic. Trajectory is being calculated in following way:

- Trajectory is generated based on anchor points. In order to ensure smooth trajectory, previous 2 points are reused. In addition to this, 3 forward anchor points are added (dependent on target lane).
- Based on these anchor points, spline is fitted and this spline shall be used to generate filler points
- After anchor points have been generated, filler points are also generated. All unvisited points from previous iteration path are reused.
- Rest of filler points is calculated in following way: Horizon point is taken (Location 30 meters in front of the vehicle). Based on this location and target speed and acceleration filler points are calculated.
- These trajectory points are in the end forwarded to the simulator and they are vistited with ego vehicle.

As already said `VehicleController` implements following behavior:

- If there are no vehicles in front, keep current lane with speed close to the limit
- If current lane is occupied, adjust speed and consider overtaking
- Check is it safe to change lane to the left and to the right
- If lane change is safe, calculate cost of each lane change and choose maneuver with lower cost.
- Transition to LCL/LCR state
- Once lane change maneuver is completed, state tranisition to KL is taken and the cycle is repeated

Cost of maneuver LCL and LCR is calculated based on cost function. Cost function is Sigmoid function. Following criterias deterim cost of maneuver:

- Cost is greater if more cars are detected in target lane
- Cost  is greater if distance to the nearest car in front is short
- Cost is greater if difference between nearest car speed and speed limit is greater

Cost function is implemented in `LaneCostFunction()` defined in `VehicleController.cpp`


