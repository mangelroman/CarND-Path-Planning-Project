# CarND-Path-Planning-Project
This is the first project of term 3 in the Self-Driving Car Engineer Nanodegree Program by Udacity.

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Description of the model

The model is implemented in the `Planner`class, which contains the following classes:
- `Map`class, responsible to manage road data and coordinate transformations.
- `Prediction`class, responsible to predict the behavior of the vehicles surrounding us.
- `Behavior`class, responsible to choose our car's behavior based on the environment.
- `Trajectory`class, responsible to generate the path our car must follow based on the desired behavior and safety constraints.

### Map class

Based on this road data loaded from a file upon initialization, this module is able to perform the following calculations:
- Frenet to cartesian coordinate translation (using spline library to interpolate map data)
- Distance between 2 points given its frenet `s` coordinates (takes into account circular road)
- Curvature of a road segment given its frenet `s` coordinates
- Tangential (`s` dot) and Normal (`d` dot) velocities given the velocity vector
- Several lane calculations based on frenet `d` coordinates

This is a plot of the road data that shows its circular nature:

![alt text](data/road.png)


### Prediction class

This module takes sensor fusion data and computes the probabilities of the different behaviors the surrounding vehicles may have. Based on our virtual highway simulator, these are the possible behaviors we should account for:
1. Acceleration/Deceleration (change in tangential velocity)
2. Changing lanes (change in normal velocity)

For simplification purposes, the prediction module just computes the tangential and normal velocities in real-time instead of future probabilities of behaviors based on history. With this values the subsequent modules can predict whether a surrounding vehicle is changing speed or lanes.

### Behavior class

This module implements a Finite State Machine (FSM) to determine what to do based on the environment. This is the FSM diagram:
![alt text](data/fsm.png)

#### Stop and Start
Stop is the initial state, and Start takes our car to the center of the lane at the maximum speed.

#### Keep Lane
If our car is already in the best lane at the target speed, it must stay there following the road.

#### Changing Lanes
The best lane is selected using a simple cost function. The `Behavior`class will always try to change to the best lane if the change is feasible. In order for a lane change to be feasible, there must be enough front and back gap once our car is in the target lane. The gaps are computed using vehicle distances and relative velocities.

The cost function is designed to favor the center lane when all the lanes are empty.

#### Changing Speed
If there are other vehicles in front of us at a minimum distance, the target speed is set to the vehicle in front of us. Otherwise, the target speed is set to the maximum speed.

### Trajectory class

This module generates the best trajectory, always in frenet coordinates, in the next 4 seconds time frame based on the state given by the `Behavior` class. The previously generated trajectory is stored in a circular buffer for convenience, so the previous `(s,d)` points can be reused when applicable (i.e. `Keep Lane` state).

#### JMT
Jerk Minimizing Trajectory (JMT) algorithm is used to generate trajectories with smooth changes of speed (`s` coordinate) and lane (`d` coordinate).

#### Safety and Comfort Constraints
The trajectory module is also responsible to ensure the safety and comfort constraints are applied. These are:
- Maximum Acceleration of 10 m/s^2
- Maximum Jerk of 10 m/s^3
- Maximum Speed of 50mph

**Maximum Acceleration and Jerk**
Ideally, the `Trajectory` class should generate multiple trajectories with different parameters and choose the one that ensures the acceleration and jerk values are within safety and comfort limits. However, for simplification purposes, the trajectories are generated with conservative values that in our simulator's environment are always below the given thresholds.

**Maximum Speed**
As the trajectories are always referred to frenet coordinates, the curvature of the road is used to ensure the maximum speed is never surpassed regardless of the lane (`d` coordinate) where the car is. To reach the target speed at any given lane, the `s` coordinate delta is obtained using this formula:

```c
s_delta = target_speed * time + d * sin(curvature);
```
where curvature is the difference of `d` vector angles between two `s` points computed by the `Map` class.

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
