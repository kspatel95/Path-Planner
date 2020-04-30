# Highway Path Planning Project
## Overview
This project's goal was to navigate a freeway with 3 lanes and traffic while finding the optimal path to efficiently navigate through.

### Requirements

Speed limit on the freeway is 50 mph.
The car needs to stay within 10m/s^2 acceleration and 10m/s^3 of jerk.
The car must avoid colliding with other traffic.
The car has to stay within the lane boundary, except when lane changing.
Needs to be able to navigate the freeway for over 4.32 miles without incident.

## Implementation
The path planner for this project was split into 3 main segments that handled detection, trajectory, and behaviors to utilize that data.

### Detection

The detection program was created to take in the raw sensor fusion data and utilize that data to make helpful calculations for other parts of the path planner. Some key information determined from detection are lane boundaries, reference information for surrounding cars, checking if the car is too close to the lead car, and if adjacent lanes are available to move into.

Lane boundaries are determined by Frenet D coordinates that allow the car to know which lane it currently occupies. This also helps to know which sensor fusion data is most important to worry about, having detections in each lane enables the car's behaviors to make better decisions. The car maintains a speed just below the speed limit when possible but must also avoid a forward collision, this is done by creating a buffer zone ahead of the car with a distance readout to the lead car. The car is also checking adjacent lanes to assist the behaviors in changing to a faster lane or to stay in the current lane.

### Behaviors

Behaviors performs all the logic for the path planner. The behavior program uses costs to weigh different manuevers and conditions. The path planner is designed to take the most optimal option based on detection. Detection provides useful information about the distance to lead car, adjacent lanes, velocity of lead car, and also vehicles coming up in the blindspot. This is helpful data for the behavior program to efficiently utilize Auto Cruise and Auto Lane Change through a Finite State Machine.

Auto Cruise is the program that utilizes lead car information and a desired speed, just under the speed limit. Using costs for distance to lead car, and speed desired this allows for an Auto Cruise to be able to inteligently speed up, slow down, and maintain speed all while avoiding a forward collision. When the current lane ends up being more costly due to slower speeds or distance to lead car is within the buffer the car utilizes Auto Lane Change to check allowable lane change options and when the desired lane is available the car can smoothly transition in. These two make up the bulk of the Finite State Machine, the logic intends for the car to remain in Auto Cruise to keep forward collision checks in place, when adjacent lanes are lower cost the car will utilize Auto Lane Change and resume back with Auto Cruise.

### Trajectory

The Trajectory portion is how all the behavior outputs get implemented. It takes in the initial states of the car and starts passing points in x and y coordinates to the path planner for the car to follow. The car reaches the next point every 0.2 seconds so spacing the points out speeds up the car and shinking the distance slows down the car. By keeping track of previous, current, and next points we can create a smooth spline connecting the points to a smooth curve that the car can follow.

The spline is the mesh that connects the points for the car. The car calculates future points from the behavior outputs and changes those points to local coordinates for the car. It also remembers the previous two points to create a smoother trajectory for the car. This is then converted back to Cartesian coordinates to send back to the simulator.

---
---
# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
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

## Basic Build Instructions

1. Clone this repo.
2. Clean, Compile, and Run: `cd /home/workspace && ./path_planner.sh`

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