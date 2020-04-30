#include <vector>
#include <iostream>
#include "trajectory.hpp"
#include "spline.h"
#include "json.hpp"

using namespace std;
using json = nlohmann::json;

// Constructor
trajectory::trajectory() {
}

// Destructor
trajectory::~trajectory() {
}

// Initialize variables and set constants and parameters
void trajectory::init(int& car_lane, double& max_vel, json& previous_path, double& car_x, double& car_y, double& car_yaw) {
  ref_lane = car_lane;
  ref_vel = max_vel;
  prev_size = previous_path.size();
  ref_x = car_x;
  ref_y = car_y;
  ref_yaw = deg2rad(car_yaw);
}

// Find previous trajectory points, utilize 2 otherwise use current position
void trajectory::previous_points(json& prev_x_points, json& prev_y_points, double& car_x, double& car_y, double& car_yaw) {
  if (prev_size < 2) {
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);
    
    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);
    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
    
  } else {
    ref_x = prev_x_points[prev_size - 1];
    ref_y = prev_y_points[prev_size - 1];
    double ref_x_prev = prev_x_points[prev_size - 2];
    double ref_y_prev = prev_y_points[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
    
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);
    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }
}

// Find scale for next trajectory points by calculating distance per point
int trajectory::next_points(int num_points, double& car_s, vector<double>& map_waypoints_s, vector<double>& map_waypoints_x, vector<double>& map_waypoints_y) {
  // Check that distance is divisible by num_points
  if (max_length % num_points != 0) {
    cout << "ERROR: num_points not divisible by dist" << endl;
    exit(EXIT_FAILURE);
  }
  int scale_factor = max_length / num_points;
  while (scale_factor <= max_length) {
    vector<double> next_waypoint = getXY(car_s + scale_factor, (2 + 4 * ref_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    
    ptsx.push_back(next_waypoint[0]);
    ptsy.push_back(next_waypoint[1]);
    scale_factor += scale_factor;
  }
  return max_length / num_points;
}

// Change the coordinate system to local coordinates of the car
void trajectory::global2local() {
  for (int i=0; i < ptsx.size(); i++) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }
}

// Use spline to create a smooth trajectory, utilizing previous path points.
void trajectory::spline_trajectory(int& scale_factor, json& prev_x_points, json& prev_y_points, vector<double>& next_x_vals, vector<double>& next_y_vals) {
  tk::spline s;
  s.set_points(ptsx, ptsy);
  double target_x = scale_factor;
  double target_y = s(target_x);
  double target_distance = sqrt(pow(target_x,2) + pow(target_y,2));
  double x_addon = 0.0;

  for (int i=0; i < prev_size; i++) {
    next_x_vals.push_back(prev_x_points[i]);
    next_y_vals.push_back(prev_y_points[i]);
  }
  for (int j=1; j < max_points - prev_size; j++) {
    double N = (target_distance/(0.02*ref_vel/2.24)); // Converting mph to m/s
    double x_point = x_addon + target_x / N;
    double y_point = s(x_point);
    x_addon = x_point;
    double x_ref = x_point;
    double y_ref = y_point;

    // Switch back to global coordinate system
    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
}