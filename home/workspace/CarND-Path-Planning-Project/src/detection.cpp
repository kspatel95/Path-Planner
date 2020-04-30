#include "detection.hpp"
#include <iostream>

using json = nlohmann::json;

// Constructor
detection::detection() {
}

// Destructor
detection::~detection() {
}

// Lane boundaries: 0 = Left, 1 = Center, 2 = Right
double lane_boundaries(double car_d) {
  double lane = 1; // default start lane
  
  if (car_d > 0 && car_d <= 4) {
    lane = 0; // left lane
  } else if (car_d > 4 && car_d <= 8) {
    lane = 1; // center lane
  } else if (car_d > 8 && car_d <= 12) {
    lane = 2; // right lane
  }
  return lane;
}

// Calculating velocity and predicted distance
std::vector<double> detection::check_prediction(double vx, double vy, double prev_size) {
  double speed_magnitude = sqrt(pow(vx,2) + pow(vy,2));
  double predicted_s = prev_size * 0.02 * speed_magnitude;

  std::vector<double> vel_pred_s = {speed_magnitude, predicted_s};

  return vel_pred_s;
}

// Check lane for opening by observing leading and trailing cars
lead_car detection::check_lane(json& sensor_fusion, int& lane, json& previous_path, double& car_s, double& end_traj_s) {
  double closest_lead = std::numeric_limits<double>::max();
  double closest_trail = -std::numeric_limits<double>::max();
  double lead_speed;
  
  // Car position is placed at the end of previous trajectory
  if (previous_path.size() > 0) {
    car_s = end_traj_s;
  }
  // Update sensor fusion data
  for (int i=0; i < sensor_fusion.size(); i++) {
    float d = sensor_fusion[i][6];
    double car_lane = lane_boundaries(d);
    // Compare car and prediction with sensor fusion data
    if (lane == car_lane) {
      std::vector<double> vel_pred_s = check_prediction(sensor_fusion[i][3], sensor_fusion[i][4], previous_path.size());
      lead_speed = vel_pred_s[0];
      double predicted_s = (double)sensor_fusion[i][5] + vel_pred_s[1];
      if ((predicted_s > 0.0001) && (predicted_s > car_s)) {
        closest_lead = std::min(predicted_s, closest_lead);
      } else if ((predicted_s <= 0.0001) && (predicted_s < car_s)) {
        closest_trail = std::max(predicted_s, closest_trail);
      }
    }
  }
  // Update lead_car with new calculations
  lead_car lead;
  if ((closest_lead - car_s > lead_car_gap) && (car_s - closest_trail > trail_car_gap)) {
    lead.open_lane = true;
  } else if ((closest_lead - car_s < lead_car_gap) && (car_s - closest_trail < trail_car_gap)) {
    lead.open_lane = false;
  }

  if (closest_lead < (car_s + lead_car_gap)) {
    lead.too_close_lead = true;
  } else {
    lead.too_close_lead = false;
  }

  if (closest_trail > (car_s - trail_car_gap)) {
    lead.too_close_trail = true;
  } else {
    lead.too_close_trail = false;
  }

  // Update values for lead_car
  lead.lane_number = lane;
  lead.velocity = lead_speed;
  lead.lead_s = closest_lead;
  lead.trail_s = closest_trail;

  return lead;
}