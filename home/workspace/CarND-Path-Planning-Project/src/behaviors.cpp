#include "behaviors.hpp"
#include "detection.hpp"
#include "trajectory.hpp"
#include "readout.hpp"

using json = nlohmann::json;

// Default Constructor
behaviors::behaviors() {
  desired_lane = 1;
}

// Default Destructor
behaviors::~behaviors(){
}

// Cost for distance to lead car
double behaviors::cost_dist_front(lead_car& detection, double& lead_car_gap, double& car_s) {
  double cost_dist;

  if ((detection.lead_s - car_s) > lead_car_gap) {
    cost_dist = 0.0;
  } else if ((detection.lead_s - car_s) < lead_car_gap) {
    cost_dist = 1.0 - ((detection.lead_s - car_s) / lead_car_gap);
  }

  return cost_dist;
}

// Cost related to speed of lane
double behaviors::cost_speed_lane(lead_car& detection, double& optimum_speed) {
  double cost_speed;
  double lead_speed = detection.velocity * 2.237;

  if (lead_speed >= optimum_speed) {
    cost_speed = 0.0;
  } else if (lead_speed < optimum_speed) {
    cost_speed = (optimum_speed - lead_speed) / optimum_speed;
  }

  return cost_speed;
}

// Cost related to changing lanes and highway boundaries
double behaviors::cost_change_lane(int& desired_lane, int& current_lane) {
  double cost_lane;

  if (desired_lane < 0 || desired_lane > 2) {
    cost_lane = 1.0;
  } else if (current_lane == 0 && desired_lane == 1) {
    cost_lane = 0.0;
  } else if (current_lane == 0 && desired_lane == 2) {
    cost_lane = 0.8;
  } else if ((current_lane == 1) && ((desired_lane == 0 || desired_lane == 2))) {
    cost_lane = 0.0;
  } else if (current_lane == 2 && desired_lane == 1) {
    cost_lane = 0.0;
  } else if (current_lane == 2 && desired_lane == 0) {
    cost_lane = 0.8;
  } else {
    cost_lane = 1.0;
  }

  return cost_lane;
}

// Cruise control with forward collision avoidance
void behaviors::auto_cruise(lead_car& detection, double& car_vel) {
  double threshold_dec = 50;
  double threshold_inc = 100;
  
  if ((detection.too_close_lead) && car_vel != 0.0) {
    if (detection.lead_s > threshold_dec) {
      car_vel -= (max_accel*0.5);
    } else if (detection.lead_s > (threshold_dec*0.5)) {
      car_vel -= (max_accel*0.3);
    } else if (detection.lead_s > (threshold_dec*0.3)) {
      car_vel -= (max_accel);
    }
  } else if (car_vel < speed_limit) {
    if (detection.lead_s > threshold_inc) {
      car_vel += (max_accel);
    } else if (detection.lead_s > (threshold_inc*0.5)) {
      car_vel += (max_accel*0.5);
    } else if (detection.lead_s > (threshold_inc*0.4)) {
      car_vel += (max_accel*0.4);
    }
  } else {
    car_vel = car_vel;
  }
}

// Auto lane change to desired lane if the cost isn't higher
void behaviors::auto_lane_change(int& desired_lane, int& car_lane) {
  double cost_lane = cost_change_lane(desired_lane, car_lane);

  if (cost_lane < 0.5) {
    if (car_lane > desired_lane) {
      car_lane -= 1;
    } else if (car_lane < desired_lane) {
      car_lane += 1;
    } else
      car_lane = car_lane;
  } else {
    car_lane = car_lane;
  }
}

// Finite State Machine for path planning
void behaviors::FiniteStateMachine(double& car_speed, int& car_lane, lead_car lane0_detect, lead_car lane1_detect, lead_car lane2_detect, double lane0_total_cost, double lane1_total_cost, double lane2_total_cost) {
  if ((car_lane == 0) || (car_lane == 1) || (car_lane == 2)) {
    if (car_speed > 0) {
      state = "Optimizing Path";
      if (car_lane == 0) {
        auto_cruise(lane0_detect, car_speed);
        if (lane0_detect.too_close_lead) {
          if (lane0_total_cost > lane1_total_cost) {
            if (lane1_detect.open_lane) {
              desired_lane = 1;
              state = "Auto Lane Change Right";
              auto_lane_change(desired_lane, car_lane);
            }
          }
        } else {
          state = "Auto Cruise";
          auto_cruise(lane0_detect, car_speed);
        }
      } else if (car_lane == 1) {
        auto_cruise(lane1_detect, car_speed);
        if (lane1_detect.too_close_lead) {
          if ((lane1_total_cost > lane0_total_cost) || (lane1_total_cost > lane2_total_cost)) {
            if (lane0_total_cost < lane2_total_cost) {
              if (lane0_detect.open_lane) {
                desired_lane = 0;
                state = "Auto Lane Change Left";
                auto_lane_change(desired_lane, car_lane);
              }
            } else if (lane0_total_cost > lane2_total_cost) {  
              if (lane2_detect.open_lane) {
                desired_lane = 2;
                state = "Auto Lane Change Right";
                auto_lane_change(desired_lane, car_lane);
              }
            }
          }
        } else {
          state = "Auto Cruise";
          auto_cruise(lane1_detect, car_speed);
        }
      } else if (car_lane == 2) {
        auto_cruise(lane2_detect, car_speed);
        if (lane2_detect.too_close_lead) {
          if (lane2_total_cost > lane1_total_cost) {
            if (lane1_detect.open_lane) {
              desired_lane = 1;
              state = "Auto Lane Change Left";
              auto_lane_change(desired_lane, car_lane);
            }
          }
        } else {
          state = "Auto Cruise";
          auto_cruise(lane2_detect, car_speed);
        }
      } 
    }
  }
}