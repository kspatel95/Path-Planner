#ifndef PATH_PLANNING_BEHAVIORS_HPP
#define PATH_PLANNING_BEHAVIORS_HPP

#include "detection.hpp"

using json = nlohmann::json;

class behaviors {
  public:
    /**
     * @brief Default Constructor
     */
    behaviors();

    /**
     * @brief Default Destructor
     */
    virtual ~behaviors();

    /**
     * @brief Cost for lane taking into account gap between lead car.
     * @param detection: Using detection from sensor fusion data.
     * @param max_gap_front: Minimum distance to keep from lead car.
     * @param car_s: S position of the car.
     * @return Cost for distance as a double.
     */
    double cost_dist_front(lead_car& detection, double& max_gap_front, double& car_s);

   /**
     * @brief Cost for lane taking into account speed based on lead car.
     * @param detection: Using detection from sensor fusion data.
     * @param optimum_speed: Most efficient speed possible.
     * @return Cost for speed as a double.
     */
    double cost_speed_lane(lead_car& detection, double& optimum_speed);

   /**
     * @brief Cost for changing lanes.
     * @param lane: Desired lane to change to.
     * @param current_lane: The current lane the car is in.
     * @return Cost for lane change as a double
     */
    double cost_change_lane(int& desired_lane, int& current_lane);

   /**
     * @brief Speed control to keep consistent speed and avoid front collision.
     * @param detection: Using detection from sensor fusion data.
     * @param car_vel: The current car speed.
     */
    void auto_cruise(lead_car& detection, double& car_vel);

   /**
     * @brief Lane change to desired lane, if available.
     * @param desired_lane: Desired lane to change to.
     * @param car_lane: The current lane the car is in.
     */
    void auto_lane_change(int& desired_lane, int& car_lane);

   /**
     * @brief The Finite State Machine is the decision maker for taking actions.
     * @param car_speed: The current speed of the car.
     * @param car_lane: The current lane of the car.
     * @param lane0_detect: Sensor fusion for lane 0.
     * @param lane1_detect: Sensor fusion for lane 1.
     * @param lane2_detect: Sensor fusion for lane 2.
     * @param lane0_total_cost: The total cost for lane 0.
     * @param lane1_total_cost: The total cost for lane 1.
     * @param lane2_total_cost: The total cost for lane 2.
     */
    void FiniteStateMachine(double& car_speed, int& car_lane, lead_car lane0_detect, lead_car lane1_detect, lead_car lane2_detect, double lane0_total_cost, double lane1_total_cost, double lane2_total_cost);

    std::string state;
    double max_accel = 0.4;
    double speed_limit = 49.5;
    int desired_lane;
};

#endif  // PATH_PLANNING_BEHAVIORS_HPP