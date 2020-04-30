#ifndef PATH_PLANNING_DETECTION_HPP
#define PATH_PLANNING_DETECTION_HPP

#include <vector>
#include "json.hpp"

using json = nlohmann::json;

struct lead_car {
    bool open_lane;        // True if lane has opening else False
    bool too_close_lead;   // True if too close to lead car else False
    bool too_close_trail;  // True if too close to trailing car else False
    int lane_number;       // 0 = Left, 1 = Center, 2 = Right
    double velocity;       // Magnitude of vx and vy
    double lead_s;         // Predicted s coord of lead car
    double trail_s;        // Predicted s coord of trailing car
};

class detection {
  public:
    /**
     * @brief Default Constructor
     */
    detection();

    /**
     * @brief Default Destructor
     */
    virtual ~detection();

    /**
     * @brief Predict the velocity and s coordinate for the car.
     * @param vx: Longitudinal velocity in m/s of the object.
     * @param vy: Lateral velocity in m/s of the object.
     * @param prev_size: The size of the previous trajectory.
     * @return A vector with the velocity and predicted s [velocity, predicted_s].
     */
    std::vector<double> check_prediction(double vx, double vy, double prev_size);

    /**
     * @brief Used to determine if a FV is in center lane and if so return its characteristics.
     * @param sensor_fusion: Data coming from sensor fusion.
     * @param lane: A lane number on the given road, 0 = Left, 1 = Center, 2 = Right.
     * @param previous_path: A json vector of the previous trajectory.
     * @param car_s: The car's s coordinate.
     * @param end_traj_s: The end of the previous trajectory s coordinate.
     * @return An update to lead_car with moving objects in lane.
     */
    lead_car check_lane(json& sensor_fusion, int& lane, json& previous_path, double& car_s, double& end_traj_s);

    double lead_car_gap = 25.0;
    double trail_car_gap = 40.0;
};

#endif  // PATH_PLANNING_DETECTION_HPP