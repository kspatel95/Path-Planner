#ifndef PATH_PLANNING_TRAJECTORY_HPP
#define PATH_PLANNING_TRAJECTORY_HPP

#include <vector>
#include "json.hpp"
#include "helpers.h"

using json = nlohmann::json;

class trajectory {
  public:
    /**
     * @brief Default Constructor
     */
    trajectory();

    /**
     * @brief Default Destructor
     */
    virtual ~trajectory();

    /**
     * @brief Initializer
     * @param car_lane: The starting lane for car.
     * @param max_vel: The max velocity the car can go.
     * @param previous_path: Previously generated trajectory.
     * @param car_x: The starting x coordinate of car.
     * @param car_y: The starting y coordinate of car.
     * @param car_yaw: The starting heading of car in degrees.
     */
    void init(int& car_lane, double& max_vel, json& previous_path, double& car_x, double& car_y, double& car_yaw);

    /**
     * @brief Find previous trajectory points, otherwise use current position.
     * @param prev_x_points: The x coordinates previously used in path.
     * @param prev_y_points: The y coordinates previously used in path.
     * @param car_x: The current x coordinate of car.
     * @param car_y: The current y coordinate of car.
     * @param car_yaw: The current heading of car in degrees.
     */
    void previous_points(json& prev_x_points, json& prev_y_points, double& car_x, double& car_y, double& car_yaw);

    /**
     * @brief Calculates the number of points and scale factor for the future trajectory.
     * @param num_points: The number of points to generate over the total distance.
     * @param car_s: The current s coordinate of car.
     * @param map_waypoints_s: The s coordinates from map waypoints.
     * @param map_waypoints_x: The x coordinates from map waypoints.
     * @param map_waypoints_y: The y coordinates from map waypoints.
     * @return An integer of the scale factor (number of points / distance).
     */
    int next_points(int num_points, double& car_s, std::vector<double>& map_waypoints_s, std::vector<double>& map_waypoints_x, std::vector<double>& map_waypoints_y);

    /**
     * @brief Converts the global coordinate points to local coordinates.
     */
    void global2local();

    /**
     * @brief Fits a spline for the trajectory utilizing previous points.
     * @param scale_factor: Scale returned from next_points.
     * @param prev_x_points: The vector of unused previous x trajectory points.
     * @param prev_y_points: The vector of unused previous y trajectory points.
     * @param next_x_vals: Vector of output x values.
     * @param next_y_vals: Vector of output y values.
     */
    void spline_trajectory(int& scale_factor, json& prev_x_points, json& prev_y_points, std::vector<double>& next_x_vals, std::vector<double>& next_y_vals);

  private:
    // Define const and parameters variables
    int ref_lane;
    double ref_vel;
    int max_points = 70;
    int max_length = 120;

    // Previous trajectory size
    int prev_size;

    // Define the car current positioning
    double ref_x;
    double ref_y;
    double ref_yaw;

    // Define vector to store x and y points
    std::vector<double> ptsx;
    std::vector<double> ptsy;
};

#endif  // PATH_PLANNING_TRAJECTORY_HPP