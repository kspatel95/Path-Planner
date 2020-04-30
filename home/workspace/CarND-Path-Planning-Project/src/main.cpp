#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "behaviors.hpp"
#include "detection.hpp"
#include "trajectory.hpp"
#include "readout.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  
  // Set initial variable values
  int ref_lane = 1;
  double ref_vel = 3.0;

  h.onMessage([&ref_lane,&ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
               (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          // Detection
          detection d_;
          
          // Lane numbers: 0 = Left, 1 = Center, 2 = Right
          std::vector<int> lanes = {0,1,2};

          // Check lane availability for each lane
          auto lane0_detect = d_.check_lane(sensor_fusion, lanes[0], previous_path_x, car_s, end_path_s);
          auto lane1_detect = d_.check_lane(sensor_fusion, lanes[1], previous_path_x, car_s, end_path_s);
          auto lane2_detect = d_.check_lane(sensor_fusion, lanes[2], previous_path_x, car_s, end_path_s);

          // Behaviors
          behaviors b_;

          // Determine cost of each lane gap for similar s value
          double lane0_dist_cost = b_.cost_dist_front(lane0_detect, d_.lead_car_gap, car_s);
          double lane1_dist_cost = b_.cost_dist_front(lane1_detect, d_.lead_car_gap, car_s);
          double lane2_dist_cost = b_.cost_dist_front(lane2_detect, d_.lead_car_gap, car_s);
          
          // Determine cost for speed of each lane
          double lane0_speed_cost = b_.cost_speed_lane(lane0_detect, b_.speed_limit);
          double lane1_speed_cost = b_.cost_speed_lane(lane1_detect, b_.speed_limit);
          double lane2_speed_cost = b_.cost_speed_lane(lane2_detect, b_.speed_limit);

          // Calculate total cost weighing gap and speed of each lane
          double lane0_total_cost = ((lane0_dist_cost + lane0_speed_cost) / 3.0);
          double lane1_total_cost = ((lane1_dist_cost + lane1_speed_cost) / 3.0);
          double lane2_total_cost = ((lane2_dist_cost + lane2_speed_cost) / 3.0);

          // Finite State Machine
          b_.FiniteStateMachine(ref_vel, ref_lane, lane0_detect, lane1_detect, lane2_detect, lane0_total_cost, lane1_total_cost, lane2_total_cost);

          // Trajectory
          trajectory t_;
          
          // Initialize, use previous points, change coordinate system, and spline
          t_.init(ref_lane, ref_vel, previous_path_x, car_x, car_y, car_yaw);
          t_.previous_points(previous_path_x, previous_path_y, car_x, car_y, car_yaw);
          int scale = t_.next_points(3, car_s, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          t_.global2local();
          t_.spline_trajectory(scale, previous_path_x, previous_path_y, next_x_vals, next_y_vals);

          // Use lead car to get info for readout
          double lead_distance, lead_speed;
          bool lead_too_close;

          if (ref_lane == 0) {
            lead_distance = lane0_detect.lead_s - car_s;
            lead_speed = lane0_detect.velocity;
            lead_too_close = lane0_detect.too_close_lead;
          } else if (ref_lane == 1) {
            lead_distance = lane1_detect.lead_s - car_s;
            lead_speed = lane1_detect.velocity;
            lead_too_close = lane1_detect.too_close_lead;
          } else if (ref_lane == 2) {
            lead_distance = lane2_detect.lead_s - car_s;
            lead_speed = lane2_detect.velocity;
            lead_too_close = lane2_detect.too_close_lead;
          }

          // Publish diagnostic output
          readout(ref_lane, ref_vel, b_.state, lead_distance, lead_speed, lead_too_close, lane0_total_cost, lane1_total_cost, lane2_total_cost);

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}