#ifndef PATH_PLANNING_READOUT_HPP
#define PATH_PLANNING_READOUT_HPP

#include <iostream>

using namespace std;

// Push a readout of data output to STDOUT
inline void readout(int& car_lane, double& car_vel, string& state, double& lead_car_dist, double& lead_car_speed, bool& lead_too_close, double& lane0_cost, double& lane1_cost, double& lane2_cost)
{
    cout << "/////////////////////////////////////" << endl;
    cout << "| CAR ------------------------------" << endl;
    cout << "||| Car Lane       : " << car_lane << endl;
    cout << "||| Car Speed      : " << car_vel << endl;
    cout << "||| Path Planner   : " << state << endl;
    cout << "| LEAD -----------------------------" << endl;
    cout << "||| Lead Distance  : " << lead_car_dist << endl;
    cout << "||| Lead Speed     : " << lead_car_speed << endl;
    cout << "||| Too Close      : " << lead_too_close << endl;
    cout << "| LANES ----------------------------" << endl;
    cout << "||| Lane 0 Cost    : " << lane0_cost << endl;
    cout << "||| Lane 1 Cost    : " << lane1_cost << endl;
    cout << "||| Lane 2 Cost    : " << lane2_cost << endl;
}

#endif  // PATH_PLANNING_READOUT_HPP