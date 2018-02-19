#ifndef COST_H
#define COST_H
#include "vehicle.h"

using namespace std;


/*
Sum weighted cost functions to get total cost for trajectory.
*/
double calculate_cost(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, const vector<Vehicle> & trajectory);

/*
Cost increases based on distance of intended lane (for planning a lane change) and final lane of trajectory.
Cost of being out of goal lane also becomes larger as vehicle approaches goal distance.
*/
double goal_distance_cost(const Vehicle & vehicle,  const vector<Vehicle> & trajectory,  const map<int, vector<Vehicle>> & predictions, map<string, double> & data);

/*
Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than vehicle's target speed.
*/
double inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, double> & data);

/*
All non ego vehicles in a lane have the same speed, so to get the speed limit for a lane,
we can just find one vehicle in that lane.
*/
double lane_speed(const map<int, vector<Vehicle>> & predictions, int lane);

map<string, double> get_helper_data(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions);

#endif
