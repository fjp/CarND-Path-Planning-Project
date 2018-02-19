#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
public:

  map<string, int> lane_direction = {{"PLCL", -1}, {"LCL", -1}, {"LCR", 1}, {"PLCR", 1}};

  struct collider{

    bool collision ; // is there a collision?
    int  time; // time collision happens

  };

  int preferred_buffer = 6; // impacts "keep ane" behavior.

  int lane;

  double s;

  double v;

  double a;

  double target_speed;

  int lanes_available;

  double max_acceleration;

  int goal_lane;

  double goal_s;

  string state;

  double dt = 0.02;

  /**
  * Constructor
  */
  Vehicle();
  Vehicle(int lane, double s, double v, double a, string state="CS");

  /**
  * Destructor
  */
  virtual ~Vehicle();

  /*
    Return the best (lowest cost) trajectory corresponding to the next state.

    INPUT: A predictions map. This is a map of vehicle id keys with predicted
        vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
        the vehicle at the current timestep and one timestep in the future.
    OUTPUT: The best (lowest cost) trajectory corresponding to the next ego vehicle state.

    */
  vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> predictions, int horizon);

  /*
    Provides the possible next states given the current state for the FSM
    discussed in the course, with the exception that lane changes happen
    instantaneously, so LCL and LCR can only transition back to KL.
    */
  vector<string> successor_states();

  /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
  vector<Vehicle> generate_trajectory(string state, map<int, vector<Vehicle>> predictions, int horizon);

  /*
    Gets next timestep kinematics (position, velocity, acceleration)
    for a given lane. Tries to choose the maximum velocity and acceleration,
    given other vehicle positions and accel/velocity constraints.
    */
  vector<double> get_kinematics(map<int, vector<Vehicle>> predictions, int lane);

  /*
    Generate a constant speed trajectory.
    */
  vector<Vehicle> constant_speed_trajectory();

  /*
    Generate a keep lane trajectory.
    */
  vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> predictions, int horizon);

  /*
    Generate a trajectory preparing for a lane change.
    */
  vector<Vehicle> prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

  /*
    Generate a lane change trajectory.
    */
  vector<Vehicle> lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

  void increment(double dt);

  double position_at(double t);

  /*
    Returns true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
  bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

  /*
    Returns true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
  bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

  vector<Vehicle> generate_predictions(int horizon=2);

  // TODO not needed
  void realize_next_state(vector<Vehicle> trajectory);

  void update_state(string state, int lane, double s, double v, double a);

  /*
  Called by simulator before simulation begins. Sets various
  parameters which will impact the ego vehicle.
  */
  void configure(double target_speed, int num_lanes, double goal_s, int goal_lane, double max_acceleration);

};

#endif
