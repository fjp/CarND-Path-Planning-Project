#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "classifier.h"
#include "vehicle.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y, int *closest2ndWaypoint = 0)
{

    double closestLen = 100000; //large number
    double closest2ndLen = 1000000;
    int closestWaypoint = 0;

    for(int i = 0; i < maps_x.size(); i++)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x,y,map_x,map_y);
        if(dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }
        if (dist < closest2ndLen && closest2ndLen > closestLen)
        {
            closest2ndLen = dist;
            *closest2ndWaypoint = i;
        }

    }

    return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2((map_y-y),(map_x-x));

    double angle = fabs(theta-heading);
    angle = min(2*pi() - angle, angle);

    if(angle > pi()/4)
    {
        closestWaypoint++;
        if (closestWaypoint == maps_x.size())
        {
            closestWaypoint = 0;
        }
    }

    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
    int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

    int prev_wp;
    prev_wp = next_wp-1;
    if(next_wp == 0)
    {
        prev_wp  = maps_x.size()-1;
    }

    double n_x = maps_x[next_wp]-maps_x[prev_wp];
    double n_y = maps_y[next_wp]-maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;

    double frenet_d = distance(x_x,x_y,proj_x,proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000-maps_x[prev_wp];
    double center_y = 2000-maps_y[prev_wp];
    double centerToPos = distance(center_x,center_y,x_x,x_y);
    double centerToRef = distance(center_x,center_y,proj_x,proj_y);

    if(centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for(int i = 0; i < prev_wp; i++)
    {
        frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    }

    frenet_s += distance(0,0,proj_x,proj_y);

    return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
    int prev_wp = -1;

    while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
    {
        prev_wp++;
    }

    int wp2 = (prev_wp+1)%maps_x.size();

    double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s-maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
    double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

    double perp_heading = heading-pi()/2;

    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);

    return {x,y};
}


// start in lane 1
int lane = 1;

// Reference velocity
// Start at zero to avoid exceeding jerk error at the start
double ref_vel = 0.0; // mph

// Safety distance to other vehicles
double safetyGap = 30; // m

double dt = 0.02;


double SPEED_LIMIT = 49.9;
int num_lanes = 3;
double goal_s = 6945.554;
int goal_lane = 1;
double MAX_ACCEL = 10; //m/s^2

int main() {

    // Prediction Gaussian Naive Bayes
    GNB gnb = GNB();

    vector< vector<double> > X_train = gnb.Load_State("../pred_data/train_states.txt");
    vector< vector<double> > X_test  = gnb.Load_State("../pred_data/test_states.txt");
    vector< string > Y_train  = gnb.Load_Label("../pred_data/train_labels.txt");
    vector< string > Y_test   = gnb.Load_Label("../pred_data/test_labels.txt");

    cout << "X_train number of elements " << X_train.size() << endl;
    cout << "X_train element size " << X_train[0].size() << endl;
    cout << "Y_train number of elements " << Y_train.size() << endl;

    cout << "Training Gaussian Naive Bayes" << endl;
    gnb.train(X_train, Y_train);
    cout << "Training Gaussian Naive Bayes done" << endl;

    cout << "X_test number of elements " << X_test.size() << endl;
    cout << "X_test element size " << X_test[0].size() << endl;
    cout << "Y_test number of elements " << Y_test.size() << endl;

    cout << "Testing Gaussian Naive Bayes" << endl;
    int score = 0;
    for(int i = 0; i < X_test.size(); i++)
    {
        vector<double> coords = X_test[i];
        string predicted = gnb.predict(coords);
        if(predicted.compare(Y_test[i]) == 0)
        {
            score += 1;
        }
    }

    double fraction_correct = double(score) / Y_test.size();
    cout << "Testing Gaussian Naive Bayes done: " << (100*fraction_correct) << " correct" << endl;


	//vector<int> ego_config = {SPEED_LIMIT, num_lanes, goal_s, goal_lane, MAX_ACCEL};

    //Vehicle(int lane, double s, double v, double a, string state="CS");
    Vehicle ego = Vehicle(lane, 6945.554, 0.0, 0);

    ego.configure(SPEED_LIMIT, num_lanes, goal_s, goal_lane, MAX_ACCEL);
    ego.state = "KL";

    // Simulator code
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

    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
        double x;
        double y;
        double s;
        double d_x;
        double d_y;
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

    h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&ego](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
        uWS::OpCode opCode) {
            // "42" at the start of the message means there's a websocket message event.
            // The 4 signifies a websocket message
            // The 2 signifies a websocket event
            //auto sdata = string(data).substr(0, length);
            //cout << sdata << endl;
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
                        double car_v = j[1]["speed"];

                        // Previous path data given to the Planner
                        auto previous_path_x = j[1]["previous_path_x"];
                        auto previous_path_y = j[1]["previous_path_y"];
                        // Previous path's end s and d values
                        double end_path_s = j[1]["end_path_s"];
                        double end_path_d = j[1]["end_path_d"];

                        // Sensor Fusion Data, a list of all other cars on the same side of the road.
                        auto sensor_fusion = j[1]["sensor_fusion"];

                        // Store the size of the previous path to create a new transition
                        int prev_size = previous_path_x.size();


                        // Sensor Fusion
                        // Find other vehicle's in Frenet frame
                        // Set the ego s coordinate to the s coordinate of the previous path
                        if (prev_size > 0)
                        {
                            car_s = end_path_s;
                        }

                        bool too_close = false;
                        int horizon;

                        // Map sourrounding vehicles with their corresponding object id
                        map<int, Vehicle> vehicles;
                        // Predicted trajectories of surrounding objects (other vehicles)
                        // Object id, predicted trajectory of this vehicle with object id
                        map<int, vector<Vehicle> > predictions = {};

                        // Go through sensor fusion list of vehicles around us
                        // find ref_v to use
                        for (int i = 0; i < sensor_fusion.size(); i++)
                        {
                            // Frenet d coordinate of the current vehicle
                            double object_s = sensor_fusion[i][5];
                            // Only consider surrounding vehicles that are close to us
                            if (fabs(object_s - car_s) > safetyGap)
                            {
                                continue;
                            }
                            // Frenet d coordinate of the current vehicle
                            double object_d = sensor_fusion[i][6];

                            int object_id = sensor_fusion[i][0];
                            double object_x = sensor_fusion[i][1];
                            double object_y = sensor_fusion[i][2];
                            double object_vx = sensor_fusion[i][3];
                            double object_vy = sensor_fusion[i][4];
                            double object_v_abs = sqrt(object_vx*object_vx + object_vy*object_vy);


                            int closest2ndWaypoint = 0;
                            int closestWaypoint = ClosestWaypoint(object_x, object_y, map_waypoints_x, map_waypoints_y, &closest2ndWaypoint);


                            double waypoint0_x = map_waypoints_x[closestWaypoint];
                            double waypoint1_x = map_waypoints_x[closest2ndWaypoint];
                            double waypoint0_y = map_waypoints_y[closestWaypoint];
                            double waypoint1_y = map_waypoints_y[closest2ndWaypoint];

                            // Calculate its velocity in s and d direction
                            // https://discussions.udacity.com/t/compute-s-speed-of-sensor-fusion-data/326620
                            double object_heading  = atan2(object_vy, object_vx);
                            double lane_heading = atan2((waypoint1_y - waypoint0_y), (waypoint1_x - waypoint0_x));
                            double delta_theta = object_heading - lane_heading;


                            double object_v_s = object_v_abs * cos(delta_theta);
                            double object_v_d = object_v_abs * sin(delta_theta);

                            // Assume a constant acceleration of the vehicle nearby
                            double object_a = 0;
                            // Associate the vehicle to a lane -> 0, 1, 2
                            int object_lane = object_d / 4;
                            // Create the vehicle object
                            Vehicle object = Vehicle(object_lane, object_s, object_v_abs, object_a, "CS");
                            std::cout << "Vehicle with ID: " << object_id << " is in lane: " << object_lane << " driving: " << object_v_abs << " mph" << " with state: " << object.state << std::endl;
                            // Add the vehicle to the vehicles map
                            vehicles.insert(std::pair<int,Vehicle>(object_id, object));
                            //std::cout << "Added vehicle with ID: " << object_id << std::endl;

                            // Predict the trajectory of the surrounding vehicle and store it for later use
                            horizon = 3/dt;
                            vector<Vehicle> preds = object.generate_predictions(horizon);
                            predictions[object_id] = preds;

                            // check if the car is in my lane
                            if (object_d < (2+4*lane+2) && object_d > (2+4*lane-2))
                            {
                                // Because it is in my lane, check how close this car is
                                // project the s value of the vehicle out into the future if previous trajectory points are used
                                object_s += ((double)prev_size*.02*object_v_abs);
                                // check if future s values greater than our car's future s value
                                // car is in front of us and s gap is smaller than 30 m
                                if ((object_s > car_s) && ((object_s - car_s) < safetyGap))
                                {
                                    // Do some logic here, lower reference velocity so we don't crash into the car infront of us
                                    // Or set flag to try to change lanes
                                    //ref_vel = 29.5; // mph
                                    too_close = true;
                                }
                            }
                        }

                        std::cout << "Number of surrounding vehicles: " << vehicles.size() << std::endl;

                        if (too_close)
                        {
                            // deaccelerate with 5 m/s^2
                            // (0.224 m/h * 2.24) m/s / 0.02 s = 5 m/s^2
                            ref_vel -= .224;
                        }
                        else if (ref_vel < 49.5)
                        {
                            // accelerate with 5 m/s^2
                            ref_vel += .224;
                        }
                        ego.update_state(ego.state, lane, car_s, car_v, 0);


                        if (ego.state.compare("PLCL") == 0)
                        {
                            ego.state = "LCL";
                            lane--;
                            ego.update_state(ego.state, lane, car_s, ref_vel, 0);
                        }
                        else if(ego.state.compare("PLCR") == 0)
                        {
                            ego.state = "LCR";
                            lane++;
                            ego.update_state(ego.state, lane, car_s, ref_vel, 0);
                        }


                        ego.choose_next_state(predictions, horizon);
                        std::cout << "============================================" << std::endl;
                        std::cout << "Next ego state: " << ego.state << std::endl;
        	            //it->second.realize_next_state(trajectory);
                        std::cout << "============================================" << std::endl;




                        // Create a list of widely spaced (x, y) waypoints, evenly space at 30 m
                        // Later we will interpolate these waypoints with a spline and fill it in with more points that control speed
                        vector<double> ptsx;
                        vector<double> ptsy;

                        // Reference states x, y, yaw
                        // Either we will reference the starting point as where the car is or at the previous paths end points
                        double ref_x = car_x;
                        double ref_y = car_y;
                        double ref_yaw = deg2rad(car_yaw);


                        // if previous size is almost empty, use the car as starting reference
                        if (prev_size < 2)
                        {
                            // Use two points that make the path tangent to the car
                            // Take the current car position car_x and go back using its yaw angle
                            double prev_car_x = car_x - cos(car_yaw);
                            double prev_car_y = car_y - sin(car_yaw);

                            ptsx.push_back(prev_car_x);
                            ptsx.push_back(car_x);

                            ptsy.push_back(prev_car_y);
                            ptsy.push_back(car_y);
                        }
                        else // Use the previous path's end point as starting reference
                        {
                            // Redefine reference state as previous path end point
                            ref_x = previous_path_x[prev_size-1];
                            ref_y = previous_path_y[prev_size-1];

                            double ref_x_prev = previous_path_x[prev_size-2];
                            double ref_y_prev = previous_path_y[prev_size-2];
                            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

                            // Use two points that make the path tangent to the previous path's end point
                            ptsx.push_back(ref_x_prev);
                            ptsx.push_back(ref_x);

                            ptsy.push_back(ref_y_prev);
                            ptsy.push_back(ref_y);
                        }


                        // In Frenet add evenly 30 m spaced points ahead of the starting reference
                        vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                        vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
                        vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

                        ptsx.push_back(next_wp0[0]);
                        ptsx.push_back(next_wp1[0]);
                        ptsx.push_back(next_wp2[0]);


                        ptsy.push_back(next_wp0[1]);
                        ptsy.push_back(next_wp1[1]);
                        ptsy.push_back(next_wp2[1]);



                        // Transform to local car's coordinates.
                        // Shift the last point of the previous path to the origin of the car and turn the path to get a zero degree angle
                        for (int i = 0; i < ptsx.size(); i++)
                        {
                            // Shift car reference angle to 0 degrees
                            double shift_x = ptsx[i]-ref_x;
                            double shift_y = ptsy[i]-ref_y;

                            ptsx[i] = shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw);
                            ptsy[i] = shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw);
                        }



                        // Create a spline
                        tk::spline s;

                        // Set (x,y) points to the spline
                        s.set_points(ptsx,ptsy);

                        // Define the actual (x,y) points we will use for the planner
                        vector<double> next_x_vals;
                        vector<double> next_y_vals;

                        // Start with all of the previous path points from last time
                        for (int i = 0; i < previous_path_x.size(); i++)
                        {
                            next_x_vals.push_back(previous_path_x[i]);
                            next_y_vals.push_back(previous_path_y[i]);
                        }

                        // Calculate how to break up spline points so that we travel at our desired reference velocity
                        double target_x = 30.0;
                        double target_y = s(target_x);
                        double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));

                        double x_add_on = 0;


                        // Calculate number of points N to get the desired target speed
                        // target_dist = 30m
                        // 0.02 seconds per update cycle
                        // convert reference velocity from mph to m/s by diving ref_vel with 2.24
                        double N = (target_dist/(.02*ref_vel/2.24));

                        // Fill up the rest of our path planner after fillin it with previous points, here we will always output 50 points
                        for (int i = 1; i <= 50 - previous_path_x.size(); i++)
                        {
                            double x_point = x_add_on + (target_x)/N;
                            double y_point = s(x_point);


                            x_add_on = x_point;

                            double x_ref = x_point;
                            double y_ref = y_point;

                            // Rotate back to normal after rotating it earlier
                            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
                            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

                            x_point += ref_x;
                            y_point += ref_y;

                            next_x_vals.push_back(x_point);
                            next_y_vals.push_back(y_point);
                        }



                        json msgJson;

                        // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds


                        msgJson["next_x"] = next_x_vals;
                        msgJson["next_y"] = next_y_vals;


                        auto msg = "42[\"control\","+ msgJson.dump()+"]";

                        //this_thread::sleep_for(chrono::milliseconds(1000));
                        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                    }
                } else {
                    // Manual driving
                    std::string msg = "42[\"manual\",{}]";
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            }
        });

        // We don't need this since we're not using HTTP but if it's removed the
        // program
        // doesn't compile :-(
        h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
            size_t, size_t) {
                const std::string s = "<h1>Hello world!</h1>";
                if (req.getUrl().valueLength == 1) {
                    res->end(s.data(), s.length());
                } else {
                    // i guess this should be done more gracefully?
                    res->end(nullptr, 0);
                }
            });

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
