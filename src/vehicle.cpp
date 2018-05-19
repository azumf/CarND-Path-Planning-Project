#define _USE_MATH_DEFINES
#include <math.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <iostream>
#include <map>
#include <string>

#include "cost.h"
#include "spline.h"
#include "vehicle.h"

// const. expression for Pi
constexpr double pi() { return M_PI; }
// convert deg to rad and vice versa
double deg2rad(double x) {
  return x * pi() / 180;
}
double rad2deg(double x) {
  return x * 180 / pi();
}
// calculate distance
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
}


int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
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

Vehicle::Vehicle() {}
Vehicle::Vehicle(int lane, float s, float v, float a, string state, float cost_){
  this->lane = lane;
  this->s_new = s;
  this->v_new = v;
  this->a_new = a;
  this->state = state;
  this->cost_ = cost_;
}

Vehicle::~Vehicle() {}

void Vehicle::set_params(double ref_vel, double target_speed, int lanes_free, double target_s, int target_lane, double max_acceleration){
  this->ref_vel = ref_vel;
  this->target_speed = target_speed;
  this->lanes_free = lanes_free;
  this->target_s = target_s;
  this->target_lane = target_lane;
  this->max_acceleration = max_acceleration;
}

void Vehicle::set_vehicle(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed){
  this->car_x = car_x;
  this->car_y = car_y;
  this->car_s = car_s;
  this->car_d = car_d;
  this->car_yaw = car_yaw;
  this->car_speed = car_speed;
}

void Vehicle::set_prev_path(const vector<double> &path_x, const vector<double> &path_y, const double &path_s, const double &path_d){
  this->prev_path_x = path_x;
  this->prev_path_y = path_y;
  this->end_path_s = path_s;
  this->end_path_d = path_d;
}

void Vehicle::set_map(const vector<double> &wp_x, const vector<double> &wp_y, const vector<double> &wp_s, const vector<double> &wp_dx, const vector<double> &wp_dy){
  this->map_wp_x = wp_x;
  this->map_wp_y = wp_y;
  this->map_wp_s = wp_s;
  this->map_wp_dx = wp_dx;
  this->map_wp_dy = wp_dy;
}

void Vehicle::set_sensor_fusion(const vector<SensorFusion> &sensor_fusion){
  this->sensor_fusion = sensor_fusion;
}

float Vehicle::get_position(int t){
  int prev_size = prev_path_x.size();
  return this->s_new + (float)prev_size * .02 * this->v_new*t + this->a_new*t*t / 2.0;
}

vector<Vehicle> Vehicle::generate_predictions(int horizon){
  /*
  Function generates predictions for other vehicles to generate
  trajectories for ego vehicle
  */
  vector<Vehicle> predictions;
  for(int i = 0; i < horizon; i++){
    float next_s = get_position(i);
    float next_v = 0.0;
    if(i < horizon - 1){
      next_v = get_position(i+1) - s_new;
    }
    predictions.push_back(Vehicle(this->lane, next_s, next_v, 0));
  }
  return predictions;
}

vector<string> Vehicle::successor_states(){
  /*
  Provides next states with respect to the current state
  Lane changes will transition back to keep-lane state
  */
  vector<string> states;
  states.push_back("KL");
  string state = this->state;

  if(state.compare("KL") == 0){
    if (lane == 0){ states.push_back("PLCR"); }
    else if (lane == lanes_free - 1){ states.push_back("PLCL"); }
    else {
      states.push_back("PLCL");
      states.push_back("PLCR");
    }
  }
  else if(state.compare("PLCL") == 0){
    if (lane != 0){
      states.push_back("PLCL");
      states.push_back("LCL");
    }
  }
  else if(state.compare("PLCR") == 0){
    if (lane != lanes_free - 1){
      states.push_back("PLCR");
      states.push_back("LCR");
    }
  }
  return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state, map<int, vector<Vehicle>> predictions){
  /*
  Generate trajectory with given state
  */
  vector<Vehicle> trajectory;
  if(state.compare("CS") == 0){
    trajectory = constant_speed_trajectory();
  }
  else if(state.compare("KL") == 0){
    trajectory = keep_lane_trajectory(predictions);
  }
  else if(state.compare("LCL") == 0 || state.compare("LCR") == 0){
    trajectory = lane_change_trajectory(state, predictions);
  }
  else if(state.compare("PLCL")  == 0 || state.compare("PLCR") == 0){
    trajectory = prep_lane_change_trajectory(state, predictions);
  }
  return trajectory;
}

bool Vehicle::is_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle){

  int min_s = this->target_s;
  bool vehicle_ahead = false;
  Vehicle temp_vehicle;
  for(map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it){
    temp_vehicle = it->second[1]; // try second[1] as well
    if(temp_vehicle.lane == lane && temp_vehicle.s_new > this->car_s && temp_vehicle.s_new < min_s){
      if((temp_vehicle.s_new - this->car_s) < this->buffer){
        min_s = temp_vehicle.s_new;
        rVehicle = temp_vehicle;
        vehicle_ahead = true;
      }
    }
  }
  return vehicle_ahead;
}

bool Vehicle::is_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle){
  int max_s = -1;
  bool vehicle_behind = false;
  Vehicle temp_vehicle;
  for(map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it){
    temp_vehicle = it->second[1]; // try second[1] as well
    if(temp_vehicle.lane == lane && temp_vehicle.s_new < this->car_s && temp_vehicle.s_new > max_s){
      if ((this->car_s - temp_vehicle.s_new) < this->buffer/6){
        max_s = temp_vehicle.s_new;
        rVehicle = temp_vehicle;
        vehicle_behind = true;
      }
    }
  }
  return vehicle_behind;
}

vector<float> Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int lane){
  /*
  Next timestep kinematics in terms of position, speed and acceleration for
  a given lane.
  */
  float max_speed_acc_limit = this->v_new + this->max_acceleration;
  float max_speed_deacc_limit = this->v_new - this->max_acceleration;
  float new_pos;
  float new_speed;
  float new_acc;

  Vehicle vehicle_ahead;
  Vehicle vehicle_behind;

  if(is_vehicle_ahead(predictions, lane, vehicle_ahead)){
    if(is_vehicle_behind(predictions, lane, vehicle_behind)){
      new_speed = min(min(vehicle_ahead.v_new, max_speed_acc_limit), this->target_speed);
    }
    else{
      float v_distance = vehicle_ahead.s_new - this->car_s;
      float max_speed_in_front = (v_distance - this->buffer) + vehicle_ahead.v_new - 0.5*(this->a_new);
      if(v_distance > this->buffer / 3.0){
        max_speed_in_front = max(max_speed_in_front, max_speed_deacc_limit);
      }
      new_speed = min(min(max_speed_in_front, max_speed_acc_limit), this->target_speed);
    }
  }
  else{
    new_speed = min(max_speed_acc_limit, this->target_speed);
  }
  new_acc = new_speed - this->v_new;
  new_pos = this->s_new + this->prev_path_x.size()*.02*new_speed + new_acc / 2.0;
  return {new_pos, new_speed, new_acc};
}

void Vehicle::choose_next_state(map<int, vector<Vehicle>> predictions, vector<double> &n_x_vals, vector<double> &n_y_vals){
  s_new = car_s;
  // from mph to m/s
  v_new = ref_vel/2.24;
  a_new = 0;
  int prev_size = prev_path_x.size();
  if(prev_size > 0){
    car_s = end_path_s;
  }

  vector<string> states = successor_states();
  float cost;
  vector<float> costs;
  vector<string> final_states;
  vector<vector<Vehicle>> final_trajectories;

  // iterator
  for(vector<string>::iterator it = states.begin(); it != states.end(); ++it){
    vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
    if(trajectory.size() != 0){
      cost = calculate_cost(*this, predictions, trajectory);
      costs.push_back(cost);
      final_trajectories.push_back(trajectory);
    }
  }
  vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
  int best_idx = distance(begin(costs), best_cost);
  realize_next_state(final_trajectories[best_idx]);
  string f_state = final_trajectories[best_idx][1].state;
  int f_lane = final_trajectories[best_idx][1].lane;

  generate_path_points(n_x_vals, n_y_vals);
}

void Vehicle::generate_path_points(vector<double> &n_x_vals, vector<double> &n_y_vals){
  int prev_size = prev_path_x.size();
  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);

  if (prev_size < 2){
      double prev_car_x = car_x - cos(car_yaw);
      double prev_car_y = car_y - sin(car_yaw);

      ptsx.push_back(prev_car_x);
      ptsx.push_back(car_x);

      ptsy.push_back(prev_car_y);
      ptsy.push_back(car_y);
  }
  else{
      // Redefine reference state as previous path end point
      ref_x = prev_path_x[prev_size - 1];
      ref_y = prev_path_y[prev_size - 1];
      double ref_x_prev = prev_path_x[prev_size - 2];
      double ref_y_prev = prev_path_y[prev_size - 2];
      ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
      // Use two points that make the path tangent to the previous path's end point
      ptsx.push_back(ref_x_prev);
      ptsx.push_back(ref_x);

      ptsy.push_back(ref_y_prev);
      ptsy.push_back(ref_y);
  }
  // In Frenet add evely 30m spaced points ahead of the staring reference
  vector<double> next_wp0 = getXY(car_s + 30, (2 + 4 * lane), map_wp_s, map_wp_x, map_wp_y);
  vector<double> next_wp1 = getXY(car_s + 60, (2 + 4 * lane), map_wp_s, map_wp_x, map_wp_y);
  vector<double> next_wp2 = getXY(car_s + 90, (2 + 4 * lane), map_wp_s, map_wp_x, map_wp_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  for (int i = 0; i < ptsx.size(); i++)
  {
      //shift car reference angle to 0 degrees
      double shift_x = ptsx[i] - ref_x;
      double shift_y = ptsy[i] - ref_y;

      ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y*sin(0 - ref_yaw));
      ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y*cos(0 - ref_yaw));
  }
  // create a spline
  tk::spline s;
  // set (x,y) points to the spline
  s.set_points(ptsx, ptsy);
  //Define the actual (x,y) points we will use for the planner
  //            vector<double> next_x_vals;
  //            vector<double> next_y_vals;
  //Start with all of the previous path points from last time
  for (int i = 0; i < prev_path_x.size(); i++)
  {
      n_x_vals.push_back(prev_path_x[i]);
      n_y_vals.push_back(prev_path_y[i]);
  }
  //Calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
  double x_add_on = 0;

  //Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
  for (int i = 1; i <= 50 - prev_path_x.size(); i++) {

      double N = (target_dist / (.02*ref_vel / 2.24));
      double x_point = x_add_on + (target_x) / N;
      double y_point = s(x_point);

      x_add_on = x_point;
      double x_ref = x_point;
      double y_ref = y_point;
      // rotate back to normal after rotating it earlier
      x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
      y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

      x_point += ref_x;
      y_point += ref_y;

      n_x_vals.push_back(x_point);
      n_y_vals.push_back(y_point);
    }
}

// add trajectories
vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Generate a lane change trajectory.
    */
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory;
    Vehicle next_lane_vehicle;
    //Check if a lane change is possible (check if another vehicle occupies that spot).
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        next_lane_vehicle = it->second[1];
        if (next_lane_vehicle.lane == new_lane)
        {
            if ((abs(next_lane_vehicle.s_new - this->car_s) < 0.5*this->buffer)) {
                //If lane change is not possible, return empty trajectory.
                return trajectory;
            }
        }
    }
    trajectory.push_back(Vehicle(this->lane, this->s_new, this->v_new, this->a_new, this->state));
    vector<float> kinematics = get_kinematics(predictions, new_lane);
    trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2], state, kinematics[1]));
    return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Generate a trajectory preparing for a lane change.
    */
    float new_s;
    float new_v;
    float new_a;
    float v_cost;
    Vehicle vehicle_behind;
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory = { Vehicle(this->lane, this->s_new, this->v_new, this->a_new, this->state) };
    vector<float> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);

    if (is_vehicle_behind(predictions, this->lane, vehicle_behind)) {
        //Keep speed of current lane so as not to collide with car behind.
        new_s = curr_lane_new_kinematics[0];
        new_v = curr_lane_new_kinematics[1];
        new_a = curr_lane_new_kinematics[2];
        v_cost = new_v;
    }
    else {
        vector<float> best_kinematics;
        vector<float> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
        //Choose kinematics with lowest velocity.
        if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
            best_kinematics = next_lane_new_kinematics;
        }
        else {
            best_kinematics = curr_lane_new_kinematics;
        }
        new_s = best_kinematics[0];
        new_v = best_kinematics[1];
        new_a = best_kinematics[2];
        v_cost = (curr_lane_new_kinematics[1] + next_lane_new_kinematics[1]) / 2;
    }

    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, state, v_cost));
    return trajectory;
}

vector<Vehicle> Vehicle::constant_speed_trajectory() {
    /*
    Generate a constant speed trajectory.
    */
    float next_pos = end_path_s;
    vector<Vehicle> trajectory = { Vehicle(this->lane, this->s_new, this->v_new, this->a_new, this->state),
        Vehicle(this->lane, next_pos, this->v_new, 0, this->state, this->v_new) };
    return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> predictions) {
    /*
    Generate a keep lane trajectory.
    */
    vector<Vehicle> trajectory = { Vehicle(lane, this->s_new, this->v_new, this->a_new, state) };
    vector<float> kinematics = get_kinematics(predictions, this->lane);
    float new_s = kinematics[0];
    float new_v = kinematics[1];
    float new_a = kinematics[2];
    trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, "KL", new_v));
    return trajectory;
}


void Vehicle::realize_next_state(vector<Vehicle> trajectory){
  Vehicle next_state = trajectory[1];
  this->state = next_state.state;
  this->lane = next_state.lane;
  this->s_new = next_state.s_new;
  this->v_new = next_state.v_new;
  this->a_new = next_state.a_new;
  this->ref_vel = this->v_new*2.24;
}

void Vehicle::increment(int dt = 1) {
    this->s_new = get_position(dt);
}
