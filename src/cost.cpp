#include "cost.h"
#include "vehicle.h"

#include <functional>
#include <iterator>
#include <map>
#include <math.h>


const float ACHIEVE_DAT_GOAL = pow(10, 1);
const float EFFICIENCY = pow(10, 2);
const float JERK = pow(40,1);

float distance_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, float> & data){
  float cost;
  float distance = data["distance_to_goal"];
  if(distance > 0){
    cost = 1 - 2 * exp(-(abs(2.0*vehicle.target_lane - data["intended_lane"] - data["final_lane"]) / distance));
  }
  else{
    cost = 1;
  }
  return cost;
}

float inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, float> & data){
  float proposed_speed_intended = lane_speed(trajectory[1], predictions, data["intended lane"]);
  if(proposed_speed_intended < 0){
    proposed_speed_intended = vehicle.target_speed;
  }
  float proposed_speed_final = lane_speed(trajectory[1], predictions, data["final_lane"]);
  if(proposed_speed_final < 0 ){
    proposed_speed_final = vehicle.target_speed;
  }
  float cost = (2.0*vehicle.target_speed - proposed_speed_intended - proposed_speed_final) / vehicle.target_speed;

  return cost;
}


float jerk_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, float> & data){

  float s_ = trajectory[1].s_new;
  //cout << s_ << endl;
  return 0.0;
}


/*
  vector<Vehicle> s_dot_traj = differentiate(trajectory[1]);
  vector<Vehicle> s_ddot_traj = differentiate(s_dot_traj);
  vector<Vehicle> s_dddot_traj = differentiate(s_ddot_traj);
  for (int i = 0; i < s_ddot_traj.size(); i++){
    if (s_ddot_traj[i] > MAX_JERK){
      return 1;
    }
  }
  return 0;
}
*/

vector<double> differentiate(vector<double> traj){
  vector<double> dif_helper;
  for(int i = 1; i < traj.size(); i++){
    dif_helper.push_back((traj[i] - traj[i-1]) / 0.20);
  }
  return dif_helper;
}



float lane_speed(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, int lane){
  return vehicle.cost_;
}

/*
  for(map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it){
    int key = it->first;
    Vehicle vehicle = it->second[0];
    if (vehicle.lane == lane && key != -1) {
      return vehicle.car_speed;
    }
  }
  return -1.0;
}
*/


float calculate_cost(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, const vector<Vehicle> & trajectory){
  /*
  Sum weighted costs
  */
  map<string, float> trajectory_data = get_helper_data(vehicle, trajectory, predictions);
  float cost = 0.0;

  vector< function<float(const Vehicle &, const vector<Vehicle> &, const map<int, vector<Vehicle>> &, map<string, float> &)>> cf_list = { distance_cost, inefficiency_cost, jerk_cost };
  vector<float> weight_list = { ACHIEVE_DAT_GOAL, EFFICIENCY, JERK };

  //  printf("vehicle state:%s, trajectory state:%s, ", vehicle.state.c_str(), trajectory[1].state.c_str());
  for (int i = 0; i < cf_list.size(); i++) {
      float new_cost = weight_list[i] * cf_list[i](vehicle, trajectory, predictions, trajectory_data);
  /*
      if (i == 0) printf("distance cost:%f, ", new_cost);
      if (i == 1) printf("inefficiency cost:%f\n\n", new_cost);
  */
      cost += new_cost;
  }
return cost;
}

map<string, float> get_helper_data(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions){
  /*
  Helper data to use in cost functions to handle intended_lane and final_lane etc.
  */
  map<string, float> trajectory_data;
  Vehicle trajectory_last = trajectory[1];
  float intended_lane;

  if(trajectory_last.state.compare("PLCL") == 0){
    intended_lane = trajectory_last.lane - 1;
  }
  else if(trajectory_last.state.compare("PLCR") == 0){
    intended_lane = trajectory_last.lane + 1;
  }
  else {
    intended_lane = trajectory_last.lane;
  }

  float distance_to_goal = vehicle.target_s - trajectory_last.s_new;
  float final_lane = trajectory_last.lane;
  trajectory_data["intended_lane"] = intended_lane;
  trajectory_data["final_lane"] = final_lane;
  trajectory_data["distance_to_goal"] = distance_to_goal;

  return trajectory_data;
}
