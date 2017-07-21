#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "PathPlanner.h"

using namespace std;

#define LANE_WIDTH 4 // Each lane is 4 meters

// Init the PathPlanner
PathPlanner::~PathPlanner() {}
Lane::~Lane() {}

// Maintain a list of all the cars surrounding the one we drive, updated for each timesteps
vector<Car> PathPlanner::MaintainListSurroundingCars(vector<vector<float>> sensorFusion){
	int n_surroundingCars = sensorFusion.size();  
	vector<Car> surroundingCars;
	if(n_surroundingCars > 0){
		for(int i = 0; i < n_surroundingCars ; i++){
			float velocity = ((sensorFusion[i][3] * sensorFusion[i][3]) + (sensorFusion[i][4] * sensorFusion[i][4]));
			Car tempCar(sensorFusion[i][0],sensorFusion[i][1],sensorFusion[i][2],
						sensorFusion[i][3],sensorFusion[i][4],sensorFusion[i][5],
						sensorFusion[i][6], velocity);
			surroundingCars.push_back(tempCar); 
		}
	}

	return surroundingCars;  
}

void PathPlanner::FollowLane(vector<double> *next_x_vals, vector<double> *next_y_vals, vector<double> prev_path_x, vector<double> prev_path_y, tk::spline waypoints_x, tk::spline waypoints_y, tk::spline waypoints_dx, tk::spline waypoints_dy, double car_x, double car_y, double car_s, double car_yaw){
	
	int pathSize = prev_path_x.size(); 
	double tmp_posx, tmp_posy, tmp_poss, tmp_angle; 
	double path_x, path_y, path_dx, path_dy; 

	// Copy old path into new path values
	for(int i = 0; i < pathSize; i++){
		next_x_vals->push_back(prev_path_x[i]);
		next_y_vals->push_back(prev_path_y[i]); 
	}

	// If no path, initialize car position to current position. Otherwise, get data (x, y, angle) from the previous position of the car. 
	if(pathSize == 0){
		tmp_posx = car_x;
		tmp_posy = car_y;
        tmp_angle = car_yaw * M_PI / 180;
        tmp_poss = car_s;
	} else {
		tmp_posx = prev_path_x[pathSize - 1];
		tmp_posy = prev_path_y[pathSize - 1]; 
		double tmp_posx_bf = prev_path_x[pathSize - 2];
		double tmp_posy_bf = prev_path_y[pathSize - 2];
		tmp_angle = atan2(tmp_posy - tmp_posy_bf, tmp_posx - tmp_posx_bf);
	}

	// Set up new path with speed at roughly 50mph
	double dist_inc = 0.5;
	for(int i = 0; i < 50 - pathSize; i++){
		tmp_poss += dist_inc; // Augment s coordinates to make the car move along the road
		path_x = waypoints_x(tmp_poss);
		path_y = waypoints_y(tmp_poss);
		path_dx = waypoints_dx(tmp_poss);
		path_dy =  waypoints_dy(tmp_poss);

		// 2 being the middle lane
		tmp_posx = path_x + path_dx * (2 + 1 * LANE_WIDTH);
 		tmp_posy = path_y + path_dy * (2 + 1 * LANE_WIDTH);

 		next_x_vals->push_back(tmp_posx);
 		next_y_vals->push_back(tmp_posy);
	}
}

/*void PathPlanner::InterpolatePoints(vector<double> &waypoints_x, vector<double> &waypoints_y, vector<double> &waypoints_dx, vector<double> &waypoints_dy, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_dx, vector<double> map_waypoints_dy){
	&waypoints_x.set_points(map_waypoints_s, map_waypoints_x);
	&waypoints_y.set_points(map_waypoints_s, map_waypoints_y);
	&waypoints_dx.set_points(map_waypoints_s, map_waypoints_dx);
	&waypoints_dy.set_points(map_waypoints_s, map_waypoints_dy);
}*/

/*void PathPlanner::FollowLane(vector<double> &next_x_vals, vector<double> &next_y_vals, vector<double> map_wp_x, vector<double> map_wp_y, vector<double> map_wp_s, vector<double> map_wp_dx, vector<double> map_wp_dy){
	// Follow a given lane at the give speed
	// Check if car ahead, and if yes, change the lane.
	// Return the next position (next_x_vals and next_y_vals of the car)

	// Define the lane
	Lane right_lane = Lane(0);
	Lane middle_lane = Lane(1);   
	Lane left_lane = Lane(2);

	// Populate the lane using waypoints data
	int nb_waypoints = map_wp_x.size(); 

	for(int i = 0; i < nb_waypoints; i++){
		// wp_x and wp_y for left lane
		left_lane.x_wp[i] = map_wp_x[i] + (map_wp_dx[i] * (1 + LANE_WIDTH));
		left_lane.y_wp[i] = map_wp_y[i] + (map_wp_dy[i] * (1 + LANE_WIDTH));

		// wp_x and wp_y for middle lane
		middle_lane.x_wp[i] = map_wp_x[i] + (map_wp_dx[i] * (2 + LANE_WIDTH)); 
		middle_lane.y_wp[i] = map_wp_y[i] + (map_wp_dy[i] * (2 + LANE_WIDTH));

		// wp_x and wp_y for right lane
		right_lane.x_wp[i] = map_wp_x[i] + (map_wp_dx[i] * (3 + LANE_WIDTH));
		right_lane.y_wp[i] = map_wp_y[i] + (map_wp_dy[i] * (3 + LANE_WIDTH));

		// wp_s for all lanes
		left_lane.s_wp[i] = map_wp_s[i]; 
		middle_lane.s_wp[i] = map_wp_s[i];
		right_lane.s_wp[i] = map_wp_s[i];
	}

	// Construct next_x_vals and next_y_vals as to follow a defined lane
	tk::spline s;
	s.set_points(middle_lane.x_wp, middle_lane.y_wp);
	double dist_inc = 0.5;
	for(int i = 0; i < 50; i++){
		next_x_vals.push_back(middle_lane.x_wp[i]+(dist_inc*i));
		next_y_vals.push_back(middle_lane.y_wp[i]+(dist_inc*i));
		//std::cout << next_x_vals[i] << std::endl; 
	}
}*/

/*void changeLane(){
	// Change lane in case a car is ahead of us and obstructing the way
}*/