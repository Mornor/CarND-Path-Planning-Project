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
			Car tempCar(sensorFusion[i][0],sensorFusion[i][1],sensorFusion[i][2],
						sensorFusion[i][3],sensorFusion[i][4],sensorFusion[i][5],
						sensorFusion[i][6]);
			surroundingCars.push_back(tempCar); 
		}
	}

	return surroundingCars;  
}

/*void PathPlanner::InterpolatePoints(vector<double> &waypoints_x, vector<double> &waypoints_y, vector<double> &waypoints_dx, vector<double> &waypoints_dy, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_dx, vector<double> map_waypoints_dy){
	&waypoints_x.set_points(map_waypoints_s, map_waypoints_x);
	&waypoints_y.set_points(map_waypoints_s, map_waypoints_y);
	&waypoints_dx.set_points(map_waypoints_s, map_waypoints_dx);
	&waypoints_dy.set_points(map_waypoints_s, map_waypoints_dy);
}*/

void PathPlanner::FollowLane(vector<double> &next_x_vals, vector<double> &next_y_vals, vector<double> map_wp_x, vector<double> map_wp_y, vector<double> map_wp_s, vector<double> map_wp_dx, vector<double> map_wp_dy){
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
}

/*void changeLane(){
	// Change lane in case a car is ahead of us and obstructing the way
}*/