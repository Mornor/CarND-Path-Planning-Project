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


void PathPlanner::FollowLane(int idLane, float speed, vector<float> &next_x_vals, vector<float> &next_y_vals, vector<double> map_wp_x, vector<double> map_wp_y, vector<double> map_wp_s, vector<double> map_wp_dx, vector<double> map_wp_dy){
	// Follow a given lane at the give speed
	// Check if car ahead, and if yes, change the lane.
	// Return the next position (next_x_vals and next_y_vals of the car)

	// Define the lane
	Lane right_lane_waypoints = Lane(); 
	Lane middle_lane_waypoints = Lane(); 
	Lane left_lane_waypoints = Lane(); 

	// Populate the lane using waypoints data
	/*int nb_waypoints = map_wp_x.size(); 
	for(int i = 0; i < nb_waypoints; i++){
		right_lane_waypoints.push_back()
	}*/

	std::cout << "[TODO]" << std::endl; 
}

/*void changeLane(){
	// Change lane in case a car is ahead of us and obstructing the way
}*/