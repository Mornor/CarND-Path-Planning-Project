#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "PathPlanner.h"

using namespace std;

// Init the PathPlanner
PathPlanner::~PathPlanner() {}

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


/*void followLane(double previous_path_x, double previous_path_y, double end_path_s, double end_path_d){
	// Follow a given lane at the give speed
	// Check if car ahead, and if yes, change the lane.
	// Return the next position (next_x_vals and next_y_vals of the car)
	vector<double> next_x_vals;
	vector<double> next_y_vals;

	return {next_x_vals, next_y_vals}
}

void changeLane(){
	// Change lane in case a car is ahead of us and obstructing the way
}*/