#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

using namespace std;

void followLane(double previous_path_x, double previous_path_y, double end_path_s, double end_path_d){
	// Follow a given lane at the give speed
	// Check if car ahead, and if yes, change the lane.
	// Return the next position (next_x_vals and next_y_vals of the car)
	vector<double> next_x_vals;
	vector<double> next_y_vals;

	return {next_x_vals, next_y_vals}
}

void changeLane(){
	// Change lane in case a car is ahead of us and obstructing the way
}