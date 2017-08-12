#ifndef PATHPLANNER_H
#define PATHPLANNER_H
#include <vector>
#include <math.h>
#include "Car.h"
#include "Lane.h"
#include "spline.h"

using namespace std;

class PathPlanner {

	public: 

		// Destructor
		virtual ~PathPlanner();

		// Maintain a list of all the cars surrounding the one we drive, updated for each timesteps
		vector<Car> MaintainListSurroundingCars(vector<vector<float>> sensorFusion);

		// Follow the given idLane at the given speed
		//void FollowLane(vector<double> &next_x_vals, vector<double> &next_y_vals, vector<double> map_wp_x, vector<double> map_wp_y, vector<double> map_wp_s, vector<double> map_wp_dx, vector<double> map_wp_dy); 
		//void FollowLane(vector<double> *next_x_vals, vector<double> *next_y_vals, vector<double> prev_path_x, vector<double> prev_path_y, tk::spline waypoints_x, tk::spline waypoints_y, tk::spline waypoints_dx, tk::spline waypoints_dy, double car_x, double car_y, double car_s, double car_yaw);

		// Fit a Bezier curve 
		// void InterpolatePoints(vector<double> &waypoints_x, vector<double> &waypoints_y, vector<double> &waypoints_dx, vector<double> &waypoints_dy, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_dx, vector<double> map_waypoints_dy);

}; 

#endif /* PATHPLANNER_H_ */