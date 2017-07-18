#ifndef PATHPLANNER_H
#define PATHPLANNER_H
#include <vector>
#include "Car.h"

using namespace std;

class PathPlanner {

	public: 

		// Destructor
		virtual ~PathPlanner();

		// Maintain a list of all the cars surrounding the one we drive, updated for each timesteps
		vector<Car> MaintainListSurroundingCars(vector<vector<float>> sensorFusion);

		// Follow the given idLane at the given speed
		void FollowLane(int idLane, float speed, vector<float> &next_x_vals, vector<float> &next_y_vals); 

}; 

#endif /* PATHPLANNER_H_ */