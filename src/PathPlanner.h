#ifndef PATHPLANNER_H
#define PATHPLANNER_H
#include <vector>
#include "Car.h"

using namespace std;

class PathPlanner {

	public: 

		// Constructor 
		PathPlanner(); 

		// Destructor
		~PathPlanner() {}

		// Maintain a list of all the cars surrounding the one we drive, updated for each timesteps
		vector<Car> MaintainListSurroundingCars(vector<double> sensorFusion);

}; 

#endif /* PATHPLANNER_H_ */