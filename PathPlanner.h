#ifndef PathPlanner_H_
#define PathPlanner_H_
#include <vector>
#include "Car.h"

using namespace std;

class PathPlanner {

	public: 
		// Destructor
		~Car() {}

		// Maintain a list of all the cars surrounding the one we drive, updated for each timesteps
		vector<Car> maintainListSurroundingCars(vector <sensorFusion>);

}; 

#endif /* PathPlanner_H_ */