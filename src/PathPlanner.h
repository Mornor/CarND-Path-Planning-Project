#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <math.h>
#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include "spline.h"

class PathPlanner{

	public:
		// Destructor
		virtual ~PathPlanner(); 

		// Change Lane, 0 being the left one, 1 the middle and 2 the right one
		void changeLane(int lane); 
};

#endif //PATHPLANNER_H