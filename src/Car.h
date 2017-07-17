#ifndef CAR_H_
#define CAR_H_

using namespace std;

class Car {
	float car_x; 
	float car_y; 
	float car_s; 
	float car_d; 
	float car_yaw; 
	float car_speed; 

	public: 
		// Destructor
		~Car() {}

		// Initialize the car 
		void init(float car_x, float car_y, float car_s, float car_d, float car_yaw, float car_speed);

		

}

#endif /* CAR_H_ */