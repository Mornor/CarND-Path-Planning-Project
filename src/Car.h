#ifndef CAR_H_
#define CAR_H_

using namespace std;

class Car {

	public:

		float car_x;
		float car_y;
		float car_s;
		float car_d;
		float car_yaw;
		float car_speed;
		
		// Default Constructor 
		Car(float car_x, float car_y, float car_s, float car_d, float car_yaw, float car_speed)
			: car_x(car_x), car_y(car_y), car_s(car_s), car_d(car_d), car_yaw{car_yaw}, car_speed(car_speed) {}

		// Destructor
		~Car() {}
};

#endif /* CAR_H_ */