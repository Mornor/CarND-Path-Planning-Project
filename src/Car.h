#ifndef CAR_H_
#define CAR_H_

using namespace std;

class Car {

	public:
		int id;
		float car_x;
		float car_y;
		float car_vx;
		float car_vy;
		float car_s;
		float car_d;
		
		// Default Constructor 
		Car(int id, float car_x, float car_y, float car_vx, float car_vy, float car_s, float car_d)
			: id(id), car_x(car_x), car_y(car_y), car_vx(car_vx), car_vy(car_vy), car_s{car_s}, car_d(car_d) {}

		// Destructor
		~Car() {}
};

#endif /* CAR_H_ */