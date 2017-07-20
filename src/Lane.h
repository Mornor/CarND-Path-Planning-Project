#ifndef LANE_H
#define LANE_H

using namespace std;

class Lane {

	public:

		int id; // 0 is left, 1 is middle and 2 is right
		vector<double> x_wp;
		vector<double> y_wp;
		vector<double> s_wp;

		// Destructor
		virtual ~Lane();

		// Default constructor 
		Lane(int id)
			:id(id){} 

		// Update the current Lane instance
		//void update(vector<double> x_wp, vector<double> y_wp, vector<double> s_wp); 
}; 

#endif /* LANE_H_ */