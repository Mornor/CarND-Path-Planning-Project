#ifndef LANE_H
#define LANE_H

using namespace std;

class Lane {

	public:

		int id; // 0 is right, 1 is middle and 2 is right
		vector<double> x_wp;
		vector<double> y_wp;
		vector<double> d_wp;
		vector<double> s_wp;

		// Destructor
		virtual ~Lane();

		// Default constructor 
		Lane(int id)
			:id(id){} 

		Lane(int id, vector<double> x_wp, vector<double> y_wp, vector<double> d_wp, vector<double> s_wp)
			: id(id), x_wp(x_wp), y_wp(y_wp), d_wp(d_wp), s_wp(s_wp){}
}; 

#endif /* LANE_H_ */