#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"

#include "acceleration_planner.h"

using namespace std;
using Eigen::MatrixXd;

acceleration_planner::acceleration_planner(double T) :
	T_(T)
{
	MatrixXd A = MatrixXd(3, 3);

	A << T*T*T, T*T*T*T, T*T*T*T*T,
	3*T*T, 4*T*T*T,5*T*T*T*T,
	6*T, 12*T*T, 20*T*T*T;

	Ai_ = A.inverse();
}


vector<double> acceleration_planner::plan(vector<double> start, vector<double> end) {

	MatrixXd B = MatrixXd(3,1);

	B << end[0]-(start[0]+start[1]*T_+.5*start[2]*T_*T_),
		end[1]-(start[1]+start[2]*T_),
		end[2]-start[2];

	MatrixXd C = Ai_ * B;

	vector <double> result = {start[0], start[1], .5*start[2]};
	for(int i = 0; i < C.size(); i++)
	{
		result.push_back(C.data()[i]);
	}

	return result;
}