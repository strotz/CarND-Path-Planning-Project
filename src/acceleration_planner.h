#ifndef PATH_PLANNING_ACCELERATION_PLANNER_H
#define PATH_PLANNING_ACCELERATION_PLANNER_H

#include <vector>

using std::vector;
using Eigen::MatrixXd;

class acceleration_planner
{
public:
	acceleration_planner(double T);

	vector<double> plan(vector<double> start, vector<double> end);

private:

	double T_;
	MatrixXd Ai_;
};

#endif //PATH_PLANNING_ACCELERATION_PLANNER_H
