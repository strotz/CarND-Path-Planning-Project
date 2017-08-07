//
// Created by Alexei Strots on 8/4/17.
//

#ifndef PATH_PLANNING_ACCELERATION_PLANNER_H
#define PATH_PLANNING_ACCELERATION_PLANNER_H

#include <vector>
#include "Eigen-3.3/Eigen/Dense"

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