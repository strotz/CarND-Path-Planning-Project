#ifndef PATH_PLANNING_POSITION_H
#define PATH_PLANNING_POSITION_H

#include <math.h>

class position {

public:

	double x_;
	double y_;

	position() :
		x_(0.0),
		y_(0.0)
	{
	}

	position(const double& x, const double& y) :
		x_(x),
		y_(y)
	{
	}

	position project_to(const double& x0, const double& y0, const double& orientation) {
		double x = x_ - x0;
		double y = y_ - y0;
		double pos_x = x * cos(orientation) + y * sin(orientation);
		double pos_y = - x * sin(orientation) + y * cos(orientation);
		return position(pos_x, pos_y);
	}

	position project_from(const double& x0, const double& y0, const double& orientation) {
		double x = x0 + x_ * cos(orientation) - y_ * sin(orientation);
		double y = y0 + x_ * sin(orientation) + y_ * cos(orientation);
		return position(x, y);
	}
};

#endif //PATH_PLANNING_POSITION_H
