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
		position result;
		result.x_ = x * cos(orientation) + y * sin(orientation);
		result.y_ = - x * sin(orientation) + y * cos(orientation);
		return result;
	}

	position project_from(const double& x0, const double& y0, const double& orientation) {
		double x = x0 + x_ * cos(orientation) - y_ * sin(orientation);
		double y = y0 + x_ * sin(orientation) + y_ * cos(orientation);
		return position(x, y);
	}
};

#endif //PATH_PLANNING_POSITION_H
