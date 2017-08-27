#ifndef PATH_PLANNING_VELOCITY_H_H
#define PATH_PLANNING_VELOCITY_H_H

#include <math.h>

class velocity {
public:

	velocity(const double& v) : v_(v) {
	}

	operator double&() { return v_; }

	operator double() const { return v_; }

private:

	double v_;
};


#endif //PATH_PLANNING_VELOCITY_H_H
