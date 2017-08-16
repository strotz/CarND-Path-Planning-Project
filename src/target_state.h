#ifndef PATH_PLANNING_TARGET_STATE_H
#define PATH_PLANNING_TARGET_STATE_H

#include <iostream>

#include "common.h"
#include "velocity.h"
#include "state_machine.h"

class target_state {

	velocity v_;
	int lane_;

public:

	target_state() :
		v_(0.0),
		lane_(0)
	{
	}

	target_state(const double& v, int lane) :
		v_(v),
		lane_(lane)
	{
	}

	double v() const {
		return v_;
	}

	void set_v(const double& v) {
#ifdef DEV
		if (!v_.same_as(v)) {
			std::cout << "change target velocity from " << v_ << " to " << v << std::endl;
		}
#endif // DEV
		v_ = v;
	}

	int lane() const {
		return lane_;
	}

	void set_lane(int lane) {
#ifdef DEV
		if (lane != lane_) {
			std::cout << "shift lane from " << lane_ << " to " << lane << std::endl;
		}
#endif // DEV
		lane_ = lane;
	}
};


#endif //PATH_PLANNING_TARGET_STATE_H
