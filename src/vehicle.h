#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include "conversion.h"
#include "position.h"
#include "json.hpp"

using json = nlohmann::json;

class vehicle_state {

public:

	position p_;

	double s_;
	double d_;

	double orientation_;
	double v_;

	double orientation() const {
		return orientation_;
	}

	position position_now() const {
		return p_;
	}

	int lane() const {
		return d_to_lane(d_);
	}

	double velocity() const {
		return v_;
	}
};

class vehicle : public vehicle_state {
public:

	double yaw_;
	double speed_;

	void load_json(const json& record) {

		p_.y_ = record["y"];
		p_.x_ = record["x"];
		s_ = record["s"];
		d_ = record["d"];
		yaw_ = record["yaw"];
		speed_ = record["speed"];

		orientation_ = deg2rad(yaw_);
		v_ = mph_to_ms(speed_);
	}
};

#endif //PATH_PLANNING_VEHICLE_H
