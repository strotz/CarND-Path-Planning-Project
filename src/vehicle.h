#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include "conversion.h"
#include "position.h"
#include "json.hpp"
#include "velocity.h"
#include "point.h"

using json = nlohmann::json;

const double car_length = 5.0;
const double car_width = 3.0;

class frenet {

public:

	frenet() {}

	frenet(const double& s, const double& d) :
		s_(s),
		d_(d)
	{}

	point s_;
	double d_;
};

class vehicle_state {

public:

	position p_;
	point s_;

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
		s_ = point((double)record["s"]);
		d_ = record["d"];
		yaw_ = record["yaw"];
		speed_ = record["speed"];

		orientation_ = deg2rad(yaw_);
		v_ = mph_to_ms(speed_);
	}
};

using vehicle_state_cref = const vehicle_state&;


class detected_vehicle : public vehicle_state {
public:

	// [ id, x, y, vx, vy, s, d]
	int id_;

	void load_json(const json& record) {
		id_ = record[0];

		p_.x_ = record[1];
		p_.y_ = record[2];

		double vx = record[3];
		double vy = record[4];

		s_ = point((double)record[5]);
		d_ = record[6];

		orientation_ = atan2(vy, vx);
		v_ = sqrt(vx * vx + vy * vy);
	}
};

#endif //PATH_PLANNING_VEHICLE_H
