//
// Created by alstrots on 8/11/17.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include "json.hpp"
using json = nlohmann::json;

const double lane_width = 6;

class position {

public:

	double x_;
	double y_;

	double yaw_; // TODO: orientation is not a position, split
};


class vehicle {
public:

	void load_json(const json& record) {

		y_ = record["y"];
		x_ = record["x"];
		s_ = record["s"];
		d_ = record["d"];
		yaw_ = record["yaw"];
		speed_ = record["speed"];
	}

	double x_;
	double y_;
	double s_;
	double d_;
	double yaw_;
	double speed_;

	position now() {
		position now;
		now.x_ = x_;
		now.y_ = y_;
		now.yaw_ = yaw_;
	}
};

// TODO: better name
class run_state {

public:

	int lane;
	double speed;

	bool initialized = false;

	void init(const vehicle& car) {
		lane = car.d_ / lane_width;
		speed = car.speed_ * 0.44704; // mph to m/s
		initialized = true;
	}

	double d() {
		return lane * lane_width;
	}
};

#endif //PATH_PLANNING_VEHICLE_H
