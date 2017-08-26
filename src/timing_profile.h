#ifndef PATH_PLANNING_TIMING_PROFILE_H
#define PATH_PLANNING_TIMING_PROFILE_H

#include "common.h"
#include "velocity.h"
#include <memory>
#include <algorithm>
#include <iostream>

const double max_acceleration = 8.0;
const double min_acceleration = -8.0;

const double max_duration = 0.5;
const double step_duration = 0.02;
const double required_steps = max_duration / step_duration;

const double preffered_distance = 1.7;

class timing_profile {

private:

	// assuming constant acceleration
	// TODO: need one more to cover jerk

	velocity start_v_;
	double acceleration_;
	double duration_;

public:

	timing_profile() :
		start_v_(0),
		acceleration_(0),
		duration_(0)
	{}

	timing_profile(const velocity& start_v, const double& acceleration, const double& duration) :
		start_v_(start_v),
		acceleration_(acceleration),
		duration_(duration)
	{
	}

private:

	double calculate_distance(const double& delay) const {
		return start_v_ * delay + delay * delay * acceleration_ / 2.0;
	}

public:

	double end_velocity() const {
		return start_v_ + acceleration_ * duration_;
	}

	double total_distance() const {
		return calculate_distance(duration_);
	}

	double total_duration() const {
		return duration_;
	}

	// TODO: explicit type for time
	double get_distance_at(const double& delay) const {
		if (delay < 0 || delay > duration_) {
			throw std::runtime_error("delay is out of range");
		}

		return calculate_distance(delay);
	}
};

class timing_profile_builder {
public:
	static timing_profile reach_velocity(const double& start, const double& target, const double& duration);
};

#endif //PATH_PLANNING_TIMING_PROFILE_H
