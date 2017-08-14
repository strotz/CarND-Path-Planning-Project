#ifndef PATH_PLANNING_TIMING_PROFILE_H
#define PATH_PLANNING_TIMING_PROFILE_H

#include "velocity.h"
#include <memory>

const double max_acceleration = 8.0;
const double min_acceleration = -8.0;

const double max_duration = 1.0;
const double step_duration = 0.02;

class timing_profile {

private:

	// assuming 2 profiles: acceleration, constant velocity
	// TODO: need one more to cover jerk

	velocity start_v_;
	double acceleration_;
	double duration_;

public:

	timing_profile(const velocity& start_v, const double& duration) :
		start_v_(start_v),
		acceleration_(0),
		duration_(duration)
	{
	}

	timing_profile(const velocity& start_v, const double& acceleration, const double& duration) :
		start_v_(start_v),
		acceleration_(acceleration),
		duration_(duration)
	{
		// double reach_speed = start_v_ + acceleration_ * stop_accelation_;
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

	static std::unique_ptr<timing_profile> reach_velocity(const double& start, const double& target) {
		double acceleration = (target > start) ? max_acceleration : min_acceleration;
		double duration = (target - start) / acceleration;
		velocity v(start);
		return make_unique<timing_profile>(v, acceleration, duration);
	}

	static std::unique_ptr<timing_profile> maintain_velocity(const double& start, const double& duration = max_duration) {
		velocity v(start);
		return make_unique<timing_profile>(v, duration);
	}

};

#endif //PATH_PLANNING_TIMING_PROFILE_H
