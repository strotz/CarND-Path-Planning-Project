#include "timing_profile.h"
#include "conversion.h"

timing_profile timing_profile_builder::reach_velocity(const double &start, const double &target, const double &duration) {
	double required = (target - start) / duration;
	double acceleration = (target > start) ? std::min(max_acceleration, required) : std::max(min_acceleration, required);
	velocity v(start);
	return timing_profile(v, acceleration, duration);
}
