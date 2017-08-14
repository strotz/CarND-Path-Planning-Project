#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <vector>

#include "world.h"
#include "vehicle.h"
#include "timing_profile.h"

#include "spline/spline.h"

using std::vector;

class trajectory {

	tk::spline spline_;

	vehicle_state ref_state_;
	vehicle_state predicted_state_;

	double distance_coeff_;

public:

	trajectory(const vehicle_state &ref);

private:

	void load_positions(const vector<position> &path);

	void finalize(const double &s, const double &d, const double &distance, const double &end_velocity);

public:

	position get_position_at(const double &distance);

	vehicle_state predicted() const {
		return predicted_state_;
	}

public:

	static std::unique_ptr<trajectory>
	maintain_lane(const world &around, const vehicle_state &car, const timing_profile &timing);
};

#endif //PATH_PLANNING_TRAJECTORY_H
