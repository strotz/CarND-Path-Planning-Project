#ifndef PATH_PLANNING_BRAIN_H
#define PATH_PLANNING_BRAIN_H

#include "states.h"
#include "vehicle.h"
#include "world.h"
#include "timing_profile.h"
#include "trajectory.h"

using namespace std;

class prediction {
	timing_profile timing_;
	shared_ptr<trajectory> path_;

public:

	vector<position> calculate_trajectory();

	prediction(timing_profile& timing, shared_ptr<trajectory>& path) :
		timing_(timing),
		path_(std::move(path))
	{
	}

	vehicle_state end_state() {
		return path_->predicted();
	}
};

class brain {

	world_cref road_;
	vehicle_state last_state_; // last state that algorithm predicts

	// TODO: do we need it?
	double target_velocity_;
	double interval_;

public:

	brain(world_cref road) :
		road_(road),
		target_velocity_(mph_to_ms(speed_limit)),
		interval_(max_duration) // each profile is going to be one (1) seconds
	{
	}

	void reset_state(vehicle_state_cref car);

	unique_ptr<prediction> run_planning(vehicle_state_cref car, const vector<detected_vehicle> &others);

private:

	unique_ptr<prediction> generate_prediction(states state, vehicle_state_cref car, const vector<detected_vehicle> &others);

	unique_ptr<prediction> generate_keep_in_line(const vehicle_state &state, const vector<detected_vehicle> &vector1);
};

#endif //PATH_PLANNING_BRAIN_H
