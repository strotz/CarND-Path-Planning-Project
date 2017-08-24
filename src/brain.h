#ifndef PATH_PLANNING_BRAIN_H
#define PATH_PLANNING_BRAIN_H

#include "states.h"
#include "vehicle.h"
#include "world.h"
#include "timing_profile.h"
#include "trajectory.h"
#include "prediction.h"

using namespace std;

vector<detected_vehicle>::const_iterator find_nearest_car(const vector<detected_vehicle>& others,
                                                          int lane, double from, double to);


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

	unique_ptr<prediction> run_planning(vehicle car, vector<detected_vehicle> others, double d);

	bool has_emergencies(const vehicle &car, const vector<detected_vehicle> &others);

private:

	unique_ptr<prediction> generate_prediction(states state, vehicle_state_cref car, const vector<detected_vehicle> &others);

	unique_ptr<prediction> generate_keep_in_line(const vehicle_state &start_state, const vector<detected_vehicle>& others);
	unique_ptr<prediction> generate_change_line(const vehicle_state &start_state, const vector<detected_vehicle>& others, int target_lane);
};

#endif //PATH_PLANNING_BRAIN_H
