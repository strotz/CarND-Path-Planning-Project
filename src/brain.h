#ifndef PATH_PLANNING_BRAIN_H
#define PATH_PLANNING_BRAIN_H

#include "states.h"
#include "vehicle.h"
#include "world.h"
#include "timing_profile.h"
#include "trajectory.h"
#include "prediction.h"
#include "sensor_fusion.h"

using namespace std;

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

	unique_ptr<prediction> run_planning(vehicle car, sensor_fusion_cref others, double d);

	void shift_s();

	void correct_last_state(double d, double d1);

private:

	unique_ptr<prediction> generate_prediction(states state, vehicle_state_cref car, vehicle_state_cref start_state, sensor_fusion_cref others);

	unique_ptr<prediction> generate_slow_down(const vehicle_state &car, vehicle_state_cref start_state, sensor_fusion_cref others);
	unique_ptr<prediction> generate_keep_in_line(const vehicle_state &start_state, sensor_fusion_cref others);
	unique_ptr<prediction> generate_change_line(const vehicle_state &start_state, sensor_fusion_cref others, int target_lane);
};

#endif //PATH_PLANNING_BRAIN_H
