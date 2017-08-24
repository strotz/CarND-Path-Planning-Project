
#ifndef PATH_PLANNING_COST_ESTIMATOR_H
#define PATH_PLANNING_COST_ESTIMATOR_H


#include "states.h"
#include "vehicle.h"
#include "prediction.h"

class cost_estimator {

	const states action_;
	vehicle_state_cref car_;
	const unique_ptr<prediction>& candidate_;
	const vector<detected_vehicle>& others_;
	const double& start_delay_;

public:

	cost_estimator(
		states action,
		vehicle_state_cref car,
		const unique_ptr<prediction>& candidate,
		const vector<detected_vehicle>& others,
		const double& start_delay)
		:
		action_(action),
		car_(car),
		candidate_(candidate),
		others_(others),
		start_delay_(start_delay)
	{
	}

	int calculate_cost();

private:

	bool check_collision();
};


#endif //PATH_PLANNING_COST_ESTIMATOR_H
