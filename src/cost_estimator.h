
#ifndef PATH_PLANNING_COST_ESTIMATOR_H
#define PATH_PLANNING_COST_ESTIMATOR_H


#include "states.h"
#include "vehicle.h"
#include "prediction.h"
#include "sensor_fusion.h"



class cost_estimator {

	const states action_;
	vehicle_state_cref car_;
	const unique_ptr<prediction>& candidate_;
	sensor_fusion_cref others_;
	const double& start_delay_;

	const point& ego_s_;
	const double ego_min_d_;
	const double ego_max_d_;
	const point& ego_start_s_;

public:

	cost_estimator(
		states action,
		vehicle_state_cref car,
		const unique_ptr<prediction>& candidate,
		sensor_fusion_cref others,
		const double& start_delay)
		:
		action_(action),
		car_(car),
		candidate_(candidate),
		others_(others),
		start_delay_(start_delay),
		ego_s_(car_.s_),
		ego_start_s_(candidate->start_state().s_),
		ego_min_d_(car_.lane_min_d()),
		ego_max_d_(car_.lane_max_d())
	{
		out() << "actual car s=" << ego_s_.value();
		out() << " lane coverage d from " << ego_min_d_ << " to " << ego_max_d_;
		out() << " v=" << car_.v_ << endl;

		out() << "prediction start at S=" << ego_start_s_.value() << endl;
	}

	int calculate_cost();

private:

	bool check_collision();

	double distance_to_slow_leader();

	double check_predicted();
};


#endif //PATH_PLANNING_COST_ESTIMATOR_H
