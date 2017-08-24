
#include "cost_estimator.h"

// TODO: fix collision logic on shift, calculate prediction of other car (slightly behind) position?

bool cost_estimator::check_collision()
{
	//  [ego]------start_delay---->([ego_start]------>[ego_end])

	//     [car]--start_delay-->([start]------[end])


	double ego_start = candidate_->start_state().s_ - car_length;
	double ego_end = candidate_->end_state().s_;

#ifdef DEV
	cout << "EGO: " << ego_start << ", " << ego_end << endl;
#endif

	int ego_start_lane = candidate_->start_state().lane();
	int ego_end_lane = candidate_->end_state().lane();

	double duration = candidate_->duration();

	for(auto c : others_) {
		double start = c.s_ + c.v_ * start_delay_ - car_length;
		double end = start + car_length + c.v_ * duration;

		if (start < ego_end && end > ego_start)
		{
			if (c.lane() == ego_start_lane || c.lane() == ego_end_lane) {
				return true;
			}
		}
	}
	return false;
}

int cost_estimator::calculate_cost() {
	int result = 0;

	if (check_collision()) {
		result += 10000;
	}

	// result += distance_to_leader() * 10;

	// penalize change lane
	if (action_ == LineChangeRight || action_ == LineChangeLeft) {
		result += 100;
	}

	// faster is better
	result += (mph_to_ms(speed_limit) - candidate_->end_state().v_) * 100;

	return result;

}
