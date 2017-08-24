
#include "cost_estimator.h"

// TODO: fix collision logic on shift, calculate prediction of other car (slightly behind) position?

bool cost_estimator::check_collision()
{
	//  [ego]------start_delay---->([ego_start]------>[ego_end])
	//     [car]--start_delay-->([start]------[end])

	point ego_start = candidate_->start_state().s_ - car_length;
	point ego_end = candidate_->end_state().s_;

#ifdef DEV
	cout << "EGO: " << ego_start << ", " << ego_end << endl;
#endif

	int ego_start_lane = candidate_->start_state().lane();
	int ego_end_lane = candidate_->end_state().lane();

	const double car_width = 3.0;
	const double car_side = car_width / 2.0;
	double ego_left = min(candidate_->start_state().d_, candidate_->end_state().d_)  - car_side;
	double ego_right = max(candidate_->start_state().d_, candidate_->end_state().d_) + car_side;

	double duration = candidate_->duration();

	for(const auto& c : others_.cars()) {
		point start = c.s_ + c.v_ * start_delay_ - car_length;
		point end = start + car_length + c.v_ * duration;
		if (start < ego_end && ego_start < end)
		{
			double right = c.d_ + car_side;
			double left = c.d_ - car_side;

			if (right >= ego_left && left <= ego_right) {
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
