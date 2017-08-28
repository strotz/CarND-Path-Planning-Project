
#include "cost_estimator.h"

bool cost_estimator::check_collision() {
	int nparts = 5;
	double part_duration = candidate_->duration() / nparts;

	auto ego = vector<point>();
	out() << "ego trajectory:" << endl;
	for (int i = 0; i <= nparts; ++i) {
		auto s = candidate_->predict_s_at(i * part_duration);
		out() << s.value() << endl;
		ego.push_back(s);
	}

	double ego_min_d = min(candidate_->start_state().lane(), candidate_->end_state().lane()) * lane_width;
	double ego_max_d = (max(candidate_->start_state().lane(), candidate_->end_state().lane()) + 1) * lane_width;

	out() << "ego d: " << ego_min_d << " -- " << ego_max_d << endl;

	for (const auto &c : others_.cars()) {
		point start = c.s_ + c.v_ * start_delay_;
		double d = c.d_;

		out() << "car id=" << c.id_ << ", d=" << c.d_ << endl;

		for (int i = 0; i <= nparts; ++i) {
			point s = start + c.v_ * i * part_duration;
			out() << s.value() << endl;

			if ((ego_min_d <= d) && (d <= ego_max_d) &&
			    (fabs(ego[i] - s) <= (1.3 * car_length))) {
				out() << "collision found" << endl;
				return true;
			}
		}
	}
	return false;
}

double cost_estimator::check_predicted() {

	auto ego_v = max(car_.v_, candidate_->start_state().v_);

	const double unsafe_distance = ego_start_s_ - ego_s_;

	double closest_distance = unsafe_distance + 1;
	double current_v;

	for(const auto& c:others_.cars()) {
		if ((ego_min_d_ < c.d_) && (c.d_ < ego_max_d_) && (ego_s_ < c.s_) && (c.s_ < ego_start_s_)) {
			out() << "car id=" << c.id_ << " in unsafe zone" << endl;
			double distance = c.s_ - ego_s_;
			if (distance < closest_distance) {
				closest_distance = distance;
				current_v = c.v_;
			}
		}
	}
	if (current_v > 0) {
		return max((ego_v - current_v), 0.0); // TODO: adjust
	}
	return 0;
}

double cost_estimator::distance_to_slow_leader() {
	double total_delay = start_delay_ + candidate_->duration();
	out() << "total delay =" << total_delay << endl;

	auto ego_s = candidate_->end_state().s_;
	out() << "ego s=" << ego_s.value() << endl;

	auto ego_min_d = (candidate_->end_state().lane()) * lane_width;
	auto ego_max_d = (candidate_->end_state().lane() + 1) * lane_width;

	auto ego_v = candidate_->end_state().v_;

	double good_distance = total_delay * ego_v;
	out() << "good distance: " << good_distance << endl;
	double leader = good_distance + 1;

	for (const auto &c : others_.cars()) {
		out() << "car id=" << c.id_ << " v=" << c.v_ << " d=" << c.d_ << endl;
		if ((ego_min_d <= c.d_) && (c.d_ <= ego_max_d)) {
			auto s = c.s_ + c.v_ * total_delay;
			out() << "s=" << s.value() << " ";
			auto distance = s - ego_s;
			out() << "distance=" << distance;
			if ((distance > 0)) {
				if (distance < leader) {
					leader = distance;
				}
			}
			out() << endl;
		}
	}
	if (leader > good_distance) {
		return 0;
	} else {
		out() << "leader is in " << leader << endl;
		return good_distance - leader;
	}
}

int cost_estimator::calculate_cost() {
	int result = 0;

	if (check_collision()) {
		out() << "collision found" << endl;
		result += 10000;

		// penalize change lane during collision
		if (action_ == LineChangeRight || action_ == LineChangeLeft) {
			result += 10000;
		}
	}

	result += check_predicted() * 3000; // force to slow down and move cars from already predicted zone

	result += distance_to_slow_leader() * 12; // penalize

	// penalize change lane without collision
	if (action_ == LineChangeRight || action_ == LineChangeLeft) {
		result += 20;
	}

	// faster is better
	result += fabs(mph_to_ms(speed_limit) - candidate_->end_state().v_) * 10;

	return result;

}

