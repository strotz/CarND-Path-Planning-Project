
#include "cost_estimator.h"

bool cost_estimator::check_collision() {
	int nparts = 5;
	double part_duration = candidate_->duration() / nparts;

	auto ego = vector<frenet>();
	out() << "ego trajectory:" << endl;
	for (int i = 0; i <= nparts; ++i) {
		auto t = candidate_->predict_frenet_at(i * part_duration);
		out() << t.s_.value() << " - " << t.d_ << endl;
		ego.push_back(t);
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
				(fabs(ego[i].s_ - s) <= (2 * car_length)))
			{
				out() << "collision found" << endl;
				return true;
			}
		}
	}
	return false;
}

double cost_estimator::distance_to_slow_leader() {
	double final_delay = start_delay_ + candidate_->duration();

	auto ego_s = candidate_->end_state().s_;

	auto ego_min_d = (candidate_->end_state().lane()) * lane_width;
	auto ego_max_d = (candidate_->end_state().lane() + 1) * lane_width;

	auto ego_v = candidate_->end_state().velocity();

	double good_distance = final_delay * ego_v;
	out() << "good distance: " << good_distance << endl;
	double leader = good_distance + 1;
	for(const auto& c : others_.cars()) {
		out() << "car id=" << c.id_ << " v=" << c.v_ << " d=" << c.d_ << endl;
		if ((c.v_ < ego_v) && (ego_min_d <= c.d_) && (c.d_ <= ego_max_d)) {
			auto s = c.s_ + c.velocity() * final_delay;
			auto distance = s - ego_s;
			if ((distance > 0)) {
				out() << "distance=" << distance << endl;
				if (distance < leader) {
					leader = distance;
				}
			}
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
		result += 100000;
	}

	result += distance_to_slow_leader() * 12; // penalize

	// penalize change lane
	if (action_ == LineChangeRight || action_ == LineChangeLeft) {
		result += 20;
	}

	// faster is better
	result += (mph_to_ms(speed_limit) - candidate_->end_state().v_) * 10;

	return result;

}
