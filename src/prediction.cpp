#include "prediction.h"

vector<position> prediction::calculate_trajectory() {
	auto result = vector<position>();

	int steps = timing_.total_duration() / step_duration;
	for (int i = 1; i <= steps; i++) {
		auto delay = i * step_duration;
		auto distance = timing_.get_distance_at(delay);
		auto next_position = path_->get_position_at(distance);
		result.push_back(next_position);
	}
	return result;
}


