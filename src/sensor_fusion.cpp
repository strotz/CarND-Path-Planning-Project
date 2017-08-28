
#include "sensor_fusion.h"

const detected_vehicle *sensor_fusion::find_nearest_in_range(int lane, point_cref start, point_cref end) const {
	const detected_vehicle* leader = nullptr;
	for (std::vector<detected_vehicle>::const_iterator c = other_cars_.cbegin(); c != other_cars_.cend(); ++c) {
		if (lane == c->lane() && start < c->s_ && c->s_ < end) {
			if (leader == nullptr) {
				leader = &(*c);
			} else {
				if (c->s_ < leader->s_) {
					leader = &(*c);
				}
			}
		}
	}
	return leader;
}

void sensor_fusion::dump() const {
	for(const auto& c : other_cars_) {
		std::cout << "car " << c.id_ << " - " << c.s_.value() << " - " << c.lane() << " v=" << c.v_ << std::endl;
	}
}
