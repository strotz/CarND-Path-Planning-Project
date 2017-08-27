
#ifndef PATH_PLANNING_SENSOR_FUSION_H
#define PATH_PLANNING_SENSOR_FUSION_H


#include "vehicle.h"

class sensor_fusion {

	std::vector<detected_vehicle> other_cars_;

public:

	void load_json(const json& records) {
		for(const auto& record : records) {
			detected_vehicle o;
			o.load_json(record);
			other_cars_.push_back(o);
		}
	}

	const detected_vehicle* find_nearest_in_range(int lane, point_cref start, point_cref end) const;

	const std::vector<detected_vehicle>& cars() const  {
		return other_cars_;
	}

	void shift_s() {
		for(auto& c: other_cars_) {
			c.s_.shift();
		}
	}

	void dump() const;
};

using sensor_fusion_cref = const sensor_fusion&;


#endif //PATH_PLANNING_SENSOR_FUSION_H
