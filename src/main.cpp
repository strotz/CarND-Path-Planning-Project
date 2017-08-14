#include <fstream>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "world.h"
#include "acceleration_planner.h"
#include "vehicle.h"
#include "timing_profile.h"
#include "trajectory.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos) {
		return "";
	} else if (b1 != string::npos && b2 != string::npos) {
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

int main() {
	uWS::Hub h;

	// Waypoint map to read from
	string map_file_ = "../../data/highway_map.csv";

	world around;
	around.load_from_file(map_file_);

	bool first_call = true;
	vehicle_state predicted; // last state that algorithm predict
	double target_velocity;
	int target_lane;

	h.onMessage([&around, &predicted, &first_call, &target_velocity, &target_lane](
		uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
		uWS::OpCode opCode) {


		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		//auto sdata = string(data).substr(0, length);
		//cout << sdata << endl;
		if (length && length > 2 && data[0] == '4' && data[1] == '2') {

			auto s = hasData(data);

			if (s != "") {
				auto j = json::parse(s);

				string event = j[0].get<string>();

				if (event == "telemetry") {
					// j[1] is the data JSON object

					// Main car's localization Data
					vehicle car;
					car.load_json(j[1]);

					// Previous path data given to the Planner
					auto previous_path_x = j[1]["previous_path_x"];
					auto previous_path_y = j[1]["previous_path_y"];

					// Previous path's end s and d values
					double end_path_s = j[1]["end_path_s"];
					double end_path_d = j[1]["end_path_d"];

					// Sensor Fusion Data, a list of all other cars on the same side of the road.
					auto sensor_fusion = j[1]["sensor_fusion"];

					if (first_call) {
						predicted = car; // first time, predicted state is current car state
						target_velocity = max_velocity;
						target_lane = car.lane();
						first_call = false;
						cout << "init done" << endl;
					}

					// TODO: collision detection and state machine

					auto previous_size = previous_path_x.size();
					cout << "items in buffer: " << previous_size << ", velocity: " << car.v_ << endl;

					vector<double> next_x_vals;
					vector<double> next_y_vals;

					for(auto x: previous_path_x) {
						next_x_vals.push_back(x);
					}
					for(auto y: previous_path_y) {
						next_y_vals.push_back(y);
					}

					const double required_steps = max_duration / step_duration;
					if (previous_size < required_steps) { // calculate new profile

						std::unique_ptr<timing_profile> timing;
						std::unique_ptr<trajectory> path;

						cout << "v: " << predicted.velocity() << ", target: " << target_velocity << endl;
						if (velocity::same(predicted.velocity(), target_velocity)) {
							timing = timing_profile_builder::maintain_velocity(predicted.velocity());
							cout << "maintain velocity: " << timing->total_duration() << endl;
						} else {
							timing = timing_profile_builder::reach_velocity(predicted.velocity(), target_velocity);
							cout << "change speed: " << timing->total_duration() << endl;
						}

						if (predicted.lane() == target_lane) {
							// TODO: how to predict orientation for run_state?
							path = trajectory::maintain_lane(around, predicted, *timing);
						} else {
							// TODO: implement change of lane
							throw not_implemented();
						}

						int steps = timing->total_duration() / step_duration;
						for (int i = 1; i <= steps; i++) {
							auto delay = i * step_duration;
							auto distance = timing->get_distance_at(delay);
							auto next_position = path->get_position_at(distance);

							next_x_vals.push_back(next_position.x_);
							next_y_vals.push_back(next_position.y_);
						}
						cout << "sending new profile" << endl;
						predicted = path->predicted();
					}

					// END

					json msgJson;
					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\"," + msgJson.dump() + "]";

					//this_thread::sleep_for(chrono::milliseconds(1000));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

				}
			} else {
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});

	// We don't need this since we're not using HTTP but if it's removed the
	// program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
										 size_t, size_t) {
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1) {
			res->end(s.data(), s.length());
		} else {
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
												 char *message, size_t length) {
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port)) {
		std::cout << "Listening to port " << port << std::endl;
	} else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}
















































































