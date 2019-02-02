#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "MPC.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
int main() {
	uWS::Hub h;

	// MPC is initialized here!
	MPC mpc;

	h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
		uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		string sdata = string(data).substr(0, length);
		//cout << sdata << endl;
		if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
			string s = hasData(sdata);
			if (s != "") {
				auto j = json::parse(s);
				string event = j[0].get<string>();
				if (event == "telemetry") {
					// j[1] is the data JSON object
					vector<double> ptsx = j[1]["ptsx"];
					vector<double> ptsy = j[1]["ptsy"];
					double px = j[1]["x"];
					double py = j[1]["y"];
					double psi = j[1]["psi"];
					double v = j[1]["speed"];

					/**
					Calculate steering angle and throttle using MPC. Both are in between [-1, 1].
					*/

					// note that MPC.solve takes the following arguments Solve(const VectorXd &state, const VectorXd &coeffs)
					// therefore, we need to build up the state and coefficients accordingly.
					//	Remember that the server returns waypoints using the map's coordinate system, which is different than the car's coordinate system.
					//Transforming these waypoints will make it easier to both display them and to calculate the CTE and Epsi values for the model predictive controller.

					size_t n_waypoints = ptsx.size();
					auto waypoints_x = Eigen::VectorXd(n_waypoints);
					auto waypoints_y = Eigen::VectorXd(n_waypoints);
					for (int i = 0; i < n_waypoints; i++) {
						double diff_x = ptsx[i] - px;
						double diff_y = ptsy[i] - py;
						waypoints_x(i) = diff_x * cos(-psi) - diff_y * sin(-psi);
						waypoints_y(i) = diff_x * sin(-psi) + diff_y * cos(-psi);
					}

					// fit a third order polynomial to the waypoints defined in the carframe
					auto coeffs = polyfit(waypoints_x, waypoints_y, 3);

					// calculating the cte and the orientation error
					// cte is calculated by evaluating at polynomial at x (-1) and subtracting y.
					double cte = polyeval(coeffs, 0.0); // this is because target x and target y are both equal to 0
														//Recall orientation error is calculated as follows e\psi = \psi - \psi{des}, where \psi{des} is can be calculated as arctan(f'(x))arctan(f(x)).
					double epsi = -atan(coeffs[1]);  // this is because target angle psi equals to 0 and f(x) = coeff[1] + coeff[2] * x + coeff[3]*x*x with x equals to 0 in the car frame

													 // the initial state of current equals to the following
					Eigen::VectorXd state(6);
					state << 0, 0, 0, v, cte, epsi;

					vector<double> info = mpc.Solve(state, coeffs);

					double steer_value = info[0];
					double throttle_value = info[1];

					json msgJson;
					// NOTE: Remember to divide by deg2rad(25) before you send the 
					//   steering value back. Otherwise the values will be in between 
					//   [-deg2rad(25), deg2rad(25] instead of [-1, 1].
					msgJson["steering_angle"] = steer_value / (deg2rad(25));
					msgJson["throttle"] = throttle_value;

					// Display the MPC predicted trajectory 
					vector<double> mpc_x_vals;
					vector<double> mpc_y_vals;

					/**
					add (x,y) points to list here, points are in reference to the vehicle's coordinate system the points in the simulator are connected by a Green line
					*/
					// notice that the vector info contains following information:[delta, a, x[1], y[1], x[2], y[2]....]
					for (int i = 2; i < info.size(); i += 2) {
						mpc_x_vals.push_back(info[i]);
						mpc_y_vals.push_back(info[i + 1]);
					}

					msgJson["mpc_x"] = mpc_x_vals;
					msgJson["mpc_y"] = mpc_y_vals;

					// Display the waypoints/reference line
					vector<double> next_x_vals;
					vector<double> next_y_vals;

					/**
					add (x,y) points to list here, points are in reference to the vehicle's coordinate system the points in the simulator are connected by a Yellow line
					*/

					double d_x = 2.0;
					int num_ref_pts = 25;
					for (unsigned int i = 0; i < num_ref_pts; i++) {
						next_x_vals.push_back(i*d_x);
						next_y_vals.push_back(polyeval(coeffs, i*d_x));
					}


					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;


					auto msg = "42[\"steer\"," + msgJson.dump() + "]";
					//std::cout << msg << std::endl;

					// Latency
					// The purpose is to mimic real driving conditions where
					// the car does actuate the commands instantly.
					//
					// Feel free to play around with this value but should be to drive
					// around the track with 100ms latency.
					//
					// NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
					// SUBMITTING.
					std::this_thread::sleep_for(std::chrono::milliseconds(100));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			}
			else {
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
		}
		else {
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
	}
	else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}