#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 10;
double dt = 0.1;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
//   simulator around in a circle with a constant steering angle and velocity on
//   a flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
//   presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
/* More specifically, the model is as follows:
x_t+1 = x_t + v_t * cos(phi_t) * dt
y_t+1 = y_t + v_t * sin(phi_t) * dt
phi_t+1 = phi_t + v_t/L_f * delta_t * dt
v_t+1 = v_t + a_t * d_t
cte_t+1 = f(x_t) - y_t + v_t*sin(ephi_t) * dt
ephi_t+1 = phi_t - phidest_t + v_t/L_f*delta_t *dt

x0 is the initial state [x ,y , \psi, v, cte, e\psi],
coeffs are the coefficients of the fitting polynomial.
The bulk of this method is setting up the vehicle model constraints (constraints) and variables (vars) for Ipopt.
*/
const double Lf = 2.67;

double ref_v = 100;
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
public:
	// Fitted polynomial coefficients
	Eigen::VectorXd coeffs;
	FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

	typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
	void operator()(ADvector& fg, const ADvector& vars) {
		/*
		* `fg` is a vector of the cost constraints, `vars` is a vector of variable
		*   values (state & actuators)
		*/
		fg[0] = 0;

		/* Cost function */

		double pen_cte = 2500.0; // penalizing large cte, penalizing large angle eroror , penalizing losing reference to the preset speed
		double pen_angle = 2000.0;
		double pen_speed = 1.0;
		double pen_steering = 5.0;
		double pen_throttle = 5.0; // penalizing steering, penalizing using throttle
		double pen_st_angle = 200.0; // penlizing using large steering angles
		double pen_break = 10.0; // penalizing using adrupt breaks

								 // the above part can be tuned
		for (unsigned int t = 0; t < N; t++) {
			fg[0] += pen_cte * CppAD::pow(vars[cte_start + t], 2);
			fg[0] += pen_angle * CppAD::pow(vars[epsi_start + t], 2);
			fg[0] += pen_speed * CppAD::pow(vars[v_start + t] - ref_v, 2);
		}

		// Minimize the use of actuators.
		for (unsigned int t = 0; t < N - 1; t++) {
			fg[0] += pen_steering * CppAD::pow(vars[delta_start + t], 2);
			fg[0] += pen_throttle * CppAD::pow(vars[a_start + t], 2);
		}

		// Minimize the value gap between sequential actuations.
		for (unsigned int t = 0; t < N - 2; t++) {
			fg[0] += pen_st_angle * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
			fg[0] += pen_break * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
		}

		// Setup Constraints
		// Initial constraints
		//
		// We add 1 to each of the starting indices due to cost being located at index 0 of `fg`.
		// This bumps up the position of all the other values.
		fg[1 + x_start] = vars[x_start];
		fg[1 + y_start] = vars[y_start];
		fg[1 + psi_start] = vars[psi_start];
		fg[1 + v_start] = vars[v_start];
		fg[1 + cte_start] = vars[cte_start];
		fg[1 + epsi_start] = vars[epsi_start];

		// The rest of the constraints
		for (unsigned int t = 1; t < N; t++) {
			// The state at time t+1 .
			AD<double> x1 = vars[x_start + t];
			AD<double> y1 = vars[y_start + t];
			AD<double> psi1 = vars[psi_start + t];
			AD<double> v1 = vars[v_start + t];
			AD<double> cte1 = vars[cte_start + t];
			AD<double> epsi1 = vars[epsi_start + t];

			// The state at time t.
			AD<double> x0 = vars[x_start + t - 1];
			AD<double> y0 = vars[y_start + t - 1];
			AD<double> psi0 = vars[psi_start + t - 1];
			AD<double> v0 = vars[v_start + t - 1];
			AD<double> cte0 = vars[cte_start + t - 1];
			AD<double> epsi0 = vars[epsi_start + t - 1];

			// Only consider the actuation at time t.
			AD<double> delta0 = vars[delta_start + t - 1];
			AD<double> a0 = vars[a_start + t - 1];

			// we consider fitting third-order polynomial to the way points
			AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
			AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));

			// Here's `x` to get you started.
			// The idea here is to constraint this value to be 0.
			//
			// Recall the equations for the model:
			// x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
			// y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
			// psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
			// v_[t] = v[t-1] + a[t-1] * dt
			// cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
			// epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
			fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
			fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
			fg[1 + psi_start + t] = psi1 - (psi0 - v0 / Lf * delta0 * dt); // we changed the sign to consider negative feedback
			fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
			fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
			fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0 / Lf * delta0 * dt);
		}
	}
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

std::vector<double> MPC::Solve(const Eigen::VectorXd &state, const Eigen::VectorXd &coeffs) {
	bool ok = true;
	typedef CPPAD_TESTVECTOR(double) Dvector;

	double x = state[0];
	double y = state[1];
	double psi = state[2];
	double v = state[3];
	double cte = state[4];
	double epsi = state[5];

	/**
	Set the number of model variables (includes both states and inputs).
	* For example: If the state is a 4 element vector, the actuators is a 2
	*   element vector and there are 10 timesteps. The number of variables is:
	*   4 * 10 + 2 * 9
	*/
	size_t n_vars = N * 6 + (N - 1) * 2;
	size_t n_constraints = N * 6;

	// Initial value of the independent variables.
	// SHOULD BE 0 besides initial state.
	Dvector vars(n_vars);
	for (unsigned int i = 0; i < n_vars; i++) {
		vars[i] = 0;
	}

	Dvector vars_lowerbound(n_vars);
	Dvector vars_upperbound(n_vars);
	// Set lower and upper limits for variables.

	// Set the initial variable values
	/*
	vars[x_start] = x;
	vars[y_start] = y;
	vars[psi_start] = psi;
	vars[v_start] = v;
	vars[cte_start] = cte;
	vars[epsi_start] = epsi;
	*/
	// Set all non-actuators upper and lowerlimits
	// to the max negative and positive values.
	for (unsigned int i = 0; i < delta_start; i++) {
		vars_lowerbound[i] = -1.0e19;
		vars_upperbound[i] = 1.0e19;
	}

	// The upper and lower limits of delta are set to -25 and 25
	// degrees (values in radians).
	// NOTE: Feel free to change this to something else.
	for (unsigned int i = delta_start; i < a_start; i++) {
		vars_lowerbound[i] = -0.436332;
		vars_upperbound[i] = 0.436332;
	}

	// Acceleration/decceleration upper and lower limits.
	// NOTE: Feel free to change this to something else.
	for (unsigned int i = a_start; i < n_vars; i++) {
		vars_lowerbound[i] = -1.0;
		vars_upperbound[i] = 1.0;
	}

	// Lower and upper limits for the constraints
	// Should be 0 besides initial state.
	Dvector constraints_lowerbound(n_constraints);
	Dvector constraints_upperbound(n_constraints);
	for (unsigned int i = 0; i < n_constraints; i++) {
		constraints_lowerbound[i] = 0;
		constraints_upperbound[i] = 0;
	}
	constraints_lowerbound[x_start] = x;
	constraints_upperbound[x_start] = x;//initial state for x position

	constraints_lowerbound[y_start] = y;// initial staet for y position
	constraints_upperbound[y_start] = y;

	constraints_lowerbound[psi_start] = psi; // initial state for psi
	constraints_upperbound[psi_start] = psi;

	constraints_lowerbound[v_start] = v; //initial state for speed/velocity
	constraints_upperbound[v_start] = v;

	constraints_lowerbound[cte_start] = cte; // initial state for cte
	constraints_upperbound[cte_start] = cte;

	constraints_lowerbound[epsi_start] = epsi; // initial state for epsi
	constraints_upperbound[epsi_start] = epsi;

	
	// object that computes objective and constraints
	FG_eval fg_eval(coeffs);

	//
	// NOTE: You don't have to worry about these options
	//
		// options for IPOPT solver
	std::string options;
	// Uncomment this if you'd like more print information
	options += "Integer print_level  0\n";
	// NOTE: Setting sparse to true allows the solver to take advantage
	// of sparse routines, this makes the computation MUCH FASTER. If you
	// can uncomment 1 of these and see if it makes a difference or not but
	// if you uncomment both the computation time should go up in orders of
	// magnitude.
	options += "Sparse  true        forward\n";
	options += "Sparse  true        reverse\n";
	// NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
	// Change this as you see fit.
	options += "Numeric max_cpu_time          0.5\n";
	// place to return solution
	CppAD::ipopt::solve_result<Dvector> solution;
	// solve the problem
	CppAD::ipopt::solve<Dvector, FG_eval>(
		options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
		constraints_upperbound, fg_eval, solution);
	// Check some of the solution values
	ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
	// Cost
	//	auto cost = solution.obj_value;
		//std::cout << "Cost " << cost << std::endl;

	
	 /**
	 Return the first actuator values. The variables can be accessed with `solution.x[i]`.
	  * {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
	  */
	 // the control we wanted to return are the delta_0 and a_0 
	 std::vector<double> res;
	 res.push_back(solution.x[delta_start]);
	 res.push_back(solution.x[a_start]);
	 // we can return additional information regarding the trajectories
	 for (unsigned int i = 0; i < N - 1; i++) {
		res.push_back(solution.x[x_start + i]);
		res.push_back(solution.x[y_start + i]);
	 }
	 return res;
}