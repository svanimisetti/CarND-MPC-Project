#include "params.h"

namespace mpc_params
{
    // Set the timestep length and duration
    const size_t N = 10;
    const double dt = 0.2;
    const double dt_latency = 0.1;

    // Cost penalties for cte, epsi and v_start
    const double lambda_cte = 1.0;
    const double lambda_epsi = 1.0;
    const double lambda_v = 1.0;

    // Additional hyperparameters to penalize agressive maneuvers
    // Following lambda's are Lagrange multipliers for the optimizer
    // Cost penalties to minimize use of steering and acceleration
    const double lambda_delta = 1.0;
    const double lambda_a = 1.0;
    // Cost penalties to minimize sudden changes in steering and acceleration
    const double lambda_ddelta = 5000.0;
    const double lambda_da = 1.0;

    // Length from front to CoG that has a similar radius.
    const double Lf = 2.67;

    // Reference (target steady state) velocity
    const double ref_cte = 0.0;
    const double ref_epsi = 0.0;
    const double ref_v = 60.0 * 0.44704; // convert mph to m/s

    // The solver takes all the state variables and actuator
    // variables in a singular vector. Thus, we should to establish
    // when one variable starts and another ends to make our lifes easier.
    const size_t x_start = 0;
    const size_t y_start = x_start + N;
    const size_t psi_start = y_start + N;
    const size_t v_start = psi_start + N;
    const size_t cte_start = v_start + N;
    const size_t epsi_start = cte_start + N;
    const size_t delta_start = epsi_start + N;
    const size_t a_start = delta_start + N - 1;

}