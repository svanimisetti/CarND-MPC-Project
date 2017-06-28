#ifndef PARAMS_H_
#define PARAMS_H_

#include <stdlib.h>

namespace mpc_params
{
    // Set the timestep length and duration
    extern const size_t N;
    extern const double dt;
    extern const double dt_latency;

    // Cost penalties for cte, epsi and v_start
    extern const double lambda_cte;
    extern const double lambda_epsi;
    extern const double lambda_v;

    // Additional hyperparameters to penalize agressive maneuvers
    // Cost penalties to minimize use of steering and acceleration
    extern const double lambda_delta;
    extern const double lambda_a;
    // Cost penalties to minimize sudden changes in steering and acceleration
    extern const double lambda_ddelta;
    extern const double lambda_da;

    // Length from front to CoG that has a similar radius.
    extern const double Lf;

    // Reference (target steady state) velocity
    extern const double ref_cte;
    extern const double ref_epsi;
    extern const double ref_v;

    // The solver takes all the state variables and actuator
    // variables in a singular vector. Thus, we should to establish
    // when one variable starts and another ends to make our lifes easier.
    extern const size_t x_start;
    extern const size_t y_start;
    extern const size_t psi_start;
    extern const size_t v_start;
    extern const size_t cte_start;
    extern const size_t epsi_start;
    extern const size_t delta_start;
    extern const size_t a_start;

}

#endif /* PARAMS_H_ */