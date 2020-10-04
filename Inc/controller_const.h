//
// Created by Tun Kapgen on 09.06.20.
//

#ifndef CONTROLLER_CONST_H
#define CONTROLLER_CONST_H


/* Constants */
#include "controller.h"
#include <math.h>

/* CONTROLLER_TYPE: 0=TEST_CONTROLLER, 1=LQR, 2=MPC */
#define CONTROLLER_TYPE 2                                                                    
#define CONTROLLER_SAMPLING_FREQ 100                                                            // Hz
#define POLY_DEG 30                                                                             // NEEDS CHANGE
#define OPT_TRAJ_CONTROL_INPUT 0.495415061477727                                                    // -
#define CONTROL_DEACTIVATION_ALTITUDE_AGL 1382.1581114217445                                     // NEEDS CHANGE
#define MIN_BOUNDARAY_AW 0.5                                                                        // -                                                             // -
#define M_AW 0.005                                                                              // -
#define DELTA_T 1.0f / CONTROLLER_SAMPLING_FREQ                                                 // s
#define TARGET_AGOGEE 1435.0                                                                    // m                                                   // m

#if CONTROLLER_TYPE == 0
    #define TEST_CONTROLLER_USE_VELOCITY false                                                       // -
    #define TEST_CONTROLLER_MAX_VELOCITY 3.0f                                                       // m/s
    #define TEST_CONTROLLER_MAX_ALTITUDE 20.0f
#elif CONTROLLER_TYPE == 1
    #define NUM_GAINS 3 
#elif CONTROLLER_TYPE == 2
    #define SOLVER_NAME "MPC_embotech_single_integrator_test_20201004181950_maximilianstoelzle"
    #define SOLVER_HEADER "../MPC_solvers/" SOLVER_NAME "/include/" SOLVER_NAME ".h"
    // #include SOLVER_HEADER

    #include "../MPC_solvers/MPC_embotech_single_integrator_test_20201004181950_maximilianstoelzle/include/MPC_embotech_single_integrator_test_20201004181950_maximilianstoelzle.h"
#endif

/* Types */
typedef struct {
    float control_input;
    float reference_error;
    float integrated_error;

    float sf_ref_altitude_AGL;
    float sf_velocity;
    float ref_velocity;
    float tracking_feedback;

    float lowerboundary_aw;
    float upperboundary_aw;

    double optimal_trajectory_coeff[POLY_DEG+1];

    #if CONTROLLER_TYPE == 1
        double gains[NUM_GAINS];
        double poly_coeff[NUM_GAINS][POLY_DEG+1];
    #elif CONTROLLER_TYPE == 2
        float Q[3][3];
        float R;
        MPC_embotech_single_integrator_test_20201004181950_maximilianstoelzle_params mpc_params;
        MPC_embotech_single_integrator_test_20201004181950_maximilianstoelzle_output mpc_output;
        MPC_embotech_single_integrator_test_20201004181950_maximilianstoelzle_info mpc_info;
    #endif

} control_data_t;


#endif //C_IMPLEMENTATION_CONTROLLER_CONST_H
