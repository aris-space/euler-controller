//
// Created by Tun Kapgen on 09.06.20.
//

#ifndef CONTROLLER_CONST_H
#define CONTROLLER_CONST_H


/* Constants */
#include "controller.h"
#include <math.h>

/* CONTROLLER_TYPE: 0=TEST_CONTROLLER, 1=LQR, 2=MPC */
#define CONTROLLER_TYPE 1                                                                    
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
    #endif

} control_data_t;


#endif //C_IMPLEMENTATION_CONTROLLER_CONST_H
