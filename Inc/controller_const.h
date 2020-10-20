//
// Created by Tun Kapgen on 09.06.20.
//

#ifndef CONTROLLER_CONST_H
#define CONTROLLER_CONST_H


/* Constants */
#include <math.h>

/* CONTROLLER_TYPE: 0=TEST_CONTROLLER, 1=LQR, 2=MPC */
#define CONTROLLER_TYPE 2                                                                    
#define CONTROLLER_SAMPLING_FREQ 20.00000f                                                           // Hz
#define POLY_DEG 30                                                                             // NEEDS CHANGE
#define OPT_TRAJ_CONTROL_INPUT 0.50000f                                                    // -
#define CONTROL_DEACTIVATION_ALTITUDE_AGL 4463.63842f                                     // NEEDS CHANGE
#define MIN_BOUNDARAY_AW 0.5                                                                        // -                                                             // -
#define M_AW 0.005                                                                              // -
#define DELTA_T 1.0f / CONTROLLER_SAMPLING_FREQ                                                 // s
#define TARGET_AGOGEE 4478.00f                                                                // m

#define MPC_HORIZON 10
#define MPC_MAXIT 15 /* maximum number of MPC solver iterations */

#if CONTROLLER_TYPE == 0
    #define TEST_CONTROLLER_USE_VELOCITY false                                                       // -
    #define TEST_CONTROLLER_MAX_VELOCITY 3.0f                                                       // m/s
    #define TEST_CONTROLLER_MAX_ALTITUDE 20.0f
#elif CONTROLLER_TYPE == 1
    #define NUM_GAINS 3 
#elif CONTROLLER_TYPE == 2
    #ifdef EULER_AV
        #if MPC_HORIZON == 5
            #define SOLVER_HEADER "../MPC_solvers/horizon_5/ARIS_Euler_MPC_embotech_single_integrator_20201019233222/include/ARIS_Euler_MPC_embotech_single_integrator_20201019233222.h"
        #else 
            #define SOLVER_HEADER "../MPC_solvers/horizon_10/ARIS_Euler_MPC_embotech_single_integrator_20201019233222/include/ARIS_Euler_MPC_embotech_single_integrator_20201019233222.h"
        #endif
        typedef struct ARIS_Euler_MPC_embotech_single_integrator_20201019233222_params mpc_params_t;
        typedef struct ARIS_Euler_MPC_embotech_single_integrator_20201019233222_output mpc_output_t;
        typedef struct ARIS_Euler_MPC_embotech_single_integrator_20201019233222_info mpc_info_t;
    #else
        #define SOLVER_HEADER "../MPC_solvers/MPC_embotech_single_integrator_Full_scale_launch_20201020185541_tunkapgen/include/MPC_embotech_single_integrator_Full_scale_launch_20201020185541_tunkapgen.h"
        typedef struct MPC_embotech_single_integrator_Full_scale_launch_20201020185541_tunkapgen_params mpc_params_t;
        typedef struct MPC_embotech_single_integrator_Full_scale_launch_20201020185541_tunkapgen_output mpc_output_t;
        typedef struct MPC_embotech_single_integrator_Full_scale_launch_20201020185541_tunkapgen_info mpc_info_t;
    #endif
    #include SOLVER_HEADER
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
        float A[2][2];
        float B[2][1];
        float Ad[2][2];
        float Bd[2][1];
        solver_int32_default mpc_exitflag;
        mpc_params_t mpc_params;
        mpc_output_t mpc_output;
        mpc_info_t mpc_info;
    #endif

} control_data_t;


#endif // CONTROLLER_CONST_H
