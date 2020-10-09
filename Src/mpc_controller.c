#include "../Inc/mpc_controller.h"

void init_params(control_data_t *control_data) {
    const float Q[3][3] = {{0.01000f, 0.00000f, 0.00000f}, {0.00000f, 1000.00000f, 0.00000f}, {0.00000f, 0.00000f, 10.00000f}};
    control_data->R = 1000000.00000f;

    memcpy(control_data->Q, Q, sizeof(Q));
}

void compute_control_input(control_data_t *control_data, flight_phase_detection_t *flight_phase_detection, env_t *env){
    if (flight_phase_detection->flight_phase == CONTROL ||flight_phase_detection->flight_phase == APOGEE_APPROACH 
        || flight_phase_detection->flight_phase == BIAS_RESET) {
        /* Compute reference velocity */
        eval_optimal_trajectory_polyfit(control_data);

        /* Calculate Velocity Error */
        compute_reference_error(control_data);

        if (flight_phase_detection->flight_phase == CONTROL) {
            plant_linearization(control_data, flight_phase_detection, env);

            float x0[3] = {0, control_data->reference_error, control_data->integrated_error};
            float C[3][4] = {{0, 1, 0, 0}, 
                            {0, 0, control_data->Ad[0][0], control_data->Ad[0][1]}, 
                            {0, 0, control_data->Ad[1][0], control_data->Ad[1][1]}};
            float D[3][4] = {{1, -1, 0, 0}, 
                            {0, control_data->Bd[0], -1, 0}, 
                            {0, control_data->Bd[1], 0, -1}};
            float cost_H[4][4] = {{control_data->R, 0, 0, 0}, 
                                  {0, control_data->Q[0][0], control_data->Q[0][1], control_data->Q[0][2]}, 
                                  {0, control_data->Q[1][0], control_data->Q[1][1], control_data->Q[1][2]}, 
                                  {0, control_data->Q[2][0], control_data->Q[2][1], control_data->Q[2][2]}};
            float cost_H_fin[4][4] = {{control_data->R, 0, 0, 0}, 
                                    {0, control_data->Q[0][0], control_data->Q[0][1], control_data->Q[0][2]}, 
                                    {0, control_data->Q[1][0], control_data->Q[1][1], control_data->Q[1][2]}, 
                                    {0, control_data->Q[2][0], control_data->Q[2][1], control_data->Q[2][2]}};

            /* somehow traditional memcpy doesnt work */
            for (int i = 0; i < 3; i++){
                control_data->mpc_params.eq_c[i] = x0[i];
                for (int j = 0; j < 4; j++){
                    control_data->mpc_params.eq_C[i*3 + j] = C[i][j];
                    control_data->mpc_params.eq_D[i*3 + j] = D[i][j];
                }
            }
            for (int i = 0; i < 4; i++){
                for (int j = 0; j < 4; j++){
                    #ifndef EULER_AV
                        control_data->mpc_params.cost_H[i*4 + j] = cost_H[i][j];
                    #endif

                    control_data->mpc_params.cost_H_fin[i*4 + j] = cost_H_fin[i][j];
                }
            }

            #ifdef EULER_AV
                control_data->mpc_exitflag = ARIS_Euler_MPC_embotech_single_integrator_20201002120922_solve(&control_data->mpc_params, 
                                                                                                            &control_data->mpc_output, 
                                                                                                            &control_data->mpc_info, NULL);
            #else
                control_data->mpc_exitflag = MPC_embotech_single_integrator_test_20201008205150_tunkapgen_solve(&control_data->mpc_params, 
                                                                                                                &control_data->mpc_output, 
                                                                                                                &control_data->mpc_info, NULL);
            #endif

            /* Exitflags:
            1 - Optimal solution has been found (subject to desired accuracy)
            2 - (only branch-and-bound) A feasible point has been identified for which the objective value is no more than codeoptions.mip.mipgap*100 per cent worse than the global optimum 
            0 - Timeout - maximum number of iterations reached
            -1 - (only branch-and-bound) Infeasible problem (problems solving the root relaxation to the desired accuracy)
            -2 - (only branch-and-bound) Out of memory - cannot fit branch and bound nodes into pre-allocated memory.
            -6 - NaN or INF occured during evaluation of functions and derivatives. Please check your initial guess.
            -7 - Method could not progress. Problem may be infeasible. Run FORCESdiagnostics on your problem to check for most common errors in the formulation.
            -10 - The convex solver could not proceed due to an internal error
            -100 - License error */

            if (control_data->mpc_exitflag >= 0) {
                 /* Check that the control input is between 0 and 1 */
                control_data->control_input = fmaxf(0, fminf(OPT_TRAJ_CONTROL_INPUT + control_data->mpc_output.u0[0], 1));
            }

            compute_integrated_error(control_data);
        } else {
            control_data_reset(control_data);
        }
    } else if ((flight_phase_detection->flight_phase == BALLISTIC_DESCENT) && 
              ((flight_phase_detection->mach_regime == SUBSONIC) || (flight_phase_detection->mach_regime == TRANSONIC))) {
        /* actuate airbrakes during ballistic descent to slow down rocket */
        control_data->control_input = 1;
    } else {
        /* This part of the controller is accessed, if the controller should not be operational */
        /* Airbrakes need to be retracted to prevent entanglement with the parachutes */
        control_data_reset(control_data);
    }
}

void plant_linearization(control_data_t *control_data, flight_phase_detection_t *flight_phase_detection, env_t *env){
    linear_model(control_data, flight_phase_detection, env);
    discretize(control_data->A, control_data->B, control_data->Ad, control_data->Bd);
}


