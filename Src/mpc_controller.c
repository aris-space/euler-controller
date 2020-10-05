#include "../Inc/mpc_controller.h"
#include "plant_manipulator.c"

void init_params(control_data_t *control_data) {
    const float Q[3][3] = {{1.e-02, 0.e+00, 0.e+00}, {0.e+00, 1.e+03, 0.e+00}, {0.e+00, 0.e+00, 1.e+01}};
    control_data->R = 1000000;

    memcpy(control_data->Q, Q, sizeof(Q));
}

void compute_control_input(control_data_t *control_data, flight_phase_detection_t *flight_phase_detection, env_t *env){
    if (flight_phase_detection->flight_phase == CONTROL) {

        /* Calculate Velocity Error */
        compute_reference_error(control_data);

        plant_linearization(control_data, flight_phase_detection, env);

        float x0[3] = {0, control_data->reference_error, control_data->integrated_error};
        float C[3][4] = {{0, 1, 0, 0}, 
                         {0, 0, control_data->Ad[0][0], control_data->Ad[0][1]}, 
                         {0, 0, control_data->Ad[1][0], control_data->Ad[1][1]}};
        float D[3][4] = {{1, -1, 0, 0}, 
                         {0, control_data->Bd[0], -1, 0}, 
                         {0, control_data->Bd[1], 0, -1}};
        float cost_H_fin[4][4] = {{control_data->R, 0, 0, 0}, 
                                 {0, control_data->Q[0][0], control_data->Q[0][1], control_data->Q[0][2]}, 
                                 {0, control_data->Q[1][0], control_data->Q[1][1], control_data->Q[1][2]}, 
                                 {0, control_data->Q[2][0], control_data->Q[2][1], control_data->Q[2][2]}};

        memcpy(&control_data->mpc_params.eq_c, &x0, sizeof(x0));
        memcpy(&control_data->mpc_params.eq_C, &C, sizeof(C));
        memcpy(&control_data->mpc_params.eq_D, &D, sizeof(D));
        memcpy(&control_data->mpc_params.cost_H_fin, &cost_H_fin, sizeof(cost_H_fin));

        MPC_embotech_single_integrator_test_20201004181950_maximilianstoelzle_solve(&control_data->mpc_params, &control_data->mpc_output, 
                                                                                    &control_data->mpc_info, NULL);

        control_data->control_input = control_data->mpc_output.u0[0];

        /* Check that the control input is between 0 and 1 */
        control_data->control_input = fmaxf(0, fminf(control_data->control_input, 1));

        compute_integrated_error(control_data);
    }
    else if ((flight_phase_detection->flight_phase == BALLISTIC_DESCENT) && 
            ((flight_phase_detection->mach_regime == SUBSONIC) || (flight_phase_detection->mach_regime == TRANSONIC))) {
        /* actuate airbrakes during ballistic descent to slow down rocket */
        control_data->control_input = 1;
    } else {
        /* This part of the controller is accessed, if the controller should not be operational or if the rocket is the apogee approach phase*/
        /* Airbrakes need to be retracted to prevent entanglement with the parachutes */
        control_data_reset(control_data);
        if (flight_phase_detection->flight_phase == APOGEE_APPROACH || flight_phase_detection->flight_phase == BIAS_RESET) {
            compute_reference_error(control_data);
        }
    }
}

void plant_linearization(control_data_t *control_data, flight_phase_detection_t *flight_phase_detection, env_t *env){
    linear_model(control_data, flight_phase_detection, env);
    discretize(control_data->A, control_data->B, control_data->Ad, control_data->Bd);
}


