#include "../Inc/mpc_controller.h"

void init_params(control_data_t *control_data) {
    const float Q[3][3] = {{1.e-02, 0.e+00, 0.e+00}, {0.e+00, 1.e+03, 0.e+00}, {0.e+00, 0.e+00, 1.e+01}};
    control_data->R = 1000000;

    memcpy(control_data->Q, Q, sizeof(Q));
}

void compute_control_input(control_data_t *control_data, flight_phase_detection_t *flight_phase_detection){
    if (flight_phase_detection->flight_phase == CONTROL) {

        /* Calculate Velocity Error */
        compute_reference_error(control_data);

        float Ad[2][2] = {0};
        float Bd[2] = {0};

        float C[3][4] = {{0, 1, 0, 0}, {0, 0, Ad[0][0], Ad[0][1]}, {0, 0, Ad[1][0], Ad[1][1]}};
        float D[3][4] = {{1, -1, 0, 0}, {0, Bd[0], -1, 0}, {0, Bd[1], 0, -1}};
        float cost_H[4][4] = {{control_data->R, 0, 0, 0}, 
                              {0, control_data->Q[0][0], control_data->Q[0][1], control_data->Q[0][2]}, 
                              {0, control_data->Q[1][0], control_data->Q[1][1], control_data->Q[1][2]}, 
                              {0, control_data->Q[2][0], control_data->Q[2][1], control_data->Q[2][2]}};

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
            eval_gains_polyfit(control_data);
            compute_reference_error(control_data);
        }
    }
}
