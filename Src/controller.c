//
// Created by Tun Kapgen on 05.06.20.
//

#include "../Inc/controller.h"

/* In this file, all the controller related function as the controller itself will be defined */

void compute_control_input(control_data_t *control_data, flight_phase_detection_t *flight_phase_detection){
    if (flight_phase_detection->flight_phase == CONTROL) {

        /* caluclate Gains and Reference velocity for given altitude AGL */
        evaluate_polyfit(control_data);

        /* Calculate Velocity Error */
        compute_reference_error(control_data);

        /* Calculate Control Input */
        control_data->control_input = (float)(-control_data->gains[0] * control_data->reference_error
                - control_data->gains[1] * control_data->integrated_error
                - control_data->gains[2] * (control_data->control_input - OPT_TRAJ_CONTROL_INPUT)
                + control_data->control_input);

        /* Check that the control input is between 0 and 1 */
        control_data->control_input = fmaxf(0, fminf(control_data->control_input, 1));

        /* Compute boundaries for the antiwindup */
        compute_antiwindup_boundaries(control_data);

        /* Compute the integrated error */
        control_data->integrated_error = fmaxf(control_data->lowerboundary_aw, fminf(control_data->integrated_error
        + DELTA_T * control_data->reference_error, control_data->upperboundary_aw));
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
            evaluate_polyfit(control_data);
            compute_reference_error(control_data);
        }
    }
}


void control_data_reset(control_data_t *control_data){
    control_data->control_input = 0;
    control_data->reference_error = 0;
    control_data->integrated_error = 0;
}

void control_data_init(control_data_t *control_data){
    control_data_reset(control_data);

    control_data->lowerboundary_aw = 0;
    control_data->upperboundary_aw = 0;

    init_coeff(control_data);

    for(int i = 0; i < NUM_GAINS; i++){
        control_data->gains[i] = 0;
    }
}

/* Does the Polynomial Calculation of the reference velocity */
void evaluate_polyfit(control_data_t *control_data) {
    /* For Speed */
    double x_placeholder = 0;

    /* Reset gains */
    for (int i = 0; i < NUM_GAINS; i++) {
        control_data->gains[i] = 0;
    }

    /* Reset ref_velocity_placeholder*/
    double ref_velocity_placeholder = 0;

    /* For loop */
    for (int i = 0; i < POLY_DEG + 1; ++i) {
        x_placeholder = pow(control_data->sf_ref_altitude_AGL, (double)(POLY_DEG - i));
        control_data->gains[0] += control_data->poly_coeff[0][i] * x_placeholder;
        control_data->gains[1] += control_data->poly_coeff[1][i] * x_placeholder;
        control_data->gains[2] += control_data->poly_coeff[2][i] * x_placeholder;
        ref_velocity_placeholder += (control_data->poly_coeff[3][i] * x_placeholder);
    }
    control_data->ref_velocity = (float)ref_velocity_placeholder;
}

void compute_antiwindup_boundaries(control_data_t *control_data) {
    control_data->upperboundary_aw = fmaxf(M_AW *
            (CONTROL_DEACTIVATION_ALTITUDE_AGL - control_data->sf_ref_altitude_AGL), MIN_BOUNDARAY_AW);
    if (CONTROL_DEACTIVATION_ALTITUDE_AGL < control_data->sf_ref_altitude_AGL) {
        control_data->upperboundary_aw = 0;
    }
    control_data->lowerboundary_aw = - control_data->upperboundary_aw;
}

void compute_reference_error(control_data_t *control_data) {
    if (control_data->ref_velocity < 0) {
        control_data->reference_error = control_data->sf_velocity;
    }
    else{
        control_data->reference_error = control_data->sf_velocity - control_data->ref_velocity;
    }
}

void save_evaluated_polyfits_to_file(control_data_t *control_data){
    int lin_num = 10000;
    FILE *fptr;
    float delta_lin = (float)TARGET_AGOGEE / (float)lin_num;
    fptr = fopen ("plotting/data/coeff_data.csv", "w+");
    for (int j = 0; j < lin_num+1; j++){
        control_data->sf_ref_altitude_AGL = delta_lin * j;
        evaluate_polyfit(control_data);
        fprintf(fptr, "%f,%f,%f,%f,%f\n", control_data->sf_ref_altitude_AGL,
                control_data->ref_velocity, control_data->gains[0], control_data->gains[1], control_data->gains[2]);
    }
    fprintf(fptr,"\n");
    fclose(fptr);
}

void compute_test_control_input(control_data_t *control_data){
    float control_input = 0.0f;
    if(TEST_CONTROLLER_USE_VELOCITY){
        control_input = 0.5f + 0.5f * control_data->sf_velocity / TEST_CONTROLLER_MAX_VELOCITY;
    }
    else {
        control_input = control_data->sf_ref_altitude_AGL / TEST_CONTROLLER_MAX_ALTITUDE;
    }
    control_data->control_input = fmaxf(fminf(control_input, 1), 0);
}