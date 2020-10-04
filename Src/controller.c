//
// Created by Tun Kapgen on 05.06.20.
//

#include "../Inc/controller.h"

#if CONTROLLER_TYPE == 0
    #include "../Src/test_controller.c" 
#elif CONTROLLER_TYPE == 1
    #include "../Src/lqr_controller.c"
#endif

void control_data_init(control_data_t *control_data){
    control_data_reset(control_data);

    control_data->sf_velocity = 0;
    control_data->sf_ref_altitude_AGL = 0;
    control_data->tracking_feedback = 0;

    control_data->lowerboundary_aw = 0;
    control_data->upperboundary_aw = 0;

    #if CONTROLLER_TYPE == 1
        init_gains_coeff(control_data);

        for(int i = 0; i < NUM_GAINS; i++){
            control_data->gains[i] = 0;
        }
    #endif
}

void control_data_reset(control_data_t *control_data){
    control_data->control_input = 0;
    control_data->reference_error = 0;
    control_data->integrated_error = 0;
}

void control_step(control_data_t *control_data, state_est_data_t *state_est_data, flight_phase_detection_t *flight_phase_detection) {
    /* Update the control data struct*/
    control_data->sf_velocity = ((float)state_est_data->velocity_rocket[0]) / 1000;
    control_data->sf_ref_altitude_AGL = ((float) state_est_data->position_world[2]) / 1000;
    control_data->tracking_feedback = ((float) state_est_data->airbrake_extension) / 1000000;

    eval_optimal_trajectory_polyfit(control_data);
    compute_control_input(control_data, flight_phase_detection);
}

void eval_optimal_trajectory_polyfit(control_data_t *control_data) {
    /* For Speed */
    double x_placeholder = 0;

    /* Reset ref_velocity_placeholder*/
    double ref_velocity_placeholder = 0;

    control_data->ref_velocity = (float)ref_velocity_placeholder;

    /* For loop */
    for (int i = 0; i < POLY_DEG + 1; ++i) {
        x_placeholder = pow(control_data->sf_ref_altitude_AGL, (double)(POLY_DEG - i));
        ref_velocity_placeholder += (control_data->poly_coeff[3][i] * x_placeholder);
    }
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

#ifdef EULER_SIMCON
void save_evaluated_polyfits_to_file(control_data_t *control_data){
    int lin_num = 10000;
    FILE *fptr;
    float delta_lin = (float)TARGET_AGOGEE / (float)lin_num;
    fptr = fopen ("plotting/data/coeff_data.csv", "w+");
    for (int j = 0; j < lin_num+1; j++){
        control_data->sf_ref_altitude_AGL = delta_lin * j;
        eval_gains_polyfit(control_data);
        fprintf(fptr, "%f,%f,%f,%f,%f\n", control_data->sf_ref_altitude_AGL,
                control_data->ref_velocity, control_data->gains[0], control_data->gains[1], control_data->gains[2]);
    }
    fprintf(fptr,"\n");
    fclose(fptr);
}
#endif
