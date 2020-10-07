//
// Created by Tun Kapgen on 05.06.20.
//

#include "../Inc/base_controller.h"


void control_data_reset(control_data_t *control_data){
    control_data->control_input = 0;
    control_data->reference_error = 0;
    control_data->integrated_error = 0;
}

void eval_optimal_trajectory_polyfit(control_data_t *control_data) {
    /* For Speed */
    double x_placeholder = 0;

    /* Reset ref_velocity_placeholder*/
    double ref_velocity_placeholder = 0;

    /* For loop */
    for (int i = 0; i < POLY_DEG + 1; ++i) {
        x_placeholder = pow(control_data->sf_ref_altitude_AGL, (double)(POLY_DEG - i));
        ref_velocity_placeholder += (control_data->optimal_trajectory_coeff[i] * x_placeholder);
    }

    control_data->ref_velocity = (float)ref_velocity_placeholder;
}

void compute_reference_error(control_data_t *control_data) {
    if (control_data->ref_velocity < 0) {
        control_data->reference_error = control_data->sf_velocity;
    }
    else{
        control_data->reference_error = control_data->sf_velocity - control_data->ref_velocity;
    }
}

void compute_integrated_error(control_data_t *control_data) {
    control_data->upperboundary_aw = fmaxf(M_AW *
            (CONTROL_DEACTIVATION_ALTITUDE_AGL - control_data->sf_ref_altitude_AGL), MIN_BOUNDARAY_AW);
    if (CONTROL_DEACTIVATION_ALTITUDE_AGL < control_data->sf_ref_altitude_AGL) {
        control_data->upperboundary_aw = 0;
    }
    control_data->lowerboundary_aw = - control_data->upperboundary_aw;

    /* Compute the integrated error */
    control_data->integrated_error = fmaxf(control_data->lowerboundary_aw, fminf(control_data->integrated_error
    + DELTA_T * control_data->reference_error, control_data->upperboundary_aw));
}
