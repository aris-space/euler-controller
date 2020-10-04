#ifndef LQR_CONTROLLER_H
#define LQR_CONTROLLER_H

#include "controller_const.h"

void init_gains_coeff(control_data_t *control_data);
void compute_control_input(control_data_t *control_data, flight_phase_detection_t *current_flight_phase_detection);
void evaluate_lqr_gains_polyfit(control_data_t *control_data);

#endif