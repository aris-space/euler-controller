#ifndef TEST_CONTROLLER_H
#define TEST_CONTROLLER_H

#include "controller_const.h"

void compute_control_input(control_data_t *control_data, flight_phase_detection_t *current_flight_phase_detection);

#endif