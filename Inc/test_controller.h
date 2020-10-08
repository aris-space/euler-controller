#ifndef TEST_CONTROLLER_H
#define TEST_CONTROLLER_H

#include "controller_const.h"
#include "../../aris-euler-state-estimation/Inc/flight_phase_detection.h"

void compute_control_input(control_data_t *control_data, flight_phase_detection_t *flight_phase_detection);

#endif