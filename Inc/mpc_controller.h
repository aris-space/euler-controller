#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

#include "controller_const.h"
#include "controller.h"
#include "plant_manipulator.h"
#include "../../aris-euler-state-estimation/Inc/Util/math_utils.h"

void init_params(control_data_t *control_data);
void compute_control_input(control_data_t *control_data, flight_phase_detection_t *current_flight_phase_detection);
void plant_linearization(control_data_t *control_data, flight_phase_detection_t *flight_phase_detection, float Ad[2][2], float Bd[2]);

#endif