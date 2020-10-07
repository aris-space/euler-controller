#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

#include "controller_const.h"
#include "base_controller.h"
#include "plant_manipulator.h"
#include "../../aris-euler-state-estimation/Inc/flight_phase_detection.h"
#include "../../aris-euler-state-estimation/Inc/Util/math_utils.h"
#include SOLVER_HEADER

void init_params(control_data_t *control_data);
void compute_control_input(control_data_t *control_data, flight_phase_detection_t *flight_phase_detection, env_t *env);
void plant_linearization(control_data_t *control_data, flight_phase_detection_t *flight_phase_detection, env_t *env);

#endif