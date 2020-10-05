//
// Created by Tun Kapgen on 05.06.20.
//

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "../../aris-euler-state-estimation/Inc/env.h"
#include "../../aris-euler-state-estimation/Inc/flight_phase_detection.h"
#include "../../aris-euler-state-estimation/Inc/state_est_const.h"
#include "controller_const.h"

/* function declaration */
void control_init(control_data_t *control_data);
void control_data_reset(control_data_t *control_data);
void control_step(control_data_t *control_data, state_est_data_t *state_est_data, flight_phase_detection_t *flight_phase_detection, env_t *env);
void eval_optimal_trajectory_polyfit(control_data_t *control_data);
void compute_reference_error(control_data_t *control_data);
void compute_integrated_error(control_data_t *control_data);
#if defined(EULER_SIMCON) && CONTROLLER_TYPE == 1
void save_evaluated_polyfits_to_file(control_data_t *control_data);
#endif

#endif //CONTROLLER_H
