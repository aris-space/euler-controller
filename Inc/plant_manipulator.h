//
// Created by Tun Kapgen on 05.10.20.
//

#ifndef C_IMPLEMENTATION_PLANT_MANIPULATOR_H
#define C_IMPLEMENTATION_PLANT_MANIPULATOR_H

#include "controller.h"
#include "plant_manipulator_const.h"
#include "../Inc/Util/util.h"
#include "../Inc/Util/math_utils.h"

void linear_model(control_data_t *control_data, flight_phase_detection_t *flight_phase_detection, float A[2][2], float B[2]);
void get_C_A_rocket(flight_phase_detection_t *flight_phase_detection, float *C_A_rocket);
void get_C_A_AB(float airbrake_extension, flight_phase_detection_t *flight_phase_detection, float *C_A_AB);
void discretize(float A[2][2], float B[2], float Ad[2][2], float Bd[2]);

#endif //C_IMPLEMENTATION_PLANT_MANIPULATOR_H
