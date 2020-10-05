//
// Created by Tun Kapgen on 05.10.20.
//

#ifndef C_IMPLEMENTATION_PLANT_MANIPULATOR_H
#define C_IMPLEMENTATION_PLANT_MANIPULATOR_H

#include "controller.h"
#include "../env.h"
#include "plant_manipulator_const.h"

void linear_model(control_data_t *control_data, flight_phase_detection_t *flight_phase_detection, float A[2][2], float B[2]);
void get_C_A_rocket(flight_phase_detection_t *flight_phase_detection, float *C_A_rocket);
void get_C_A_AB(float airbrake_extension, flight_phase_detection_t *flight_phase_detection, float *C_A_AB);
void discretize(float[2][2] A, float[2][2] B);

#endif //C_IMPLEMENTATION_PLANT_MANIPULATOR_H
