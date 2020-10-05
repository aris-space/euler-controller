//
// Created by Tun Kapgen on 04.10.20.
//

#include "../Inc/plant_manipulator.h"


void linear_model(control_data_t *control_data, flight_phase_detection_t *flight_phase_detection, env_t *env,
                  float A[2][2], float B[2]){
    float C_A_rocket = 0.0f;
    float C_A_AB_steady_state = 0.0f;
    float C_A_AB_full_extension = 0.0f;

    get_C_A_rocket(flight_phase_detection, &C_A_rocket);
    get_C_A_AB(OPT_TRAJ_CONTROL_INPUT, flight_phase_detection, &C_A_AB_steady_state);
    get_C_A_AB(1, flight_phase_detection, &C_A_AB_full_extension);

    float dadv = - (float)((A_REF * env->rho_g * (C_A_rocket + C_A_AB_steady_state) * control_data->sf_velocity * pow((
            (env->T_g - T_GRAD * control_data->sf_ref_altitude_AGL) / env->T_g),(-1 + GRAVITATION / (R_0 * T_GRAD)))) /
                    MASS_DRY);
    float dadu = - (float)((C_A_AB_full_extension * A_REF * env->rho_g * powf(control_data->sf_velocity,2) * pow((
            (env->T_g - T_GRAD * control_data->sf_ref_altitude_AGL) / env->T_g), (-1 + GRAVITATION / (R_0 * T_GRAD)))) /
                    (2 * MASS_DRY));

    A[0][0] = dadv;
    A[0][1] = 0;
    A[1][0] = 1;
    A[1][1] = 0;

    B[0] = dadu;
    B[1] = 0;

}

void discretize(float A[2][2], float B[2], float Ad[2][2], float Bd[2]){
    // Tustin transform
    float lambda = 0.1f; // todo: set this value
    float A_inv[2][2] = {0};
    bool check1 =  inverse(2, A, A_inv, lambda);
    
}

void get_C_A_rocket(flight_phase_detection_t *flight_phase_detection, float *C_A_rocket){
    // Assumption made that the angle of attack is zero
    // todo: loading from txt file needs to be implemented
    *C_A_rocket = 5.1f;
}

void get_C_A_AB(float airbrake_extension, flight_phase_detection_t *flight_phase_detection, float *C_A_AB){
    // linearization is done using CFD values
    float v_values[2] = {100, 240};
    float C_A_values[2] = {1.7f, 1.89f};
    float speed_of_sound = 310.0f;
    float mach_values[2] = {v_values[0] / speed_of_sound, v_values[1] / speed_of_sound};
    float C_A_AB_full = 0.0f;
    interpolate(C_A_values, mach_values, flight_phase_detection->mach_number, &C_A_AB_full);
    float actual_C_A_AB_range[2] = {0.0f, C_A_AB_full};
    float airbrake_range[2] = {0, 1};
    interpolate(actual_C_A_AB_range, airbrake_range, airbrake_extension, C_A_AB);
}