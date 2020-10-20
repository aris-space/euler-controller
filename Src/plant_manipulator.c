//
// Created by Tun Kapgen on 04.10.20.
//

#include "../Inc/plant_manipulator.h"


void linear_model(control_data_t *control_data, flight_phase_detection_t *flight_phase_detection, env_t *env){
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

    control_data->A[0][0] = dadv;
    control_data->A[0][1] = 0;
    control_data->A[1][0] = 1;
    control_data->A[1][1] = 0;

    control_data->B[0][0] = dadu;
    control_data->B[1][0] = 0;

}

void get_C_A_AB(float airbrake_extension, flight_phase_detection_t *flight_phase_detection, float *C_A_AB){
    // linearization is done using CFD values
    float v_values[2] = {100.0f, 240.0f};
    float C_A_values[2] = {1.5f, 1.5f};
    float speed_of_sound = 330.0f;
    float mach_values[2] = {v_values[0] / speed_of_sound, v_values[1] / speed_of_sound};
    float C_A_AB_full = 0.0f;
    interpolate(C_A_values, mach_values, flight_phase_detection->mach_number, &C_A_AB_full);
    float actual_C_A_AB_range[2] = {0.0f, C_A_AB_full};
    float airbrake_range[2] = {0.0f, 1.0f};
    interpolate(actual_C_A_AB_range, airbrake_range, airbrake_extension, C_A_AB);
}


void get_C_A_rocket(flight_phase_detection_t *flight_phase_detection, float *C_A_rocket){
    // Assumption made that the angle of attack is zero
    const float C_A_values[18] = {0.48580f, 0.41880f, 0.43808f, 0.43710f, 0.43287f, 0.42853f, 0.42567f, 0.42327f, 0.42136f, 0.41995f, 0.41955f, 0.42033f, 0.42376f, 0.42965f, 0.43539f, 0.43802f, 0.44259f, 0.45489f};
    const float mach_dim[18] = {0.01000f, 0.06000f, 0.11000f, 0.16000f, 0.21000f, 0.26000f, 0.31000f, 0.36000f, 0.41000f, 0.46000f, 0.51000f, 0.56000f, 0.61000f, 0.66000f, 0.71000f, 0.76000f, 0.81000f, 0.86000f};

    for (int i = 0; i < (int)(sizeof(mach_dim)/sizeof(float)); i++ ){
        if (flight_phase_detection->mach_number <= mach_dim[i]){
            *C_A_rocket = C_A_values[i];
            break;
        }
    }
}