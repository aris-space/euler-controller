#include "../Inc/test_controller.h"

void compute_control_input(control_data_t *control_data, flight_phase_detection_t *flight_phase_detection){
    float control_input = 0.0f;
    if(TEST_CONTROLLER_USE_VELOCITY){
        control_input = 0.5f + 0.5f * control_data->sf_velocity / TEST_CONTROLLER_MAX_VELOCITY;
    }
    else {
        control_input = control_data->sf_ref_altitude_AGL / TEST_CONTROLLER_MAX_ALTITUDE;
    }
    control_data->control_input = fmaxf(fminf(control_input, 1), 0);
}