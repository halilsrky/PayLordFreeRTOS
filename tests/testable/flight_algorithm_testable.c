/**
 * @file flight_algorithm_testable.c
 * @brief Flight algorithm - Test edilebilir implementasyon
 */

#include "flight_algorithm_testable.h"
#include <math.h>

/* ============================================================================
 *                         DEFAULT CONFIG
 * ============================================================================ */

static const FlightConfig_Testable_t DEFAULT_CONFIG = {
    .launch_accel_threshold = 45.0f,
    .launch_velocity_threshold = 50.0f,
    .min_arming_altitude = 1800.0f,
    .main_chute_altitude = 500.0f,
    .max_angle_threshold = 70.0f,
    .landing_velocity_threshold = 2.0f,
    .burnout_timeout_ms = 8000,
    .apogee_confirm_count = 9,
    .burnout_confirm_count = 10,
    .landing_confirm_count = 50
};

/* ============================================================================
 *                         PRIVATE FUNCTIONS
 * ============================================================================ */

static void transition_to_phase(
    FlightState_Testable_t* state,
    FlightPhase_Testable_t new_phase,
    uint32_t tick
) {
    if (state->current_phase != new_phase) {
        state->previous_phase = state->current_phase;
        state->current_phase = new_phase;
        state->phase_start_tick = tick;
    }
}

/* ============================================================================
 *                         PUBLIC FUNCTIONS
 * ============================================================================ */

void FlightAlgo_Testable_Init(FlightState_Testable_t* state) {
    state->current_phase = PHASE_IDLE;
    state->previous_phase = PHASE_IDLE;
    state->status_bits = 0;
    state->is_armed = 0;
    state->drogue_deployed = 0;
    state->main_deployed = 0;
    state->apogee_counter = 0;
    state->burnout_counter = 0;
    state->landing_counter = 0;
    state->flight_start_tick = 0;
    state->phase_start_tick = 0;
    state->prev_velocity = 0.0f;
    state->prev_altitude = 0.0f;
    state->drogue_deploy_called = 0;
    state->main_deploy_called = 0;
}

void FlightAlgo_Testable_SetConfig(
    FlightState_Testable_t* state,
    const FlightConfig_Testable_t* config
) {
    (void)state;
    (void)config;
    /* Config stateless olarak kullanılır */
}

FlightConfig_Testable_t FlightAlgo_Testable_GetDefaultConfig(void) {
    return DEFAULT_CONFIG;
}

float FlightAlgo_Testable_CalcTotalAccel(const FlightIMU_Testable_t* imu) {
    return sqrtf(
        imu->accel_x * imu->accel_x +
        imu->accel_y * imu->accel_y +
        imu->accel_z * imu->accel_z
    );
}

uint8_t FlightAlgo_Testable_CheckLaunch(
    const FlightConfig_Testable_t* config,
    const FlightIMU_Testable_t* imu,
    const FlightFused_Testable_t* fused
) {
    float total_accel = FlightAlgo_Testable_CalcTotalAccel(imu);
    
    if (total_accel > config->launch_accel_threshold) {
        return 1;
    }
    
    if (fused->velocity > config->launch_velocity_threshold) {
        return 1;
    }
    
    return 0;
}

uint8_t FlightAlgo_Testable_CheckBurnout(
    FlightState_Testable_t* state,
    const FlightConfig_Testable_t* config,
    const FlightIMU_Testable_t* imu,
    uint32_t current_tick
) {
    uint32_t elapsed = current_tick - state->flight_start_tick;
    
    if (elapsed > config->burnout_timeout_ms) {
        return 1;
    }
    
    if (imu->accel_x < 0.0f) {
        state->burnout_counter++;
        if (state->burnout_counter >= config->burnout_confirm_count) {
            return 1;
        }
    } else {
        if (state->burnout_counter > 0) {
            state->burnout_counter--;
        }
    }
    
    return 0;
}

uint8_t FlightAlgo_Testable_CheckApogee(
    FlightState_Testable_t* state,
    const FlightConfig_Testable_t* config,
    const FlightFused_Testable_t* fused
) {
    if (fused->velocity < 0.0f && fused->velocity < state->prev_velocity) {
        state->apogee_counter++;
        if (state->apogee_counter >= config->apogee_confirm_count) {
            return 1;
        }
    } else {
        if (state->apogee_counter > 0) {
            state->apogee_counter--;
        }
    }
    
    return 0;
}

uint8_t FlightAlgo_Testable_CheckLanding(
    FlightState_Testable_t* state,
    const FlightConfig_Testable_t* config,
    const FlightFused_Testable_t* fused
) {
    if (fabsf(fused->velocity) < config->landing_velocity_threshold) {
        state->landing_counter++;
        if (state->landing_counter >= config->landing_confirm_count) {
            return 1;
        }
    } else {
        state->landing_counter = 0;
    }
    
    return 0;
}

void FlightAlgo_Testable_Update(
    FlightState_Testable_t* state,
    const FlightConfig_Testable_t* config,
    const FlightIMU_Testable_t* imu,
    const FlightBaro_Testable_t* baro,
    const FlightFused_Testable_t* fused,
    uint32_t current_tick
) {
    switch (state->current_phase) {
        
        case PHASE_IDLE:
            if (FlightAlgo_Testable_CheckLaunch(config, imu, fused)) {
                state->flight_start_tick = current_tick;
                state->status_bits |= BIT_LAUNCH_DETECTED;
                transition_to_phase(state, PHASE_BOOST, current_tick);
            }
            break;
            
        case PHASE_BOOST:
            if (FlightAlgo_Testable_CheckBurnout(state, config, imu, current_tick)) {
                state->status_bits |= BIT_BURNOUT_DETECTED;
                transition_to_phase(state, PHASE_COAST, current_tick);
            }
            break;
            
        case PHASE_COAST:
            /* Arming altitude check */
            if (fused->filtered_altitude > config->min_arming_altitude) {
                state->is_armed = 1;
                state->status_bits |= BIT_MIN_ALTITUDE_ARMED;
            }
            
            /* Angle threshold check */
            if (fabsf(imu->theta) > config->max_angle_threshold && state->is_armed) {
                state->status_bits |= BIT_ANGLE_THRESHOLD;
                state->status_bits |= BIT_APOGEE_DETECTED;
                state->drogue_deployed = 1;
                state->drogue_deploy_called = 1;
                state->status_bits |= BIT_DROGUE_DEPLOYED;
                transition_to_phase(state, PHASE_DROGUE_DESCENT, current_tick);
                break;
            }
            
            /* Apogee check */
            if (FlightAlgo_Testable_CheckApogee(state, config, fused)) {
                state->status_bits |= BIT_APOGEE_DETECTED;
                transition_to_phase(state, PHASE_APOGEE, current_tick);
            }
            
            state->prev_velocity = fused->velocity;
            break;
            
        case PHASE_APOGEE:
            /* Deploy drogue */
            if (!state->drogue_deployed) {
                state->drogue_deployed = 1;
                state->drogue_deploy_called = 1;
                state->status_bits |= BIT_DROGUE_DEPLOYED;
            }
            transition_to_phase(state, PHASE_DROGUE_DESCENT, current_tick);
            break;
            
        case PHASE_DROGUE_DESCENT:
            /* Main altitude check */
            if (baro->altitude < config->main_chute_altitude) {
                state->status_bits |= BIT_MAIN_ALTITUDE;
                if (!state->main_deployed) {
                    state->main_deployed = 1;
                    state->main_deploy_called = 1;
                    state->status_bits |= BIT_MAIN_DEPLOYED;
                }
                transition_to_phase(state, PHASE_MAIN_DESCENT, current_tick);
            }
            break;
            
        case PHASE_MAIN_DESCENT:
            if (FlightAlgo_Testable_CheckLanding(state, config, fused)) {
                state->status_bits |= BIT_LANDED;
                transition_to_phase(state, PHASE_LANDED, current_tick);
            }
            break;
            
        case PHASE_LANDED:
            /* Final state */
            break;
            
        case PHASE_ABORT:
            /* Emergency state */
            break;
    }
    
    state->prev_altitude = fused->filtered_altitude;
}

FlightPhase_Testable_t FlightAlgo_Testable_GetPhase(const FlightState_Testable_t* state) {
    return state->current_phase;
}

uint16_t FlightAlgo_Testable_GetStatusBits(const FlightState_Testable_t* state) {
    return state->status_bits;
}

void FlightAlgo_Testable_Reset(FlightState_Testable_t* state) {
    FlightAlgo_Testable_Init(state);
}

void FlightAlgo_Testable_ForcePhase(
    FlightState_Testable_t* state, 
    FlightPhase_Testable_t phase,
    uint32_t tick
) {
    transition_to_phase(state, phase, tick);
}
