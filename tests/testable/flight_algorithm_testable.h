/**
 * @file flight_algorithm_testable.h
 * @brief Flight algorithm state machine - Test edilebilir versiyon
 * @note HAL_GetTick() yerine parametre olarak tick alır
 */

#ifndef FLIGHT_ALGORITHM_TESTABLE_H
#define FLIGHT_ALGORITHM_TESTABLE_H

#include <stdint.h>

/* ============================================================================
 *                         FLIGHT PHASES
 * ============================================================================ */

typedef enum {
    PHASE_IDLE = 0,
    PHASE_BOOST,
    PHASE_COAST,
    PHASE_APOGEE,
    PHASE_DROGUE_DESCENT,
    PHASE_MAIN_DESCENT,
    PHASE_LANDED,
    PHASE_ABORT
} FlightPhase_Testable_t;

/* ============================================================================
 *                         STATUS BITS
 * ============================================================================ */

#define BIT_LAUNCH_DETECTED       0x0001
#define BIT_BURNOUT_DETECTED      0x0002
#define BIT_MIN_ALTITUDE_ARMED    0x0004
#define BIT_ANGLE_THRESHOLD       0x0008
#define BIT_APOGEE_DETECTED       0x0010
#define BIT_DROGUE_DEPLOYED       0x0020
#define BIT_MAIN_ALTITUDE         0x0040
#define BIT_MAIN_DEPLOYED         0x0080
#define BIT_LANDED                0x0100

/* ============================================================================
 *                         SENSOR INPUT STRUCTURES
 * ============================================================================ */

/**
 * @brief IMU sample (test için sadeleştirilmiş)
 */
typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float theta;
} FlightIMU_Testable_t;

/**
 * @brief Barometer sample
 */
typedef struct {
    float altitude;
    float pressure;
} FlightBaro_Testable_t;

/**
 * @brief Fused sensor sample
 */
typedef struct {
    float filtered_altitude;
    float velocity;
} FlightFused_Testable_t;

/* ============================================================================
 *                         CONFIG STRUCTURE
 * ============================================================================ */

typedef struct {
    float launch_accel_threshold;
    float launch_velocity_threshold;
    float min_arming_altitude;
    float main_chute_altitude;
    float max_angle_threshold;
    float landing_velocity_threshold;
    uint32_t burnout_timeout_ms;
    uint16_t apogee_confirm_count;
    uint16_t burnout_confirm_count;
    uint16_t landing_confirm_count;
} FlightConfig_Testable_t;

/* ============================================================================
 *                         STATE STRUCTURE
 * ============================================================================ */

typedef struct {
    FlightPhase_Testable_t current_phase;
    FlightPhase_Testable_t previous_phase;
    
    uint16_t status_bits;
    uint8_t is_armed;
    uint8_t drogue_deployed;
    uint8_t main_deployed;
    
    uint16_t apogee_counter;
    uint16_t burnout_counter;
    uint16_t landing_counter;
    
    uint32_t flight_start_tick;
    uint32_t phase_start_tick;
    
    float prev_velocity;
    float prev_altitude;
    
    /* Event callbacks (test için) */
    int drogue_deploy_called;
    int main_deploy_called;
} FlightState_Testable_t;

/* ============================================================================
 *                         PUBLIC FUNCTIONS
 * ============================================================================ */

/**
 * @brief State machine başlat
 */
void FlightAlgo_Testable_Init(FlightState_Testable_t* state);

/**
 * @brief Config ayarla
 */
void FlightAlgo_Testable_SetConfig(
    FlightState_Testable_t* state,
    const FlightConfig_Testable_t* config
);

/**
 * @brief Varsayılan config al
 */
FlightConfig_Testable_t FlightAlgo_Testable_GetDefaultConfig(void);

/**
 * @brief State machine güncelle
 * @param state State yapısı
 * @param imu IMU okuması
 * @param baro Barometer okuması
 * @param fused Fused sensor okuması
 * @param current_tick Mevcut tick (HAL_GetTick yerine)
 */
void FlightAlgo_Testable_Update(
    FlightState_Testable_t* state,
    const FlightConfig_Testable_t* config,
    const FlightIMU_Testable_t* imu,
    const FlightBaro_Testable_t* baro,
    const FlightFused_Testable_t* fused,
    uint32_t current_tick
);

/**
 * @brief Mevcut phase al
 */
FlightPhase_Testable_t FlightAlgo_Testable_GetPhase(const FlightState_Testable_t* state);

/**
 * @brief Status bits al
 */
uint16_t FlightAlgo_Testable_GetStatusBits(const FlightState_Testable_t* state);

/**
 * @brief State reset
 */
void FlightAlgo_Testable_Reset(FlightState_Testable_t* state);

/**
 * @brief Force phase transition (test için)
 */
void FlightAlgo_Testable_ForcePhase(
    FlightState_Testable_t* state, 
    FlightPhase_Testable_t phase,
    uint32_t tick
);

/* ============================================================================
 *                         CONDITION CHECK FUNCTIONS (TEST İÇİN EXPOSE)
 * ============================================================================ */

/**
 * @brief Launch koşulu kontrol
 */
uint8_t FlightAlgo_Testable_CheckLaunch(
    const FlightConfig_Testable_t* config,
    const FlightIMU_Testable_t* imu,
    const FlightFused_Testable_t* fused
);

/**
 * @brief Burnout koşulu kontrol
 */
uint8_t FlightAlgo_Testable_CheckBurnout(
    FlightState_Testable_t* state,
    const FlightConfig_Testable_t* config,
    const FlightIMU_Testable_t* imu,
    uint32_t current_tick
);

/**
 * @brief Apogee koşulu kontrol
 */
uint8_t FlightAlgo_Testable_CheckApogee(
    FlightState_Testable_t* state,
    const FlightConfig_Testable_t* config,
    const FlightFused_Testable_t* fused
);

/**
 * @brief Landing koşulu kontrol
 */
uint8_t FlightAlgo_Testable_CheckLanding(
    FlightState_Testable_t* state,
    const FlightConfig_Testable_t* config,
    const FlightFused_Testable_t* fused
);

/**
 * @brief Total acceleration hesapla
 */
float FlightAlgo_Testable_CalcTotalAccel(const FlightIMU_Testable_t* imu);

#endif /* FLIGHT_ALGORITHM_TESTABLE_H */
