/**
 * @file flight_state_vectors.h
 * @brief Flight state machine test vektörleri
 */

#ifndef FLIGHT_STATE_VECTORS_H
#define FLIGHT_STATE_VECTORS_H

#include "../testable/flight_algorithm_testable.h"

/* ============================================================================
 *                         TEST STEP STRUCTURE
 * ============================================================================ */

typedef struct {
    /* Inputs */
    float accel_x;
    float accel_y;
    float accel_z;
    float theta;
    float baro_altitude;
    float fused_altitude;
    float fused_velocity;
    uint32_t tick;
    
    /* Expected outputs */
    FlightPhase_Testable_t expected_phase;
    uint16_t expected_status_mask;   /* Bu bitler SET olmalı */
    int expect_drogue_deploy;
    int expect_main_deploy;
} FlightStateTestStep_t;

typedef struct {
    const char* name;
    const FlightStateTestStep_t* steps;
    int count;
} FlightStateTestScenario_t;

/* ============================================================================
 *                         SCENARIO 1: NOMINAL FLIGHT
 * ============================================================================ */

static const FlightStateTestStep_t FLIGHT_NOMINAL_STEPS[] = {
    /* Step 0: IDLE - Rampa üstünde bekleme */
    {
        .accel_x = 0.0f, .accel_y = 0.0f, .accel_z = 9.8f,
        .theta = 0.0f, .baro_altitude = 0.0f,
        .fused_altitude = 0.0f, .fused_velocity = 0.0f,
        .tick = 0,
        .expected_phase = PHASE_IDLE,
        .expected_status_mask = 0,
        .expect_drogue_deploy = 0, .expect_main_deploy = 0
    },
    
    /* Step 1: LAUNCH DETECTED - Yüksek ivme */
    {
        .accel_x = 60.0f, .accel_y = 0.0f, .accel_z = 9.8f,
        .theta = 0.0f, .baro_altitude = 5.0f,
        .fused_altitude = 5.0f, .fused_velocity = 20.0f,
        .tick = 100,
        .expected_phase = PHASE_BOOST,
        .expected_status_mask = BIT_LAUNCH_DETECTED,
        .expect_drogue_deploy = 0, .expect_main_deploy = 0
    },
    
    /* Step 2: BOOST - Motor yanıyor */
    {
        .accel_x = 70.0f, .accel_y = 0.0f, .accel_z = 9.8f,
        .theta = 5.0f, .baro_altitude = 200.0f,
        .fused_altitude = 200.0f, .fused_velocity = 150.0f,
        .tick = 2000,
        .expected_phase = PHASE_BOOST,
        .expected_status_mask = BIT_LAUNCH_DETECTED,
        .expect_drogue_deploy = 0, .expect_main_deploy = 0
    },
    
    /* Step 3: BURNOUT - Motor bitti (timeout ile) */
    {
        .accel_x = -5.0f, .accel_y = 0.0f, .accel_z = 9.8f,
        .theta = 10.0f, .baro_altitude = 1000.0f,
        .fused_altitude = 1000.0f, .fused_velocity = 200.0f,
        .tick = 9000,  /* 8000ms timeout aşıldı */
        .expected_phase = PHASE_COAST,
        .expected_status_mask = BIT_LAUNCH_DETECTED | BIT_BURNOUT_DETECTED,
        .expect_drogue_deploy = 0, .expect_main_deploy = 0
    },
    
    /* Step 4: COAST - Arming altitude'a ulaşma */
    {
        .accel_x = -10.0f, .accel_y = 0.0f, .accel_z = 0.0f,
        .theta = 15.0f, .baro_altitude = 2000.0f,
        .fused_altitude = 2000.0f, .fused_velocity = 100.0f,
        .tick = 15000,
        .expected_phase = PHASE_COAST,
        .expected_status_mask = BIT_LAUNCH_DETECTED | BIT_BURNOUT_DETECTED | BIT_MIN_ALTITUDE_ARMED,
        .expect_drogue_deploy = 0, .expect_main_deploy = 0
    },
    
    /* Step 5-12: APOGEE CONFIRM - 9 ardışık negatif velocity */
    {
        .accel_x = -10.0f, .accel_y = 0.0f, .accel_z = 0.0f,
        .theta = 20.0f, .baro_altitude = 3000.0f,
        .fused_altitude = 3000.0f, .fused_velocity = -1.0f,
        .tick = 20000,
        .expected_phase = PHASE_COAST,
        .expected_status_mask = BIT_LAUNCH_DETECTED | BIT_BURNOUT_DETECTED | BIT_MIN_ALTITUDE_ARMED,
        .expect_drogue_deploy = 0, .expect_main_deploy = 0
    },
    {
        .accel_x = -10.0f, .accel_y = 0.0f, .accel_z = 0.0f,
        .theta = 20.0f, .baro_altitude = 2998.0f,
        .fused_altitude = 2998.0f, .fused_velocity = -3.0f,
        .tick = 20100,
        .expected_phase = PHASE_COAST,
        .expected_status_mask = BIT_LAUNCH_DETECTED | BIT_BURNOUT_DETECTED | BIT_MIN_ALTITUDE_ARMED,
        .expect_drogue_deploy = 0, .expect_main_deploy = 0
    },
    {
        .accel_x = -10.0f, .accel_y = 0.0f, .accel_z = 0.0f,
        .theta = 20.0f, .baro_altitude = 2995.0f,
        .fused_altitude = 2995.0f, .fused_velocity = -5.0f,
        .tick = 20200,
        .expected_phase = PHASE_COAST,
        .expected_status_mask = BIT_LAUNCH_DETECTED | BIT_BURNOUT_DETECTED | BIT_MIN_ALTITUDE_ARMED,
        .expect_drogue_deploy = 0, .expect_main_deploy = 0
    },
    {
        .accel_x = -10.0f, .accel_y = 0.0f, .accel_z = 0.0f,
        .theta = 22.0f, .baro_altitude = 2990.0f,
        .fused_altitude = 2990.0f, .fused_velocity = -8.0f,
        .tick = 20300,
        .expected_phase = PHASE_COAST,
        .expected_status_mask = BIT_LAUNCH_DETECTED | BIT_BURNOUT_DETECTED | BIT_MIN_ALTITUDE_ARMED,
        .expect_drogue_deploy = 0, .expect_main_deploy = 0
    },
    {
        .accel_x = -10.0f, .accel_y = 0.0f, .accel_z = 0.0f,
        .theta = 25.0f, .baro_altitude = 2983.0f,
        .fused_altitude = 2983.0f, .fused_velocity = -12.0f,
        .tick = 20400,
        .expected_phase = PHASE_COAST,
        .expected_status_mask = BIT_LAUNCH_DETECTED | BIT_BURNOUT_DETECTED | BIT_MIN_ALTITUDE_ARMED,
        .expect_drogue_deploy = 0, .expect_main_deploy = 0
    },
    {
        .accel_x = -10.0f, .accel_y = 0.0f, .accel_z = 0.0f,
        .theta = 27.0f, .baro_altitude = 2974.0f,
        .fused_altitude = 2974.0f, .fused_velocity = -15.0f,
        .tick = 20500,
        .expected_phase = PHASE_COAST,
        .expected_status_mask = BIT_LAUNCH_DETECTED | BIT_BURNOUT_DETECTED | BIT_MIN_ALTITUDE_ARMED,
        .expect_drogue_deploy = 0, .expect_main_deploy = 0
    },
    {
        .accel_x = -10.0f, .accel_y = 0.0f, .accel_z = 0.0f,
        .theta = 30.0f, .baro_altitude = 2962.0f,
        .fused_altitude = 2962.0f, .fused_velocity = -18.0f,
        .tick = 20600,
        .expected_phase = PHASE_COAST,
        .expected_status_mask = BIT_LAUNCH_DETECTED | BIT_BURNOUT_DETECTED | BIT_MIN_ALTITUDE_ARMED,
        .expect_drogue_deploy = 0, .expect_main_deploy = 0
    },
    {
        .accel_x = -10.0f, .accel_y = 0.0f, .accel_z = 0.0f,
        .theta = 32.0f, .baro_altitude = 2948.0f,
        .fused_altitude = 2948.0f, .fused_velocity = -22.0f,
        .tick = 20700,
        .expected_phase = PHASE_COAST,
        .expected_status_mask = BIT_LAUNCH_DETECTED | BIT_BURNOUT_DETECTED | BIT_MIN_ALTITUDE_ARMED,
        .expect_drogue_deploy = 0, .expect_main_deploy = 0
    },
    
    /* Step 13: APOGEE DETECTED - 9. sample ile confirm */
    {
        .accel_x = -10.0f, .accel_y = 0.0f, .accel_z = 0.0f,
        .theta = 35.0f, .baro_altitude = 2932.0f,
        .fused_altitude = 2932.0f, .fused_velocity = -26.0f,
        .tick = 20800,
        .expected_phase = PHASE_DROGUE_DESCENT,  /* APOGEE -> DROGUE hızlı geçiş */
        .expected_status_mask = BIT_LAUNCH_DETECTED | BIT_BURNOUT_DETECTED | 
                                BIT_MIN_ALTITUDE_ARMED | BIT_APOGEE_DETECTED | BIT_DROGUE_DEPLOYED,
        .expect_drogue_deploy = 1, .expect_main_deploy = 0
    },
    
    /* Step 14: DROGUE DESCENT - Main altitude'a düşüş */
    {
        .accel_x = -10.0f, .accel_y = 0.0f, .accel_z = 0.0f,
        .theta = 40.0f, .baro_altitude = 600.0f,
        .fused_altitude = 600.0f, .fused_velocity = -30.0f,
        .tick = 80000,
        .expected_phase = PHASE_DROGUE_DESCENT,
        .expected_status_mask = BIT_LAUNCH_DETECTED | BIT_BURNOUT_DETECTED | 
                                BIT_MIN_ALTITUDE_ARMED | BIT_APOGEE_DETECTED | BIT_DROGUE_DEPLOYED,
        .expect_drogue_deploy = 1, .expect_main_deploy = 0
    },
    
    /* Step 15: MAIN DEPLOY - 500m altı */
    {
        .accel_x = -10.0f, .accel_y = 0.0f, .accel_z = 0.0f,
        .theta = 45.0f, .baro_altitude = 450.0f,
        .fused_altitude = 450.0f, .fused_velocity = -25.0f,
        .tick = 85000,
        .expected_phase = PHASE_MAIN_DESCENT,
        .expected_status_mask = BIT_LAUNCH_DETECTED | BIT_BURNOUT_DETECTED | 
                                BIT_MIN_ALTITUDE_ARMED | BIT_APOGEE_DETECTED | 
                                BIT_DROGUE_DEPLOYED | BIT_MAIN_ALTITUDE | BIT_MAIN_DEPLOYED,
        .expect_drogue_deploy = 1, .expect_main_deploy = 1
    },
};

static const FlightStateTestScenario_t FLIGHT_SCENARIO_NOMINAL = {
    .name = "Nominal Flight",
    .steps = FLIGHT_NOMINAL_STEPS,
    .count = 16
};

/* ============================================================================
 *                         SCENARIO 2: ANGLE ABORT
 * ============================================================================ */

static const FlightStateTestStep_t FLIGHT_ANGLE_ABORT_STEPS[] = {
    /* Başlangıç */
    {
        .accel_x = 0.0f, .accel_y = 0.0f, .accel_z = 9.8f,
        .theta = 0.0f, .baro_altitude = 0.0f,
        .fused_altitude = 0.0f, .fused_velocity = 0.0f,
        .tick = 0,
        .expected_phase = PHASE_IDLE,
        .expected_status_mask = 0,
        .expect_drogue_deploy = 0, .expect_main_deploy = 0
    },
    
    /* Launch */
    {
        .accel_x = 50.0f, .accel_y = 0.0f, .accel_z = 9.8f,
        .theta = 5.0f, .baro_altitude = 10.0f,
        .fused_altitude = 10.0f, .fused_velocity = 30.0f,
        .tick = 100,
        .expected_phase = PHASE_BOOST,
        .expected_status_mask = BIT_LAUNCH_DETECTED,
        .expect_drogue_deploy = 0, .expect_main_deploy = 0
    },
    
    /* Burnout */
    {
        .accel_x = -5.0f, .accel_y = 0.0f, .accel_z = 9.8f,
        .theta = 20.0f, .baro_altitude = 1500.0f,
        .fused_altitude = 1500.0f, .fused_velocity = 180.0f,
        .tick = 9000,
        .expected_phase = PHASE_COAST,
        .expected_status_mask = BIT_LAUNCH_DETECTED | BIT_BURNOUT_DETECTED,
        .expect_drogue_deploy = 0, .expect_main_deploy = 0
    },
    
    /* Armed */
    {
        .accel_x = -10.0f, .accel_y = 0.0f, .accel_z = 0.0f,
        .theta = 40.0f, .baro_altitude = 2500.0f,
        .fused_altitude = 2500.0f, .fused_velocity = 80.0f,
        .tick = 15000,
        .expected_phase = PHASE_COAST,
        .expected_status_mask = BIT_LAUNCH_DETECTED | BIT_BURNOUT_DETECTED | BIT_MIN_ALTITUDE_ARMED,
        .expect_drogue_deploy = 0, .expect_main_deploy = 0
    },
    
    /* ANGLE THRESHOLD EXCEEDED - 70° aşıldı */
    {
        .accel_x = -10.0f, .accel_y = 0.0f, .accel_z = 0.0f,
        .theta = 75.0f,  /* > 70° threshold */
        .baro_altitude = 2800.0f,
        .fused_altitude = 2800.0f, .fused_velocity = 50.0f,
        .tick = 17000,
        .expected_phase = PHASE_DROGUE_DESCENT,
        .expected_status_mask = BIT_LAUNCH_DETECTED | BIT_BURNOUT_DETECTED | 
                                BIT_MIN_ALTITUDE_ARMED | BIT_ANGLE_THRESHOLD | 
                                BIT_APOGEE_DETECTED | BIT_DROGUE_DEPLOYED,
        .expect_drogue_deploy = 1, .expect_main_deploy = 0
    },
};

static const FlightStateTestScenario_t FLIGHT_SCENARIO_ANGLE_ABORT = {
    .name = "Angle Threshold Abort",
    .steps = FLIGHT_ANGLE_ABORT_STEPS,
    .count = 5
};

/* ============================================================================
 *                         ALL SCENARIOS
 * ============================================================================ */

static const FlightStateTestScenario_t* FLIGHT_ALL_SCENARIOS[] = {
    &FLIGHT_SCENARIO_NOMINAL,
    &FLIGHT_SCENARIO_ANGLE_ABORT
};

#define FLIGHT_SCENARIO_COUNT (sizeof(FLIGHT_ALL_SCENARIOS) / sizeof(FLIGHT_ALL_SCENARIOS[0]))

#endif /* FLIGHT_STATE_VECTORS_H */
