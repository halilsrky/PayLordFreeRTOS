/**
 * @file test_flight_algorithm.c
 * @brief Flight state machine unit tests
 * @note Safety-critical: State geçişleri ve paraşüt deployment doğruluğu
 */

#include "../framework/unity_minimal.h"
#include "../testable/flight_algorithm_testable.h"
#include "../vectors/flight_state_vectors.h"
#include <math.h>

/* ============================================================================
 *                         TEST FIXTURES
 * ============================================================================ */

static FlightState_Testable_t test_state;
static FlightConfig_Testable_t test_config;

static void setup(void) {
    FlightAlgo_Testable_Init(&test_state);
    test_config = FlightAlgo_Testable_GetDefaultConfig();
}

static void teardown(void) {
    /* Cleanup */
}

/* ============================================================================
 *                         INITIALIZATION TESTS
 * ============================================================================ */

TEST_CASE(test_flight_init_phase_idle) {
    setup();
    
    TEST_ASSERT_EQUAL_INT(PHASE_IDLE, test_state.current_phase);
    TEST_ASSERT_EQUAL_INT(PHASE_IDLE, test_state.previous_phase);
    
    teardown();
    return TEST_PASS;
}

TEST_CASE(test_flight_init_status_zero) {
    setup();
    
    TEST_ASSERT_EQUAL_UINT(0, test_state.status_bits);
    TEST_ASSERT_EQUAL_UINT(0, test_state.is_armed);
    TEST_ASSERT_EQUAL_UINT(0, test_state.drogue_deployed);
    TEST_ASSERT_EQUAL_UINT(0, test_state.main_deployed);
    
    teardown();
    return TEST_PASS;
}

TEST_CASE(test_flight_init_counters_zero) {
    setup();
    
    TEST_ASSERT_EQUAL_UINT(0, test_state.apogee_counter);
    TEST_ASSERT_EQUAL_UINT(0, test_state.burnout_counter);
    TEST_ASSERT_EQUAL_UINT(0, test_state.landing_counter);
    
    teardown();
    return TEST_PASS;
}

TEST_CASE(test_flight_default_config_values) {
    setup();
    
    TEST_ASSERT_FLOAT_EQUAL(45.0f, test_config.launch_accel_threshold);
    TEST_ASSERT_FLOAT_EQUAL(1800.0f, test_config.min_arming_altitude);
    TEST_ASSERT_FLOAT_EQUAL(500.0f, test_config.main_chute_altitude);
    TEST_ASSERT_FLOAT_EQUAL(70.0f, test_config.max_angle_threshold);
    
    teardown();
    return TEST_PASS;
}

/* ============================================================================
 *                         LAUNCH DETECTION TESTS
 * ============================================================================ */

TEST_CASE(test_flight_no_launch_low_accel) {
    setup();
    
    FlightIMU_Testable_t imu = {
        .accel_x = 10.0f,  /* Below 45 threshold */
        .accel_y = 0.0f,
        .accel_z = 9.8f,
        .theta = 0.0f
    };
    FlightBaro_Testable_t baro = { .altitude = 0.0f, .pressure = 1013.25f };
    FlightFused_Testable_t fused = { .filtered_altitude = 0.0f, .velocity = 0.0f };
    
    FlightAlgo_Testable_Update(&test_state, &test_config, &imu, &baro, &fused, 100);
    
    TEST_ASSERT_EQUAL_INT(PHASE_IDLE, test_state.current_phase);
    TEST_ASSERT((test_state.status_bits & BIT_LAUNCH_DETECTED) == 0);
    
    teardown();
    return TEST_PASS;
}

TEST_CASE(test_flight_launch_detected_high_accel) {
    setup();
    
    FlightIMU_Testable_t imu = {
        .accel_x = 50.0f,  /* Above 45 threshold */
        .accel_y = 0.0f,
        .accel_z = 9.8f,
        .theta = 0.0f
    };
    FlightBaro_Testable_t baro = { .altitude = 5.0f, .pressure = 1013.0f };
    FlightFused_Testable_t fused = { .filtered_altitude = 5.0f, .velocity = 10.0f };
    
    FlightAlgo_Testable_Update(&test_state, &test_config, &imu, &baro, &fused, 100);
    
    TEST_ASSERT_EQUAL_INT(PHASE_BOOST, test_state.current_phase);
    TEST_ASSERT((test_state.status_bits & BIT_LAUNCH_DETECTED) != 0);
    
    teardown();
    return TEST_PASS;
}

TEST_CASE(test_flight_launch_detected_high_velocity) {
    setup();
    
    FlightIMU_Testable_t imu = {
        .accel_x = 20.0f,  /* Below accel threshold */
        .accel_y = 0.0f,
        .accel_z = 9.8f,
        .theta = 0.0f
    };
    FlightBaro_Testable_t baro = { .altitude = 50.0f, .pressure = 1012.0f };
    FlightFused_Testable_t fused = { 
        .filtered_altitude = 50.0f, 
        .velocity = 60.0f  /* Above 50 threshold */
    };
    
    FlightAlgo_Testable_Update(&test_state, &test_config, &imu, &baro, &fused, 100);
    
    TEST_ASSERT_EQUAL_INT(PHASE_BOOST, test_state.current_phase);
    TEST_ASSERT((test_state.status_bits & BIT_LAUNCH_DETECTED) != 0);
    
    teardown();
    return TEST_PASS;
}

/* ============================================================================
 *                         BURNOUT DETECTION TESTS
 * ============================================================================ */

TEST_CASE(test_flight_burnout_timeout) {
    setup();
    
    /* Launch first */
    FlightIMU_Testable_t imu = { .accel_x = 60.0f, .accel_y = 0.0f, .accel_z = 9.8f, .theta = 0.0f };
    FlightBaro_Testable_t baro = { .altitude = 10.0f, .pressure = 1012.0f };
    FlightFused_Testable_t fused = { .filtered_altitude = 10.0f, .velocity = 30.0f };
    
    FlightAlgo_Testable_Update(&test_state, &test_config, &imu, &baro, &fused, 100);
    TEST_ASSERT_EQUAL_INT(PHASE_BOOST, test_state.current_phase);
    
    /* Wait for timeout (8000ms) */
    imu.accel_x = 20.0f;  /* Still positive, no negative accel */
    FlightAlgo_Testable_Update(&test_state, &test_config, &imu, &baro, &fused, 9000);
    
    TEST_ASSERT_EQUAL_INT(PHASE_COAST, test_state.current_phase);
    TEST_ASSERT((test_state.status_bits & BIT_BURNOUT_DETECTED) != 0);
    
    teardown();
    return TEST_PASS;
}

TEST_CASE(test_flight_burnout_negative_accel) {
    setup();
    
    /* Launch */
    FlightIMU_Testable_t imu = { .accel_x = 60.0f, .accel_y = 0.0f, .accel_z = 9.8f, .theta = 0.0f };
    FlightBaro_Testable_t baro = { .altitude = 10.0f, .pressure = 1012.0f };
    FlightFused_Testable_t fused = { .filtered_altitude = 10.0f, .velocity = 30.0f };
    
    FlightAlgo_Testable_Update(&test_state, &test_config, &imu, &baro, &fused, 100);
    
    /* Negative accel for 10 samples (burnout_confirm_count) */
    imu.accel_x = -5.0f;
    for (int i = 0; i < 10; i++) {
        FlightAlgo_Testable_Update(&test_state, &test_config, &imu, &baro, &fused, 200 + i * 10);
    }
    
    TEST_ASSERT_EQUAL_INT(PHASE_COAST, test_state.current_phase);
    
    teardown();
    return TEST_PASS;
}

/* ============================================================================
 *                         ARMING TESTS
 * ============================================================================ */

TEST_CASE(test_flight_armed_at_altitude) {
    setup();
    
    /* Force to COAST phase */
    FlightAlgo_Testable_ForcePhase(&test_state, PHASE_COAST, 1000);
    test_state.status_bits |= BIT_LAUNCH_DETECTED | BIT_BURNOUT_DETECTED;
    
    FlightIMU_Testable_t imu = { .accel_x = -10.0f, .accel_y = 0.0f, .accel_z = 0.0f, .theta = 10.0f };
    FlightBaro_Testable_t baro = { .altitude = 2000.0f, .pressure = 800.0f };
    FlightFused_Testable_t fused = { 
        .filtered_altitude = 2000.0f,  /* Above 1800m arming altitude */
        .velocity = 80.0f 
    };
    
    FlightAlgo_Testable_Update(&test_state, &test_config, &imu, &baro, &fused, 15000);
    
    TEST_ASSERT_EQUAL_UINT(1, test_state.is_armed);
    TEST_ASSERT((test_state.status_bits & BIT_MIN_ALTITUDE_ARMED) != 0);
    
    teardown();
    return TEST_PASS;
}

TEST_CASE(test_flight_not_armed_below_altitude) {
    setup();
    
    FlightAlgo_Testable_ForcePhase(&test_state, PHASE_COAST, 1000);
    
    FlightIMU_Testable_t imu = { .accel_x = -10.0f, .accel_y = 0.0f, .accel_z = 0.0f, .theta = 10.0f };
    FlightBaro_Testable_t baro = { .altitude = 1500.0f, .pressure = 850.0f };
    FlightFused_Testable_t fused = { 
        .filtered_altitude = 1500.0f,  /* Below 1800m arming altitude */
        .velocity = 100.0f 
    };
    
    FlightAlgo_Testable_Update(&test_state, &test_config, &imu, &baro, &fused, 15000);
    
    TEST_ASSERT_EQUAL_UINT(0, test_state.is_armed);
    
    teardown();
    return TEST_PASS;
}

/* ============================================================================
 *                         APOGEE DETECTION TESTS
 * ============================================================================ */

TEST_CASE(test_flight_apogee_after_confirm_count) {
    setup();
    
    FlightAlgo_Testable_ForcePhase(&test_state, PHASE_COAST, 1000);
    test_state.is_armed = 1;
    test_state.status_bits |= BIT_MIN_ALTITUDE_ARMED;
    
    FlightIMU_Testable_t imu = { .accel_x = -10.0f, .accel_y = 0.0f, .accel_z = 0.0f, .theta = 30.0f };
    FlightBaro_Testable_t baro = { .altitude = 3000.0f, .pressure = 700.0f };
    FlightFused_Testable_t fused = { .filtered_altitude = 3000.0f, .velocity = -5.0f };
    
    /* 9 samples with decreasing negative velocity */
    for (int i = 0; i < 9; i++) {
        fused.velocity = -5.0f - (float)i * 2.0f;  /* Decreasing: -5, -7, -9, ... */
        fused.filtered_altitude = 3000.0f - (float)i * 5.0f;
        FlightAlgo_Testable_Update(&test_state, &test_config, &imu, &baro, &fused, 20000 + i * 100);
    }
    
    TEST_ASSERT((test_state.status_bits & BIT_APOGEE_DETECTED) != 0);
    
    teardown();
    return TEST_PASS;
}

/* ============================================================================
 *                         ANGLE THRESHOLD TESTS
 * ============================================================================ */

TEST_CASE(test_flight_angle_threshold_triggers_drogue) {
    setup();
    
    FlightAlgo_Testable_ForcePhase(&test_state, PHASE_COAST, 1000);
    test_state.is_armed = 1;
    test_state.status_bits |= BIT_MIN_ALTITUDE_ARMED;
    
    FlightIMU_Testable_t imu = { 
        .accel_x = -10.0f, .accel_y = 0.0f, .accel_z = 0.0f, 
        .theta = 75.0f  /* > 70° threshold */
    };
    FlightBaro_Testable_t baro = { .altitude = 2800.0f, .pressure = 720.0f };
    FlightFused_Testable_t fused = { .filtered_altitude = 2800.0f, .velocity = 50.0f };
    
    FlightAlgo_Testable_Update(&test_state, &test_config, &imu, &baro, &fused, 17000);
    
    TEST_ASSERT((test_state.status_bits & BIT_ANGLE_THRESHOLD) != 0);
    TEST_ASSERT((test_state.status_bits & BIT_DROGUE_DEPLOYED) != 0);
    TEST_ASSERT_EQUAL_INT(PHASE_DROGUE_DESCENT, test_state.current_phase);
    TEST_ASSERT_EQUAL_INT(1, test_state.drogue_deploy_called);
    
    teardown();
    return TEST_PASS;
}

TEST_CASE(test_flight_angle_not_triggered_when_not_armed) {
    setup();
    
    FlightAlgo_Testable_ForcePhase(&test_state, PHASE_COAST, 1000);
    test_state.is_armed = 0;  /* Not armed */
    
    FlightIMU_Testable_t imu = { 
        .accel_x = -10.0f, .accel_y = 0.0f, .accel_z = 0.0f, 
        .theta = 80.0f  /* > 70° threshold but not armed */
    };
    FlightBaro_Testable_t baro = { .altitude = 1500.0f, .pressure = 850.0f };
    FlightFused_Testable_t fused = { .filtered_altitude = 1500.0f, .velocity = 100.0f };
    
    FlightAlgo_Testable_Update(&test_state, &test_config, &imu, &baro, &fused, 12000);
    
    /* Should NOT trigger drogue because not armed */
    TEST_ASSERT_EQUAL_INT(PHASE_COAST, test_state.current_phase);
    TEST_ASSERT_EQUAL_INT(0, test_state.drogue_deploy_called);
    
    teardown();
    return TEST_PASS;
}

/* ============================================================================
 *                         MAIN CHUTE DEPLOYMENT TESTS
 * ============================================================================ */

TEST_CASE(test_flight_main_deploys_at_altitude) {
    setup();
    
    FlightAlgo_Testable_ForcePhase(&test_state, PHASE_DROGUE_DESCENT, 30000);
    test_state.drogue_deployed = 1;
    test_state.status_bits |= BIT_DROGUE_DEPLOYED;
    
    FlightIMU_Testable_t imu = { .accel_x = -10.0f, .accel_y = 0.0f, .accel_z = 0.0f, .theta = 45.0f };
    FlightBaro_Testable_t baro = { 
        .altitude = 450.0f,  /* Below 500m threshold */
        .pressure = 960.0f 
    };
    FlightFused_Testable_t fused = { .filtered_altitude = 450.0f, .velocity = -25.0f };
    
    FlightAlgo_Testable_Update(&test_state, &test_config, &imu, &baro, &fused, 85000);
    
    TEST_ASSERT((test_state.status_bits & BIT_MAIN_ALTITUDE) != 0);
    TEST_ASSERT((test_state.status_bits & BIT_MAIN_DEPLOYED) != 0);
    TEST_ASSERT_EQUAL_INT(PHASE_MAIN_DESCENT, test_state.current_phase);
    TEST_ASSERT_EQUAL_INT(1, test_state.main_deploy_called);
    
    teardown();
    return TEST_PASS;
}

/* ============================================================================
 *                         LANDING DETECTION TESTS
 * ============================================================================ */

TEST_CASE(test_flight_landing_detected) {
    setup();
    
    FlightAlgo_Testable_ForcePhase(&test_state, PHASE_MAIN_DESCENT, 90000);
    test_state.main_deployed = 1;
    
    FlightIMU_Testable_t imu = { .accel_x = 0.0f, .accel_y = 0.0f, .accel_z = 9.8f, .theta = 0.0f };
    FlightBaro_Testable_t baro = { .altitude = 5.0f, .pressure = 1013.0f };
    FlightFused_Testable_t fused = { 
        .filtered_altitude = 5.0f, 
        .velocity = 0.5f  /* Below 2 m/s threshold */
    };
    
    /* 50 samples with low velocity */
    for (int i = 0; i < 50; i++) {
        FlightAlgo_Testable_Update(&test_state, &test_config, &imu, &baro, &fused, 100000 + i * 10);
    }
    
    TEST_ASSERT((test_state.status_bits & BIT_LANDED) != 0);
    TEST_ASSERT_EQUAL_INT(PHASE_LANDED, test_state.current_phase);
    
    teardown();
    return TEST_PASS;
}

/* ============================================================================
 *                         HELPER FUNCTION TESTS
 * ============================================================================ */

TEST_CASE(test_flight_calc_total_accel) {
    FlightIMU_Testable_t imu = {
        .accel_x = 3.0f,
        .accel_y = 4.0f,
        .accel_z = 0.0f,
        .theta = 0.0f
    };
    
    float total = FlightAlgo_Testable_CalcTotalAccel(&imu);
    
    /* sqrt(3² + 4² + 0²) = 5 */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 5.0f, total);
    
    return TEST_PASS;
}

/* ============================================================================
 *                         STATE TRANSITION TESTS
 * ============================================================================ */

TEST_CASE(test_flight_no_backward_transitions) {
    setup();
    
    /* Once in LANDED, should not go back */
    FlightAlgo_Testable_ForcePhase(&test_state, PHASE_LANDED, 200000);
    test_state.status_bits = 0xFFFF;  /* All bits set */
    
    FlightIMU_Testable_t imu = { .accel_x = 60.0f, .accel_y = 0.0f, .accel_z = 9.8f, .theta = 0.0f };
    FlightBaro_Testable_t baro = { .altitude = 0.0f, .pressure = 1013.0f };
    FlightFused_Testable_t fused = { .filtered_altitude = 0.0f, .velocity = 0.0f };
    
    FlightAlgo_Testable_Update(&test_state, &test_config, &imu, &baro, &fused, 210000);
    
    /* Should stay in LANDED */
    TEST_ASSERT_EQUAL_INT(PHASE_LANDED, test_state.current_phase);
    
    teardown();
    return TEST_PASS;
}

/* ============================================================================
 *                         TEST GROUP RUNNER
 * ============================================================================ */

void run_flight_algorithm_tests(void) {
    RUN_TEST(test_flight_init_phase_idle);
    RUN_TEST(test_flight_init_status_zero);
    RUN_TEST(test_flight_init_counters_zero);
    RUN_TEST(test_flight_default_config_values);
    
    RUN_TEST(test_flight_no_launch_low_accel);
    RUN_TEST(test_flight_launch_detected_high_accel);
    RUN_TEST(test_flight_launch_detected_high_velocity);
    
    RUN_TEST(test_flight_burnout_timeout);
    RUN_TEST(test_flight_burnout_negative_accel);
    
    RUN_TEST(test_flight_armed_at_altitude);
    RUN_TEST(test_flight_not_armed_below_altitude);
    
    RUN_TEST(test_flight_apogee_after_confirm_count);
    
    RUN_TEST(test_flight_angle_threshold_triggers_drogue);
    RUN_TEST(test_flight_angle_not_triggered_when_not_armed);
    
    RUN_TEST(test_flight_main_deploys_at_altitude);
    
    RUN_TEST(test_flight_landing_detected);
    
    RUN_TEST(test_flight_calc_total_accel);
    
    RUN_TEST(test_flight_no_backward_transitions);
}
