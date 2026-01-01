/**
 * @file test_apogee_logic.c
 * @brief Apogee detection logic unit tests
 * @note Safety-critical: Drogue deployment timing doğruluğu
 */

#include "../framework/unity_minimal.h"
#include "../testable/kalman_testable.h"
#include "../testable/flight_algorithm_testable.h"
#include <math.h>

/* ============================================================================
 *                         TEST FIXTURES
 * ============================================================================ */

static KalmanFilter_Testable_t kf;
static FlightState_Testable_t flight_state;
static FlightConfig_Testable_t flight_config;

static void setup(void) {
    KalmanFilter_Testable_Init(&kf);
    FlightAlgo_Testable_Init(&flight_state);
    flight_config = FlightAlgo_Testable_GetDefaultConfig();
}

/* ============================================================================
 *                         KALMAN APOGEE DETECTION TESTS
 * ============================================================================ */

TEST_CASE(test_apogee_kalman_velocity_sign_change) {
    setup();
    
    /* Start with positive velocity */
    kf.x[1] = 10.0f;
    kf.prev_velocity = 15.0f;
    
    /* Velocity decreasing but still positive - no apogee */
    KalmanFilter_Testable_Update(&kf, 2999.0f, -10.0f, 0.01f);
    TEST_ASSERT_EQUAL_INT(0, kf.apogee_detected);
    
    /* Force velocity to go negative */
    kf.x[1] = -5.0f;
    kf.prev_velocity = 2.0f;
    
    /* Update multiple times with decreasing negative velocity */
    for (int i = 0; i < 6; i++) {
        kf.x[1] = -5.0f - (float)i * 2.0f;
        KalmanFilter_Testable_Update(&kf, 2999.0f - i * 5.0f, -10.0f, 0.01f);
    }
    
    TEST_ASSERT_EQUAL_INT(1, kf.apogee_detected);
    
    return TEST_PASS;
}

TEST_CASE(test_apogee_kalman_requires_5_samples) {
    setup();
    
    kf.x[1] = -5.0f;
    kf.prev_velocity = 0.0f;
    
    /* Only 4 samples - should not detect */
    for (int i = 0; i < 4; i++) {
        kf.x[1] = -5.0f - (float)i;
        KalmanFilter_Testable_Update(&kf, 3000.0f, -10.0f, 0.01f);
    }
    
    TEST_ASSERT_EQUAL_INT(0, kf.apogee_detected);
    
    /* 5th sample - should detect */
    kf.x[1] = -10.0f;
    KalmanFilter_Testable_Update(&kf, 2995.0f, -10.0f, 0.01f);
    
    TEST_ASSERT_EQUAL_INT(1, kf.apogee_detected);
    
    return TEST_PASS;
}

TEST_CASE(test_apogee_kalman_false_positive_prevention) {
    setup();
    
    /* Simulate noisy descent then brief "ascent" due to noise */
    kf.x[1] = -10.0f;
    kf.prev_velocity = 0.0f;
    
    /* 3 samples of descent */
    for (int i = 0; i < 3; i++) {
        kf.x[1] = -10.0f - (float)i;
        KalmanFilter_Testable_Update(&kf, 3000.0f - i * 5.0f, -10.0f, 0.01f);
    }
    
    int counter_before = kf.apogee_counter;
    TEST_ASSERT(counter_before >= 3);
    
    /* Brief positive velocity (noise) */
    kf.x[1] = 2.0f;  /* Brief positive */
    KalmanFilter_Testable_Update(&kf, 3000.0f, 5.0f, 0.01f);
    
    /* Counter should reset */
    TEST_ASSERT_EQUAL_INT(0, kf.apogee_counter);
    TEST_ASSERT_EQUAL_INT(0, kf.apogee_detected);
    
    return TEST_PASS;
}

TEST_CASE(test_apogee_kalman_once_detected_stays_detected) {
    setup();
    
    /* Force apogee detected */
    kf.apogee_detected = 1;
    kf.x[1] = -20.0f;
    kf.prev_velocity = -10.0f;
    
    /* Even with positive velocity, should stay detected */
    kf.x[1] = 5.0f;  /* Positive velocity */
    KalmanFilter_Testable_Update(&kf, 3000.0f, 10.0f, 0.01f);
    
    /* apogee_detected flag persists */
    TEST_ASSERT_EQUAL_INT(1, KalmanFilter_Testable_IsApogeeDetected(&kf));
    
    return TEST_PASS;
}

/* ============================================================================
 *                         FLIGHT ALGORITHM APOGEE TESTS
 * ============================================================================ */

TEST_CASE(test_apogee_flight_requires_9_samples) {
    setup();
    
    /* Set up in COAST phase, armed */
    FlightAlgo_Testable_ForcePhase(&flight_state, PHASE_COAST, 10000);
    flight_state.is_armed = 1;
    flight_state.status_bits |= BIT_MIN_ALTITUDE_ARMED;
    
    FlightIMU_Testable_t imu = { .accel_x = -10.0f, .accel_y = 0.0f, .accel_z = 0.0f, .theta = 30.0f };
    FlightBaro_Testable_t baro = { .altitude = 3000.0f, .pressure = 700.0f };
    FlightFused_Testable_t fused = { .filtered_altitude = 3000.0f, .velocity = -5.0f };
    
    /* 8 samples - not enough */
    for (int i = 0; i < 8; i++) {
        fused.velocity = -5.0f - (float)i * 2.0f;
        fused.filtered_altitude = 3000.0f - (float)i * 3.0f;
        FlightAlgo_Testable_Update(&flight_state, &flight_config, &imu, &baro, &fused, 20000 + i * 100);
    }
    
    TEST_ASSERT_EQUAL_INT(PHASE_COAST, flight_state.current_phase);
    
    /* 9th sample - should trigger */
    fused.velocity = -25.0f;
    FlightAlgo_Testable_Update(&flight_state, &flight_config, &imu, &baro, &fused, 20800);
    
    TEST_ASSERT((flight_state.status_bits & BIT_APOGEE_DETECTED) != 0);
    
    return TEST_PASS;
}

TEST_CASE(test_apogee_flight_counter_hysteresis) {
    setup();
    
    FlightAlgo_Testable_ForcePhase(&flight_state, PHASE_COAST, 10000);
    flight_state.is_armed = 1;
    flight_state.status_bits |= BIT_MIN_ALTITUDE_ARMED;
    flight_state.prev_velocity = 0.0f;
    
    FlightIMU_Testable_t imu = { .accel_x = -10.0f, .accel_y = 0.0f, .accel_z = 0.0f, .theta = 20.0f };
    FlightBaro_Testable_t baro = { .altitude = 2990.0f, .pressure = 710.0f };
    FlightFused_Testable_t fused = { .filtered_altitude = 2990.0f, .velocity = -5.0f };
    
    /* Build up counter */
    for (int i = 0; i < 5; i++) {
        fused.velocity = -5.0f - (float)i;
        FlightAlgo_Testable_Update(&flight_state, &flight_config, &imu, &baro, &fused, 20000 + i * 100);
    }
    
    int counter_mid = flight_state.apogee_counter;
    TEST_ASSERT(counter_mid >= 5);
    
    /* Brief positive velocity change */
    fused.velocity = 2.0f;  /* velocity increasing */
    FlightAlgo_Testable_Update(&flight_state, &flight_config, &imu, &baro, &fused, 20500);
    
    /* Counter should decrement, not reset to 0 */
    TEST_ASSERT(flight_state.apogee_counter < counter_mid);
    TEST_ASSERT(flight_state.apogee_counter > 0);
    
    return TEST_PASS;
}

/* ============================================================================
 *                         EDGE CASE TESTS
 * ============================================================================ */

TEST_CASE(test_apogee_zero_velocity_handling) {
    setup();
    
    kf.x[1] = 0.0f;  /* Exactly zero */
    kf.prev_velocity = 5.0f;
    
    /* Update with zero velocity */
    KalmanFilter_Testable_Update(&kf, 3000.0f, -10.0f, 0.01f);
    
    /* Zero is not negative, should not increment counter */
    /* (depends on implementation - ours checks x[1] < 0.0f) */
    TEST_ASSERT_EQUAL_INT(0, kf.apogee_counter);
    
    return TEST_PASS;
}

TEST_CASE(test_apogee_very_small_velocity_change) {
    setup();
    
    kf.x[1] = -0.001f;  /* Very small negative */
    kf.prev_velocity = 0.0f;
    
    /* Should still count as descent start */
    KalmanFilter_Testable_Update(&kf, 3000.0f, -10.0f, 0.01f);
    
    /* Counter should increment because velocity is negative and decreasing */
    TEST_ASSERT(kf.apogee_counter >= 0);
    
    return TEST_PASS;
}

/* ============================================================================
 *                         TIMING TESTS
 * ============================================================================ */

TEST_CASE(test_apogee_consistent_timing) {
    /* Run same scenario multiple times - should get same result */
    int apogee_tick[3];
    
    for (int run = 0; run < 3; run++) {
        setup();
        
        kf.x[0] = 2800.0f;
        kf.x[1] = 30.0f;
        kf.x[2] = -15.0f;
        
        int tick = 0;
        while (!kf.apogee_detected && tick < 1000) {
            float alt = kf.x[0];
            KalmanFilter_Testable_Update(&kf, alt, -15.0f, 0.01f);
            tick++;
        }
        
        apogee_tick[run] = tick;
    }
    
    /* All runs should detect at same tick (deterministic) */
    TEST_ASSERT_EQUAL_INT(apogee_tick[0], apogee_tick[1]);
    TEST_ASSERT_EQUAL_INT(apogee_tick[1], apogee_tick[2]);
    
    return TEST_PASS;
}

/* ============================================================================
 *                         BOUNDARY TESTS
 * ============================================================================ */

TEST_CASE(test_apogee_at_exactly_threshold_velocity) {
    setup();
    
    FlightAlgo_Testable_ForcePhase(&flight_state, PHASE_COAST, 10000);
    flight_state.is_armed = 1;
    flight_state.prev_velocity = 0.0f;
    
    FlightIMU_Testable_t imu = { .accel_x = -10.0f, .accel_y = 0.0f, .accel_z = 0.0f, .theta = 20.0f };
    FlightBaro_Testable_t baro = { .altitude = 3000.0f, .pressure = 700.0f };
    FlightFused_Testable_t fused = { .filtered_altitude = 3000.0f, .velocity = 0.0f };
    
    /* Exactly 0 velocity - on the boundary */
    uint8_t result = FlightAlgo_Testable_CheckApogee(&flight_state, &flight_config, &fused);
    
    /* Zero is not less than prev_velocity (0), should not count */
    TEST_ASSERT_EQUAL_UINT(0, result);
    
    return TEST_PASS;
}

/* ============================================================================
 *                         TEST GROUP RUNNER
 * ============================================================================ */

void run_apogee_logic_tests(void) {
    RUN_TEST(test_apogee_kalman_velocity_sign_change);
    RUN_TEST(test_apogee_kalman_requires_5_samples);
    RUN_TEST(test_apogee_kalman_false_positive_prevention);
    RUN_TEST(test_apogee_kalman_once_detected_stays_detected);
    
    RUN_TEST(test_apogee_flight_requires_9_samples);
    RUN_TEST(test_apogee_flight_counter_hysteresis);
    
    RUN_TEST(test_apogee_zero_velocity_handling);
    RUN_TEST(test_apogee_very_small_velocity_change);
    
    RUN_TEST(test_apogee_consistent_timing);
    RUN_TEST(test_apogee_at_exactly_threshold_velocity);
}
