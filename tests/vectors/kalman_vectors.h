/**
 * @file kalman_vectors.h
 * @brief Kalman filter test vektörleri
 * @note Deterministik, tekrarlanabilir test senaryoları
 */

#ifndef KALMAN_VECTORS_H
#define KALMAN_VECTORS_H

#include <stdint.h>

/* ============================================================================
 *                         TEST VECTOR STRUCTURE
 * ============================================================================ */

typedef struct {
    float altitude_measured;
    float accel_measured;
    float dt;
    float expected_altitude;
    float expected_velocity;
    float tolerance;
} KalmanTestVector_t;

typedef struct {
    const char* name;
    const KalmanTestVector_t* vectors;
    int count;
    float initial_altitude;
    float initial_velocity;
} KalmanTestScenario_t;

/* ============================================================================
 *                         SCENARIO 1: STATIC ON GROUND
 * ============================================================================ */

static const KalmanTestVector_t KALMAN_STATIC_VECTORS[] = {
    /* Yerden statik okuma - filter stabil kalmalı */
    { .altitude_measured = 0.0f, .accel_measured = 0.0f, .dt = 0.01f, 
      .expected_altitude = 0.0f, .expected_velocity = 0.0f, .tolerance = 0.5f },
    { .altitude_measured = 0.1f, .accel_measured = 0.0f, .dt = 0.01f, 
      .expected_altitude = 0.0f, .expected_velocity = 0.0f, .tolerance = 0.5f },
    { .altitude_measured = -0.1f, .accel_measured = 0.0f, .dt = 0.01f, 
      .expected_altitude = 0.0f, .expected_velocity = 0.0f, .tolerance = 0.5f },
    { .altitude_measured = 0.0f, .accel_measured = 0.1f, .dt = 0.01f, 
      .expected_altitude = 0.0f, .expected_velocity = 0.0f, .tolerance = 0.5f },
    { .altitude_measured = 0.0f, .accel_measured = -0.1f, .dt = 0.01f, 
      .expected_altitude = 0.0f, .expected_velocity = 0.0f, .tolerance = 0.5f },
};

static const KalmanTestScenario_t KALMAN_SCENARIO_STATIC = {
    .name = "Static Ground",
    .vectors = KALMAN_STATIC_VECTORS,
    .count = 5,
    .initial_altitude = 0.0f,
    .initial_velocity = 0.0f
};

/* ============================================================================
 *                         SCENARIO 2: CONSTANT VELOCITY ASCENT
 * ============================================================================ */

static const KalmanTestVector_t KALMAN_CONST_VEL_VECTORS[] = {
    /* 100 m/s sabit hız ile yükseliş */
    { .altitude_measured = 1.0f, .accel_measured = 0.0f, .dt = 0.01f, 
      .expected_altitude = 1.0f, .expected_velocity = 100.0f, .tolerance = 5.0f },
    { .altitude_measured = 2.0f, .accel_measured = 0.0f, .dt = 0.01f, 
      .expected_altitude = 2.0f, .expected_velocity = 100.0f, .tolerance = 5.0f },
    { .altitude_measured = 3.0f, .accel_measured = 0.0f, .dt = 0.01f, 
      .expected_altitude = 3.0f, .expected_velocity = 100.0f, .tolerance = 5.0f },
    { .altitude_measured = 4.0f, .accel_measured = 0.0f, .dt = 0.01f, 
      .expected_altitude = 4.0f, .expected_velocity = 100.0f, .tolerance = 5.0f },
    { .altitude_measured = 5.0f, .accel_measured = 0.0f, .dt = 0.01f, 
      .expected_altitude = 5.0f, .expected_velocity = 100.0f, .tolerance = 5.0f },
};

static const KalmanTestScenario_t KALMAN_SCENARIO_CONST_VEL = {
    .name = "Constant Velocity Ascent",
    .vectors = KALMAN_CONST_VEL_VECTORS,
    .count = 5,
    .initial_altitude = 0.0f,
    .initial_velocity = 100.0f
};

/* ============================================================================
 *                         SCENARIO 3: APOGEE DETECTION
 * ============================================================================ */

/* Apogee'ye yaklaşım - velocity pozitiften negatife geçiş */
static const KalmanTestVector_t KALMAN_APOGEE_VECTORS[] = {
    /* Yükselirken yavaşlama */
    { .altitude_measured = 2990.0f, .accel_measured = -15.0f, .dt = 0.01f, 
      .expected_altitude = 2990.0f, .expected_velocity = 20.0f, .tolerance = 10.0f },
    { .altitude_measured = 2995.0f, .accel_measured = -15.0f, .dt = 0.01f, 
      .expected_altitude = 2995.0f, .expected_velocity = 10.0f, .tolerance = 10.0f },
    { .altitude_measured = 2998.0f, .accel_measured = -10.0f, .dt = 0.01f, 
      .expected_altitude = 2998.0f, .expected_velocity = 5.0f, .tolerance = 10.0f },
    { .altitude_measured = 2999.0f, .accel_measured = -10.0f, .dt = 0.01f, 
      .expected_altitude = 2999.0f, .expected_velocity = 2.0f, .tolerance = 10.0f },
    { .altitude_measured = 3000.0f, .accel_measured = -10.0f, .dt = 0.01f, 
      .expected_altitude = 3000.0f, .expected_velocity = 0.0f, .tolerance = 10.0f },
    /* Apogee sonrası düşüş */
    { .altitude_measured = 2999.0f, .accel_measured = -10.0f, .dt = 0.01f, 
      .expected_altitude = 2999.0f, .expected_velocity = -2.0f, .tolerance = 10.0f },
    { .altitude_measured = 2997.0f, .accel_measured = -10.0f, .dt = 0.01f, 
      .expected_altitude = 2997.0f, .expected_velocity = -5.0f, .tolerance = 10.0f },
    { .altitude_measured = 2993.0f, .accel_measured = -10.0f, .dt = 0.01f, 
      .expected_altitude = 2993.0f, .expected_velocity = -10.0f, .tolerance = 10.0f },
};

static const KalmanTestScenario_t KALMAN_SCENARIO_APOGEE = {
    .name = "Apogee Detection",
    .vectors = KALMAN_APOGEE_VECTORS,
    .count = 8,
    .initial_altitude = 2980.0f,
    .initial_velocity = 30.0f
};

/* ============================================================================
 *                         SCENARIO 4: HIGH ACCELERATION (BOOST)
 * ============================================================================ */

static const KalmanTestVector_t KALMAN_BOOST_VECTORS[] = {
    /* Motor yanması sırasında yüksek ivme */
    { .altitude_measured = 10.0f, .accel_measured = 80.0f, .dt = 0.01f, 
      .expected_altitude = 10.0f, .expected_velocity = 10.0f, .tolerance = 20.0f },
    { .altitude_measured = 25.0f, .accel_measured = 85.0f, .dt = 0.01f, 
      .expected_altitude = 25.0f, .expected_velocity = 30.0f, .tolerance = 20.0f },
    { .altitude_measured = 50.0f, .accel_measured = 90.0f, .dt = 0.01f, 
      .expected_altitude = 50.0f, .expected_velocity = 50.0f, .tolerance = 20.0f },
    { .altitude_measured = 85.0f, .accel_measured = 85.0f, .dt = 0.01f, 
      .expected_altitude = 85.0f, .expected_velocity = 70.0f, .tolerance = 20.0f },
    { .altitude_measured = 130.0f, .accel_measured = 80.0f, .dt = 0.01f, 
      .expected_altitude = 130.0f, .expected_velocity = 90.0f, .tolerance = 20.0f },
};

static const KalmanTestScenario_t KALMAN_SCENARIO_BOOST = {
    .name = "Boost Phase",
    .vectors = KALMAN_BOOST_VECTORS,
    .count = 5,
    .initial_altitude = 0.0f,
    .initial_velocity = 0.0f
};

/* ============================================================================
 *                         SCENARIO 5: NOISY MEASUREMENT
 * ============================================================================ */

static const KalmanTestVector_t KALMAN_NOISY_VECTORS[] = {
    /* Gürültülü ölçümler - filter smooth etmeli */
    { .altitude_measured = 100.0f, .accel_measured = 0.0f, .dt = 0.01f, 
      .expected_altitude = 100.0f, .expected_velocity = 0.0f, .tolerance = 5.0f },
    { .altitude_measured = 115.0f, .accel_measured = 2.0f, .dt = 0.01f,   /* spike up */
      .expected_altitude = 100.0f, .expected_velocity = 0.0f, .tolerance = 10.0f },
    { .altitude_measured = 85.0f, .accel_measured = -2.0f, .dt = 0.01f,   /* spike down */
      .expected_altitude = 100.0f, .expected_velocity = 0.0f, .tolerance = 10.0f },
    { .altitude_measured = 102.0f, .accel_measured = 0.5f, .dt = 0.01f, 
      .expected_altitude = 100.0f, .expected_velocity = 0.0f, .tolerance = 10.0f },
    { .altitude_measured = 98.0f, .accel_measured = -0.5f, .dt = 0.01f, 
      .expected_altitude = 100.0f, .expected_velocity = 0.0f, .tolerance = 10.0f },
};

static const KalmanTestScenario_t KALMAN_SCENARIO_NOISY = {
    .name = "Noisy Measurements",
    .vectors = KALMAN_NOISY_VECTORS,
    .count = 5,
    .initial_altitude = 100.0f,
    .initial_velocity = 0.0f
};

/* ============================================================================
 *                         ALL SCENARIOS ARRAY
 * ============================================================================ */

static const KalmanTestScenario_t* KALMAN_ALL_SCENARIOS[] = {
    &KALMAN_SCENARIO_STATIC,
    &KALMAN_SCENARIO_CONST_VEL,
    &KALMAN_SCENARIO_APOGEE,
    &KALMAN_SCENARIO_BOOST,
    &KALMAN_SCENARIO_NOISY
};

#define KALMAN_SCENARIO_COUNT (sizeof(KALMAN_ALL_SCENARIOS) / sizeof(KALMAN_ALL_SCENARIOS[0]))

#endif /* KALMAN_VECTORS_H */
