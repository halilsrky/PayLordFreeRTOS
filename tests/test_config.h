/**
 * @file test_config.h
 * @brief Unit test konfigürasyonu
 * @note Safety-critical: Bu değerler production ile aynı olmalı
 */

#ifndef TEST_CONFIG_H
#define TEST_CONFIG_H

/* ============================================================================
 *                         COMPILE-TIME SWITCHES
 * ============================================================================ */

/* Unit test mode - production build'de TANIMLANMAMALI */
#define UNIT_TEST_MODE  1

/* FreeRTOS ve HAL devre dışı */
#ifdef UNIT_TEST_MODE
    #define NO_FREERTOS     1
    #define NO_HAL          1
#endif

/* ============================================================================
 *                         TEST TOLERANSLARI
 * ============================================================================ */

/* Floating point karşılaştırma toleransı */
#define TEST_EPSILON_FLOAT      1e-4f
#define TEST_EPSILON_DOUBLE     1e-8

/* Kalman filter spesifik toleranslar */
#define KALMAN_ALT_TOLERANCE    0.5f    /* ±0.5 metre */
#define KALMAN_VEL_TOLERANCE    0.1f    /* ±0.1 m/s */
#define KALMAN_ACC_TOLERANCE    0.5f    /* ±0.5 m/s² */

/* State machine timing (simulated) */
#define SIM_TICK_PERIOD_MS      10      /* 100 Hz simülasyon */

/* ============================================================================
 *                         PRODUCTION SABITLERI
 * ============================================================================ */

/* Bu değerler Core/Inc/flight_algorithm.h ile SYNC olmalı */
#define PROD_LAUNCH_ACCEL_THRESHOLD     45.0f
#define PROD_MIN_ARMING_ALTITUDE        1800.0f
#define PROD_MAIN_CHUTE_ALTITUDE        500.0f
#define PROD_APOGEE_CONFIRM_COUNT       9
#define PROD_BURNOUT_CONFIRM_COUNT      10

/* BMI088 constants are now defined in Core/Inc/bmi088_conversions.h */
/* Do not redefine here to avoid conflicts */

/* ============================================================================
 *                         TEST FRAMEWORK SEÇİMİ
 * ============================================================================ */

/* Minimal framework kullan (dependency yok) */
#define USE_MINIMAL_FRAMEWORK   1

/* Verbose output (debug için) */
#define TEST_VERBOSE            0

/* ============================================================================
 *                         TYPE DEFINITIONS
 * ============================================================================ */

#include <stdint.h>
#include <stddef.h>

/* Boolean standardizasyonu */
#ifndef __cplusplus
    typedef uint8_t bool;
    #define true    1
    #define false   0
#endif

/* Test result codes */
typedef enum {
    TEST_PASS = 0,
    TEST_FAIL = 1,
    TEST_SKIP = 2,
    TEST_ERROR = 3
} TestResult_t;

/* Test function pointer type */
typedef TestResult_t (*TestFunc_t)(void);

/* Test case structure */
typedef struct {
    const char* name;
    TestFunc_t func;
    const char* file;
    int line;
} TestCase_t;

/* ============================================================================
 *                         TEST REGISTRATION MAKROLARI
 * ============================================================================ */

#define TEST_CASE(name) \
    TestResult_t name(void)

#define REGISTER_TEST(test_func) \
    { #test_func, test_func, __FILE__, __LINE__ }

#define END_TEST_LIST \
    { NULL, NULL, NULL, 0 }

#endif /* TEST_CONFIG_H */
