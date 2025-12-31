/**
 * @file test_bmi088_conversion.c
 * @brief BMI088 raw → physical unit dönüşüm unit tests - PRODUCTION CODE
 * 
 * CRITICAL: "Test edilen kod uçmalı. Uçacak kod test edilmeli."
 *           This file tests the ACTUAL production code in Core/Inc/bmi088_conversions.h
 * 
 * @note Sensor doğruluğu için kritik
 */

/* Production code header - THE CODE THAT FLIES */
#include "../../Core/Inc/bmi088_conversions.h"

/* Test framework */
#include "../framework/unity_minimal.h"

#include <math.h>

/* Range definitions matching production code register values */
#define BMI088_ACCEL_RANGE_3G   0
#define BMI088_ACCEL_RANGE_6G   1
#define BMI088_ACCEL_RANGE_12G  2
#define BMI088_ACCEL_RANGE_24G  3

#define BMI088_GYRO_RANGE_2000  0
#define BMI088_GYRO_RANGE_1000  1
#define BMI088_GYRO_RANGE_500   2
#define BMI088_GYRO_RANGE_250   3
#define BMI088_GYRO_RANGE_125   4

/* ============================================================================
 *                         ACCELEROMETER CONVERSION TESTS
 * ============================================================================ */

TEST_CASE(test_accel_raw_zero_returns_zero) {
    float ms2 = bmi088_accel_raw_to_ms2(0, BMI088_ACCEL_RANGE_24G, 0.0f);
    
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, ms2);
    
    return TEST_PASS;
}

TEST_CASE(test_accel_max_positive_24g) {
    /* Full scale = 24G at range 3 */
    float ms2 = bmi088_accel_raw_to_ms2(32767, BMI088_ACCEL_RANGE_24G, 0.0f);
    float expected_g = 24.0f;
    float expected_ms2 = expected_g * 9.81f;
    
    /* Should be approximately 24g in m/s² */
    TEST_ASSERT_FLOAT_WITHIN(2.0f, expected_ms2, ms2);
    
    return TEST_PASS;
}

TEST_CASE(test_accel_max_negative_24g) {
    float ms2 = bmi088_accel_raw_to_ms2(-32768, BMI088_ACCEL_RANGE_24G, 0.0f);
    float expected_g = -24.0f;
    float expected_ms2 = expected_g * 9.81f;
    
    /* Should be approximately -24g in m/s² */
    TEST_ASSERT_FLOAT_WITHIN(2.0f, expected_ms2, ms2);
    
    return TEST_PASS;
}

TEST_CASE(test_accel_1g_value) {
    /* At 24G range, full scale = 24G for ±32768
     * 1G = 32768/24 ≈ 1365 raw */
    float ms2 = bmi088_accel_raw_to_ms2(1365, BMI088_ACCEL_RANGE_24G, 0.0f);
    
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 9.81f, ms2);
    
    return TEST_PASS;
}

TEST_CASE(test_accel_different_ranges) {
    int16_t raw = 16384;  /* Half scale */
    
    float ms2_3g = bmi088_accel_raw_to_ms2(raw, BMI088_ACCEL_RANGE_3G, 0.0f);
    float ms2_6g = bmi088_accel_raw_to_ms2(raw, BMI088_ACCEL_RANGE_6G, 0.0f);
    float ms2_12g = bmi088_accel_raw_to_ms2(raw, BMI088_ACCEL_RANGE_12G, 0.0f);
    float ms2_24g = bmi088_accel_raw_to_ms2(raw, BMI088_ACCEL_RANGE_24G, 0.0f);
    
    /* Same raw value, different results based on range
     * Half scale = half of full scale G value
     * 3G range: 1.5G = 14.7 m/s²
     * 6G range: 3.0G = 29.4 m/s²
     * 12G range: 6.0G = 58.9 m/s²
     * 24G range: 12.0G = 117.7 m/s²
     */
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 1.5f * 9.81f, ms2_3g);
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 3.0f * 9.81f, ms2_6g);
    TEST_ASSERT_FLOAT_WITHIN(6.0f, 6.0f * 9.81f, ms2_12g);
    TEST_ASSERT_FLOAT_WITHIN(12.0f, 12.0f * 9.81f, ms2_24g);
    
    return TEST_PASS;
}

TEST_CASE(test_accel_with_offset) {
    /* Test offset correction */
    float ms2_no_offset = bmi088_accel_raw_to_ms2(0, BMI088_ACCEL_RANGE_24G, 0.0f);
    float ms2_with_offset = bmi088_accel_raw_to_ms2(0, BMI088_ACCEL_RANGE_24G, 100.0f);
    
    /* With 100 mG offset, result should be -0.981 m/s² at raw=0 */
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, ms2_no_offset);
    TEST_ASSERT(ms2_with_offset < ms2_no_offset);
    
    return TEST_PASS;
}

TEST_CASE(test_accel_fast_version) {
    /* Compare fast version with standard version */
    int16_t raw = 10000;
    uint8_t range = BMI088_ACCEL_RANGE_24G;
    float offset = 50.0f;
    
    float ms2_std = bmi088_accel_raw_to_ms2(raw, range, offset);
    float ms2_fast = bmi088_accel_raw_to_ms2_fast(raw, range, offset);
    
    /* Results should be very close */
    TEST_ASSERT_FLOAT_WITHIN(0.1f, ms2_std, ms2_fast);
    
    return TEST_PASS;
}

/* ============================================================================
 *                         GYROSCOPE CONVERSION TESTS
 * ============================================================================ */

TEST_CASE(test_gyro_raw_zero_returns_zero) {
    float rads = bmi088_gyro_raw_to_rads(0, BMI088_GYRO_RANGE_2000, 0.0f);
    float dps = bmi088_gyro_raw_to_dps(0, BMI088_GYRO_RANGE_2000, 0.0f);
    
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, rads);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, dps);
    
    return TEST_PASS;
}

TEST_CASE(test_gyro_max_positive_2000dps) {
    float dps = bmi088_gyro_raw_to_dps(32767, BMI088_GYRO_RANGE_2000, 0.0f);
    
    TEST_ASSERT_FLOAT_WITHIN(10.0f, 2000.0f, dps);
    
    return TEST_PASS;
}

TEST_CASE(test_gyro_max_negative_2000dps) {
    float dps = bmi088_gyro_raw_to_dps(-32768, BMI088_GYRO_RANGE_2000, 0.0f);
    
    TEST_ASSERT_FLOAT_WITHIN(10.0f, -2000.0f, dps);
    
    return TEST_PASS;
}

TEST_CASE(test_gyro_different_ranges) {
    int16_t raw = 16384;  /* Half scale */
    
    float dps_2000 = bmi088_gyro_raw_to_dps(raw, BMI088_GYRO_RANGE_2000, 0.0f);
    float dps_1000 = bmi088_gyro_raw_to_dps(raw, BMI088_GYRO_RANGE_1000, 0.0f);
    float dps_500 = bmi088_gyro_raw_to_dps(raw, BMI088_GYRO_RANGE_500, 0.0f);
    float dps_250 = bmi088_gyro_raw_to_dps(raw, BMI088_GYRO_RANGE_250, 0.0f);
    float dps_125 = bmi088_gyro_raw_to_dps(raw, BMI088_GYRO_RANGE_125, 0.0f);
    
    TEST_ASSERT_FLOAT_WITHIN(10.0f, 1000.0f, dps_2000);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 500.0f, dps_1000);
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 250.0f, dps_500);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 125.0f, dps_250);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 62.5f, dps_125);
    
    return TEST_PASS;
}

TEST_CASE(test_gyro_dps_to_rads_conversion) {
    int16_t raw = 16384;
    
    float dps = bmi088_gyro_raw_to_dps(raw, BMI088_GYRO_RANGE_2000, 0.0f);
    float rads = bmi088_gyro_raw_to_rads(raw, BMI088_GYRO_RANGE_2000, 0.0f);
    
    /* rads = dps * π/180 */
    float expected_rads = dps * DEG_TO_RAD;
    
    TEST_ASSERT_FLOAT_WITHIN(0.1f, expected_rads, rads);
    
    return TEST_PASS;
}

TEST_CASE(test_gyro_with_offset) {
    /* Test offset correction */
    float rads_no_offset = bmi088_gyro_raw_to_rads(0, BMI088_GYRO_RANGE_2000, 0.0f);
    float rads_with_offset = bmi088_gyro_raw_to_rads(0, BMI088_GYRO_RANGE_2000, 10.0f);
    
    /* With 10 dps offset at raw=0, result should be negative */
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, rads_no_offset);
    TEST_ASSERT(rads_with_offset < 0.0f);
    
    return TEST_PASS;
}

/* ============================================================================
 *                         BYTE CONVERSION TESTS
 * ============================================================================ */

TEST_CASE(test_bytes_to_int16_zero) {
    int16_t result = bmi088_bytes_to_int16(0x00, 0x00);
    TEST_ASSERT_EQUAL_INT(0, result);
    
    return TEST_PASS;
}

TEST_CASE(test_bytes_to_int16_max_positive) {
    int16_t result = bmi088_bytes_to_int16(0xFF, 0x7F);
    TEST_ASSERT_EQUAL_INT(32767, result);
    
    return TEST_PASS;
}

TEST_CASE(test_bytes_to_int16_max_negative) {
    int16_t result = bmi088_bytes_to_int16(0x00, 0x80);
    TEST_ASSERT_EQUAL_INT(-32768, result);
    
    return TEST_PASS;
}

TEST_CASE(test_bytes_to_int16_minus_one) {
    int16_t result = bmi088_bytes_to_int16(0xFF, 0xFF);
    TEST_ASSERT_EQUAL_INT(-1, result);
    
    return TEST_PASS;
}

TEST_CASE(test_bytes_to_int16_typical) {
    /* 0x1234 = 4660 decimal */
    int16_t result = bmi088_bytes_to_int16(0x34, 0x12);
    TEST_ASSERT_EQUAL_INT(0x1234, result);
    
    return TEST_PASS;
}

/* ============================================================================
 *                         TEMPERATURE CONVERSION TESTS
 * ============================================================================ */

TEST_CASE(test_temp_zero_offset) {
    /* At raw 0, temperature should be 23°C (offset) */
    float temp = bmi088_temp_raw_to_celsius(0x00, 0x00);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 23.0f, temp);
    
    return TEST_PASS;
}

TEST_CASE(test_temp_positive_value) {
    /* Positive offset from 23°C */
    float temp = bmi088_temp_raw_to_celsius(0x50, 0x00);
    TEST_ASSERT(temp > 23.0f);
    
    return TEST_PASS;
}

/* ============================================================================
 *                         VALIDATION HELPER TESTS
 * ============================================================================ */

TEST_CASE(test_accel_validation_in_range) {
    /* 1G should be valid for 30G max */
    int valid = bmi088_accel_is_valid(9.81f, 30.0f);
    TEST_ASSERT_EQUAL_INT(1, valid);
    
    return TEST_PASS;
}

TEST_CASE(test_accel_validation_out_of_range) {
    /* 50G should be invalid for 30G max */
    int valid = bmi088_accel_is_valid(50.0f * 9.81f, 30.0f);
    TEST_ASSERT_EQUAL_INT(0, valid);
    
    return TEST_PASS;
}

TEST_CASE(test_gyro_validation_in_range) {
    /* 100 dps in rad/s should be valid for 2000 dps max */
    float rads = 100.0f * DEG_TO_RAD;
    int valid = bmi088_gyro_is_valid(rads, 2000.0f);
    TEST_ASSERT_EQUAL_INT(1, valid);
    
    return TEST_PASS;
}

TEST_CASE(test_gyro_validation_out_of_range) {
    /* 3000 dps in rad/s should be invalid for 2000 dps max */
    float rads = 3000.0f * DEG_TO_RAD;
    int valid = bmi088_gyro_is_valid(rads, 2000.0f);
    TEST_ASSERT_EQUAL_INT(0, valid);
    
    return TEST_PASS;
}

/* ============================================================================
 *                         TEST GROUP RUNNER
 * ============================================================================ */

void run_bmi088_conversion_tests(void) {
    /* Accelerometer tests */
    RUN_TEST(test_accel_raw_zero_returns_zero);
    RUN_TEST(test_accel_max_positive_24g);
    RUN_TEST(test_accel_max_negative_24g);
    RUN_TEST(test_accel_1g_value);
    RUN_TEST(test_accel_different_ranges);
    RUN_TEST(test_accel_with_offset);
    RUN_TEST(test_accel_fast_version);
    
    /* Gyroscope tests */
    RUN_TEST(test_gyro_raw_zero_returns_zero);
    RUN_TEST(test_gyro_max_positive_2000dps);
    RUN_TEST(test_gyro_max_negative_2000dps);
    RUN_TEST(test_gyro_different_ranges);
    RUN_TEST(test_gyro_dps_to_rads_conversion);
    RUN_TEST(test_gyro_with_offset);
    
    /* Byte conversion tests */
    RUN_TEST(test_bytes_to_int16_zero);
    RUN_TEST(test_bytes_to_int16_max_positive);
    RUN_TEST(test_bytes_to_int16_max_negative);
    RUN_TEST(test_bytes_to_int16_minus_one);
    RUN_TEST(test_bytes_to_int16_typical);
    
    /* Temperature tests */
    RUN_TEST(test_temp_zero_offset);
    RUN_TEST(test_temp_positive_value);
    
    /* Validation tests */
    RUN_TEST(test_accel_validation_in_range);
    RUN_TEST(test_accel_validation_out_of_range);
    RUN_TEST(test_gyro_validation_in_range);
    RUN_TEST(test_gyro_validation_out_of_range);
}
