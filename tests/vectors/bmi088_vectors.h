/**
 * @file bmi088_vectors.h
 * @brief BMI088 raw → physical unit dönüşüm test vektörleri
 */

#ifndef BMI088_VECTORS_H
#define BMI088_VECTORS_H

#include <stdint.h>
#include "../testable/bmi088_conversion.h"

/* ============================================================================
 *                         ACCELEROMETER TEST VECTORS
 * ============================================================================ */

typedef struct {
    int16_t raw_value;
    BMI088_AccelRange_t range;
    float expected_g;
    float expected_mps2;
    float tolerance;
} AccelTestVector_t;

static const AccelTestVector_t ACCEL_VECTORS[] = {
    /* Zero */
    { .raw_value = 0, .range = BMI088_ACCEL_RANGE_24G,
      .expected_g = 0.0f, .expected_mps2 = 0.0f, .tolerance = 0.01f },
    
    /* Max positive - 24G range */
    { .raw_value = 32767, .range = BMI088_ACCEL_RANGE_24G,
      .expected_g = 24.0f, .expected_mps2 = 235.36f, .tolerance = 0.1f },
    
    /* Max negative - 24G range */
    { .raw_value = -32768, .range = BMI088_ACCEL_RANGE_24G,
      .expected_g = -24.0f, .expected_mps2 = -235.36f, .tolerance = 0.1f },
    
    /* 1G at 24G range */
    { .raw_value = 1365, .range = BMI088_ACCEL_RANGE_24G,
      .expected_g = 1.0f, .expected_mps2 = 9.8f, .tolerance = 0.2f },
    
    /* -1G at 24G range */
    { .raw_value = -1365, .range = BMI088_ACCEL_RANGE_24G,
      .expected_g = -1.0f, .expected_mps2 = -9.8f, .tolerance = 0.2f },
    
    /* Different ranges - same raw value different output */
    /* 3G range */
    { .raw_value = 16384, .range = BMI088_ACCEL_RANGE_3G,
      .expected_g = 1.5f, .expected_mps2 = 14.71f, .tolerance = 0.1f },
    
    /* 6G range */
    { .raw_value = 16384, .range = BMI088_ACCEL_RANGE_6G,
      .expected_g = 3.0f, .expected_mps2 = 29.42f, .tolerance = 0.1f },
    
    /* 12G range */
    { .raw_value = 16384, .range = BMI088_ACCEL_RANGE_12G,
      .expected_g = 6.0f, .expected_mps2 = 58.84f, .tolerance = 0.1f },
    
    /* 24G range */
    { .raw_value = 16384, .range = BMI088_ACCEL_RANGE_24G,
      .expected_g = 12.0f, .expected_mps2 = 117.68f, .tolerance = 0.1f },
    
    /* Typical rocket launch: ~5G */
    { .raw_value = 6827, .range = BMI088_ACCEL_RANGE_24G,
      .expected_g = 5.0f, .expected_mps2 = 49.03f, .tolerance = 0.3f },
    
    /* High acceleration: 15G */
    { .raw_value = 20480, .range = BMI088_ACCEL_RANGE_24G,
      .expected_g = 15.0f, .expected_mps2 = 147.1f, .tolerance = 0.5f },
};

#define ACCEL_VECTOR_COUNT (sizeof(ACCEL_VECTORS) / sizeof(ACCEL_VECTORS[0]))

/* ============================================================================
 *                         GYROSCOPE TEST VECTORS
 * ============================================================================ */

typedef struct {
    int16_t raw_value;
    BMI088_GyroRange_t range;
    float expected_dps;
    float expected_rps;
    float tolerance;
} GyroTestVector_t;

static const GyroTestVector_t GYRO_VECTORS[] = {
    /* Zero */
    { .raw_value = 0, .range = BMI088_GYRO_RANGE_2000,
      .expected_dps = 0.0f, .expected_rps = 0.0f, .tolerance = 0.01f },
    
    /* Max positive - 2000 deg/s */
    { .raw_value = 32767, .range = BMI088_GYRO_RANGE_2000,
      .expected_dps = 2000.0f, .expected_rps = 34.91f, .tolerance = 0.1f },
    
    /* Max negative - 2000 deg/s */
    { .raw_value = -32768, .range = BMI088_GYRO_RANGE_2000,
      .expected_dps = -2000.0f, .expected_rps = -34.91f, .tolerance = 0.1f },
    
    /* Different ranges */
    /* 1000 deg/s range, half scale */
    { .raw_value = 16384, .range = BMI088_GYRO_RANGE_1000,
      .expected_dps = 500.0f, .expected_rps = 8.73f, .tolerance = 0.1f },
    
    /* 500 deg/s range */
    { .raw_value = 16384, .range = BMI088_GYRO_RANGE_500,
      .expected_dps = 250.0f, .expected_rps = 4.36f, .tolerance = 0.1f },
    
    /* 250 deg/s range */
    { .raw_value = 16384, .range = BMI088_GYRO_RANGE_250,
      .expected_dps = 125.0f, .expected_rps = 2.18f, .tolerance = 0.1f },
    
    /* 125 deg/s range */
    { .raw_value = 16384, .range = BMI088_GYRO_RANGE_125,
      .expected_dps = 62.5f, .expected_rps = 1.09f, .tolerance = 0.1f },
    
    /* Typical rocket spin: ~100 deg/s */
    { .raw_value = 1638, .range = BMI088_GYRO_RANGE_2000,
      .expected_dps = 100.0f, .expected_rps = 1.745f, .tolerance = 0.1f },
    
    /* Negative rotation */
    { .raw_value = -8192, .range = BMI088_GYRO_RANGE_2000,
      .expected_dps = -500.0f, .expected_rps = -8.73f, .tolerance = 0.1f },
};

#define GYRO_VECTOR_COUNT (sizeof(GYRO_VECTORS) / sizeof(GYRO_VECTORS[0]))

/* ============================================================================
 *                         BYTE CONVERSION TEST VECTORS
 * ============================================================================ */

typedef struct {
    uint8_t lsb;
    uint8_t msb;
    int16_t expected;
} ByteConversionVector_t;

static const ByteConversionVector_t BYTE_VECTORS[] = {
    /* Zero */
    { .lsb = 0x00, .msb = 0x00, .expected = 0 },
    
    /* Max positive */
    { .lsb = 0xFF, .msb = 0x7F, .expected = 32767 },
    
    /* Max negative */
    { .lsb = 0x00, .msb = 0x80, .expected = -32768 },
    
    /* -1 */
    { .lsb = 0xFF, .msb = 0xFF, .expected = -1 },
    
    /* Typical value */
    { .lsb = 0x34, .msb = 0x12, .expected = 0x1234 },
    
    /* Another typical value */
    { .lsb = 0xCD, .msb = 0xAB, .expected = (int16_t)0xABCD },
};

#define BYTE_VECTOR_COUNT (sizeof(BYTE_VECTORS) / sizeof(BYTE_VECTORS[0]))

/* ============================================================================
 *                         TEMPERATURE TEST VECTORS
 * ============================================================================ */

typedef struct {
    uint8_t msb;
    uint8_t lsb;
    float expected_celsius;
    float tolerance;
} TempTestVector_t;

static const TempTestVector_t TEMP_VECTORS[] = {
    /* 23°C (zero offset) */
    { .msb = 0x00, .lsb = 0x00, .expected_celsius = 23.0f, .tolerance = 0.5f },
    
    /* Positive temperature */
    { .msb = 0x50, .lsb = 0x00, .expected_celsius = 33.0f, .tolerance = 0.5f },
    
    /* Room temperature ~25°C */
    { .msb = 0x10, .lsb = 0x00, .expected_celsius = 25.0f, .tolerance = 0.5f },
};

#define TEMP_VECTOR_COUNT (sizeof(TEMP_VECTORS) / sizeof(TEMP_VECTORS[0]))

/* ============================================================================
 *                         EDGE CASE VECTORS
 * ============================================================================ */

typedef struct {
    float input_value;
    BMI088_AccelRange_t accel_range;
    BMI088_GyroRange_t gyro_range;
    uint8_t expected_accel_valid;
    uint8_t expected_gyro_valid;
} ValidationVector_t;

static const ValidationVector_t VALIDATION_VECTORS[] = {
    /* Valid values */
    { .input_value = 100.0f, .accel_range = BMI088_ACCEL_RANGE_24G,
      .gyro_range = BMI088_GYRO_RANGE_2000,
      .expected_accel_valid = 1, .expected_gyro_valid = 1 },
    
    /* Out of range accel */
    { .input_value = 300.0f, .accel_range = BMI088_ACCEL_RANGE_24G,
      .gyro_range = BMI088_GYRO_RANGE_2000,
      .expected_accel_valid = 0, .expected_gyro_valid = 1 },
    
    /* Out of range gyro */
    { .input_value = 2500.0f, .accel_range = BMI088_ACCEL_RANGE_24G,
      .gyro_range = BMI088_GYRO_RANGE_2000,
      .expected_accel_valid = 0, .expected_gyro_valid = 0 },
};

#define VALIDATION_VECTOR_COUNT (sizeof(VALIDATION_VECTORS) / sizeof(VALIDATION_VECTORS[0]))

#endif /* BMI088_VECTORS_H */
