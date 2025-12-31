/**
 * @file bmi088_conversion.c
 * @brief BMI088 raw → physical unit dönüşüm implementasyonu
 */

#include "bmi088_conversion.h"
#include <math.h>

/* ============================================================================
 *                         ACCELEROMETER CONVERSIONS
 * ============================================================================ */

float BMI088_GetAccelFullScale(BMI088_AccelRange_t range) {
    switch (range) {
        case BMI088_ACCEL_RANGE_3G:  return 3.0f;
        case BMI088_ACCEL_RANGE_6G:  return 6.0f;
        case BMI088_ACCEL_RANGE_12G: return 12.0f;
        case BMI088_ACCEL_RANGE_24G: return 24.0f;
        default: return 24.0f;
    }
}

float BMI088_Convert_AccelRawToG(int16_t raw_value, BMI088_AccelRange_t range) {
    float full_scale = BMI088_GetAccelFullScale(range);
    return ((float)raw_value / BMI088_RAW_MAX) * full_scale;
}

float BMI088_Convert_AccelRawToMps2(int16_t raw_value, BMI088_AccelRange_t range) {
    float g_value = BMI088_Convert_AccelRawToG(raw_value, range);
    return g_value * BMI088_GRAVITY;
}

void BMI088_Convert_AccelRaw3AxisToMps2(
    int16_t raw_x, int16_t raw_y, int16_t raw_z,
    BMI088_AccelRange_t range,
    float* out_x, float* out_y, float* out_z
) {
    *out_x = BMI088_Convert_AccelRawToMps2(raw_x, range);
    *out_y = BMI088_Convert_AccelRawToMps2(raw_y, range);
    *out_z = BMI088_Convert_AccelRawToMps2(raw_z, range);
}

int16_t BMI088_Convert_BytesToInt16(uint8_t lsb, uint8_t msb) {
    return (int16_t)((uint16_t)msb << 8 | lsb);
}

/* ============================================================================
 *                         GYROSCOPE CONVERSIONS
 * ============================================================================ */

float BMI088_GetGyroFullScale(BMI088_GyroRange_t range) {
    switch (range) {
        case BMI088_GYRO_RANGE_2000: return 2000.0f;
        case BMI088_GYRO_RANGE_1000: return 1000.0f;
        case BMI088_GYRO_RANGE_500:  return 500.0f;
        case BMI088_GYRO_RANGE_250:  return 250.0f;
        case BMI088_GYRO_RANGE_125:  return 125.0f;
        default: return 2000.0f;
    }
}

float BMI088_Convert_GyroRawToDps(int16_t raw_value, BMI088_GyroRange_t range) {
    float full_scale = BMI088_GetGyroFullScale(range);
    return ((float)raw_value / BMI088_RAW_MAX) * full_scale;
}

float BMI088_Convert_GyroRawToRps(int16_t raw_value, BMI088_GyroRange_t range) {
    float dps = BMI088_Convert_GyroRawToDps(raw_value, range);
    return dps * BMI088_DEG_TO_RAD;
}

void BMI088_Convert_GyroRaw3AxisToRps(
    int16_t raw_x, int16_t raw_y, int16_t raw_z,
    BMI088_GyroRange_t range,
    float* out_x, float* out_y, float* out_z
) {
    *out_x = BMI088_Convert_GyroRawToRps(raw_x, range);
    *out_y = BMI088_Convert_GyroRawToRps(raw_y, range);
    *out_z = BMI088_Convert_GyroRawToRps(raw_z, range);
}

/* ============================================================================
 *                         TEMPERATURE CONVERSION
 * ============================================================================ */

float BMI088_Convert_TempRawToCelsius(uint8_t msb, uint8_t lsb) {
    /* BMI088 temperature format:
     * MSB: bits 10:3
     * LSB: bits 7:5 -> bits 2:0 of 11-bit value
     * Resolution: 0.125°C/LSB, offset: 23°C at 0
     */
    int16_t raw = ((int16_t)msb << 3) | ((lsb >> 5) & 0x07);
    
    /* Sign extension for 11-bit value */
    if (raw > 1023) {
        raw -= 2048;
    }
    
    return (raw * 0.125f) + 23.0f;
}

/* ============================================================================
 *                         OFFSET CORRECTION
 * ============================================================================ */

float BMI088_ApplyOffset(float value, float offset) {
    return value - offset;
}

void BMI088_ApplyOffset3Axis(
    float* x, float* y, float* z,
    float offset_x, float offset_y, float offset_z
) {
    *x = *x - offset_x;
    *y = *y - offset_y;
    *z = *z - offset_z;
}

/* ============================================================================
 *                         VALIDATION
 * ============================================================================ */

uint8_t BMI088_IsAccelValueValid(float value_mps2, BMI088_AccelRange_t range) {
    float full_scale_g = BMI088_GetAccelFullScale(range);
    float max_mps2 = full_scale_g * BMI088_GRAVITY;
    
    if (isnan(value_mps2) || isinf(value_mps2)) {
        return 0;
    }
    
    if (fabsf(value_mps2) > max_mps2 * 1.1f) {  /* %10 margin */
        return 0;
    }
    
    return 1;
}

uint8_t BMI088_IsGyroValueValid(float value_dps, BMI088_GyroRange_t range) {
    float full_scale = BMI088_GetGyroFullScale(range);
    
    if (isnan(value_dps) || isinf(value_dps)) {
        return 0;
    }
    
    if (fabsf(value_dps) > full_scale * 1.1f) {  /* %10 margin */
        return 0;
    }
    
    return 1;
}
