/**
 * @file bmi088_conversion.h
 * @brief BMI088 raw → physical unit dönüşüm fonksiyonları
 * @note HAL/I2C bağımsız, pure math
 */

#ifndef BMI088_CONVERSION_H
#define BMI088_CONVERSION_H

#include <stdint.h>

/* ============================================================================
 *                         RANGE DEFINITIONS
 * ============================================================================ */

typedef enum {
    BMI088_ACCEL_RANGE_3G  = 0,
    BMI088_ACCEL_RANGE_6G  = 1,
    BMI088_ACCEL_RANGE_12G = 2,
    BMI088_ACCEL_RANGE_24G = 3
} BMI088_AccelRange_t;

typedef enum {
    BMI088_GYRO_RANGE_2000 = 0,
    BMI088_GYRO_RANGE_1000 = 1,
    BMI088_GYRO_RANGE_500  = 2,
    BMI088_GYRO_RANGE_250  = 3,
    BMI088_GYRO_RANGE_125  = 4
} BMI088_GyroRange_t;

/* ============================================================================
 *                         CONSTANTS
 * ============================================================================ */

#define BMI088_RAW_MAX          32768.0f
#define BMI088_GRAVITY          9.80665f
#define BMI088_DEG_TO_RAD       (3.14159265359f / 180.0f)
#define BMI088_RAD_TO_DEG       (180.0f / 3.14159265359f)

/* ============================================================================
 *                         ACCELEROMETER CONVERSIONS
 * ============================================================================ */

/**
 * @brief Raw accelerometer → m/s²
 * @param raw_value 16-bit signed raw değer
 * @param range Accelerometer range setting
 * @return Acceleration in m/s²
 */
float BMI088_Convert_AccelRawToMps2(int16_t raw_value, BMI088_AccelRange_t range);

/**
 * @brief Raw accelerometer → g
 * @param raw_value 16-bit signed raw değer
 * @param range Accelerometer range setting
 * @return Acceleration in g
 */
float BMI088_Convert_AccelRawToG(int16_t raw_value, BMI088_AccelRange_t range);

/**
 * @brief 3-axis raw accelerometer → m/s²
 */
void BMI088_Convert_AccelRaw3AxisToMps2(
    int16_t raw_x, int16_t raw_y, int16_t raw_z,
    BMI088_AccelRange_t range,
    float* out_x, float* out_y, float* out_z
);

/**
 * @brief Raw byte array → int16 (little endian)
 */
int16_t BMI088_Convert_BytesToInt16(uint8_t lsb, uint8_t msb);

/**
 * @brief Range enum → full scale değeri (g)
 */
float BMI088_GetAccelFullScale(BMI088_AccelRange_t range);

/* ============================================================================
 *                         GYROSCOPE CONVERSIONS
 * ============================================================================ */

/**
 * @brief Raw gyroscope → deg/s
 * @param raw_value 16-bit signed raw değer
 * @param range Gyroscope range setting
 * @return Angular rate in deg/s
 */
float BMI088_Convert_GyroRawToDps(int16_t raw_value, BMI088_GyroRange_t range);

/**
 * @brief Raw gyroscope → rad/s
 * @param raw_value 16-bit signed raw değer
 * @param range Gyroscope range setting
 * @return Angular rate in rad/s
 */
float BMI088_Convert_GyroRawToRps(int16_t raw_value, BMI088_GyroRange_t range);

/**
 * @brief 3-axis raw gyroscope → rad/s
 */
void BMI088_Convert_GyroRaw3AxisToRps(
    int16_t raw_x, int16_t raw_y, int16_t raw_z,
    BMI088_GyroRange_t range,
    float* out_x, float* out_y, float* out_z
);

/**
 * @brief Range enum → full scale değeri (deg/s)
 */
float BMI088_GetGyroFullScale(BMI088_GyroRange_t range);

/* ============================================================================
 *                         TEMPERATURE CONVERSION
 * ============================================================================ */

/**
 * @brief Raw temperature → °C
 * @param msb Temperature MSB
 * @param lsb Temperature LSB (bits 7:5)
 * @return Temperature in °C
 */
float BMI088_Convert_TempRawToCelsius(uint8_t msb, uint8_t lsb);

/* ============================================================================
 *                         OFFSET CORRECTION
 * ============================================================================ */

/**
 * @brief Offset uygula
 */
float BMI088_ApplyOffset(float value, float offset);

/**
 * @brief 3-axis offset uygula
 */
void BMI088_ApplyOffset3Axis(
    float* x, float* y, float* z,
    float offset_x, float offset_y, float offset_z
);

/* ============================================================================
 *                         VALIDATION
 * ============================================================================ */

/**
 * @brief Değer geçerli aralıkta mı?
 */
uint8_t BMI088_IsAccelValueValid(float value_mps2, BMI088_AccelRange_t range);

/**
 * @brief Değer geçerli aralıkta mı?
 */
uint8_t BMI088_IsGyroValueValid(float value_dps, BMI088_GyroRange_t range);

#endif /* BMI088_CONVERSION_H */
