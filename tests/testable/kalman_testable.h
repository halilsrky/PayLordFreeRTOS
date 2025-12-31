/**
 * @file kalman_testable.h
 * @brief Kalman filter - HAL/RTOS arındırılmış test edilebilir versiyon
 * @note Production kodu: Core/Src/kalman.c
 * @note Bu dosya SADECE unit test için kullanılır
 */

#ifndef KALMAN_TESTABLE_H
#define KALMAN_TESTABLE_H

#include <stdint.h>

/* ============================================================================
 *                         KALMAN STATE STRUCTURE
 * ============================================================================ */

/**
 * @brief Kalman filter state structure
 * @note Production ile AYNI yapı - senkronize tut
 */
typedef struct {
    /* State vector [altitude, velocity, acceleration] */
    float x[3];
    
    /* Covariance matrix 3x3 */
    float P[3][3];
    
    /* Process noise coefficient */
    float process_noise;
    
    /* Measurement noise coefficients */
    float measurement_noise_alt;
    float measurement_noise_acc;
    
    /* Apogee detection */
    int apogee_detected;
    int apogee_counter;
    float prev_velocity;
    
    /* Mach transition control */
    int in_mach_transition;
} KalmanFilter_Testable_t;

/* ============================================================================
 *                         PUBLIC FUNCTIONS
 * ============================================================================ */

/**
 * @brief Kalman filter başlat
 * @param kf Pointer to filter state
 */
void KalmanFilter_Testable_Init(KalmanFilter_Testable_t* kf);

/**
 * @brief Custom parametrelerle başlat
 */
void KalmanFilter_Testable_InitWithParams(
    KalmanFilter_Testable_t* kf,
    float process_noise,
    float measurement_noise_alt,
    float measurement_noise_acc
);

/**
 * @brief Kalman filter güncelle
 * @param kf Filter state
 * @param altitude Ölçülen irtifa (m)
 * @param accel Ölçülen ivme (m/s²)
 * @param dt Zaman adımı (s)
 * @return Filtrelenmiş irtifa
 */
float KalmanFilter_Testable_Update(
    KalmanFilter_Testable_t* kf, 
    float altitude, 
    float accel, 
    float dt
);

/**
 * @brief Sadece time update (prediction)
 */
void KalmanFilter_Testable_Predict(KalmanFilter_Testable_t* kf, float dt);

/**
 * @brief Sadece measurement update (correction)
 */
void KalmanFilter_Testable_Correct(
    KalmanFilter_Testable_t* kf, 
    float altitude, 
    float accel
);

/**
 * @brief Apogee tespit edildi mi?
 */
int KalmanFilter_Testable_IsApogeeDetected(KalmanFilter_Testable_t* kf);

/**
 * @brief Mevcut velocity al
 */
float KalmanFilter_Testable_GetVelocity(KalmanFilter_Testable_t* kf);

/**
 * @brief Mevcut altitude al
 */
float KalmanFilter_Testable_GetAltitude(KalmanFilter_Testable_t* kf);

/**
 * @brief Mevcut acceleration al
 */
float KalmanFilter_Testable_GetAcceleration(KalmanFilter_Testable_t* kf);

/**
 * @brief State reset (test için)
 */
void KalmanFilter_Testable_Reset(KalmanFilter_Testable_t* kf);

/**
 * @brief Covariance diagonal toplamı (convergence check)
 */
float KalmanFilter_Testable_GetCovarianceTrace(KalmanFilter_Testable_t* kf);

#endif /* KALMAN_TESTABLE_H */
