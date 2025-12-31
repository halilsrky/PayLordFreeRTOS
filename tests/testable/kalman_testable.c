/**
 * @file kalman_testable.c
 * @brief Kalman filter - Test edilebilir implementasyon
 * @note HAL/RTOS dependency YOK
 * @note Production ile matematik AYNI olmalı
 */

#include "kalman_testable.h"
#include <math.h>

/* ============================================================================
 *                         PRIVATE FUNCTIONS
 * ============================================================================ */

/**
 * @brief Time update (prediction step)
 */
static void KalmanFilter_TimeUpdate_Internal(KalmanFilter_Testable_t* kf, float dt) {
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt2 * dt2;
    
    /* State transition: x = F*x */
    /* F = [1 dt dt²/2; 0 1 dt; 0 0 1] */
    float x0_new = kf->x[0] + kf->x[1] * dt + kf->x[2] * dt2 / 2.0f;
    float x1_new = kf->x[1] + kf->x[2] * dt;
    float x2_new = kf->x[2];  /* Constant acceleration model */
    
    kf->x[0] = x0_new;
    kf->x[1] = x1_new;
    kf->x[2] = x2_new;
    
    /* Process noise covariance Q */
    float q = kf->process_noise;
    float Q[3][3] = {
        {dt4/4.0f * q, dt3/2.0f * q, dt2/2.0f * q},
        {dt3/2.0f * q, dt2 * q,      dt * q},
        {dt2/2.0f * q, dt * q,       q}
    };
    
    /* State transition matrix F */
    float F[3][3] = {
        {1.0f, dt, dt2/2.0f},
        {0.0f, 1.0f, dt},
        {0.0f, 0.0f, 1.0f}
    };
    
    /* P = F*P*F' + Q */
    float FP[3][3] = {0};
    float FPFT[3][3] = {0};
    
    /* FP = F*P */
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            FP[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                FP[i][j] += F[i][k] * kf->P[k][j];
            }
        }
    }
    
    /* FPFT = FP*F' */
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            FPFT[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                FPFT[i][j] += FP[i][k] * F[j][k];
            }
        }
    }
    
    /* P = FPFT + Q */
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            kf->P[i][j] = FPFT[i][j] + Q[i][j];
        }
    }
}

/**
 * @brief Measurement update (correction step)
 */
static void KalmanFilter_MeasurementUpdate_Internal(
    KalmanFilter_Testable_t* kf, 
    float altitude, 
    float accel
) {
    if (kf->in_mach_transition) {
        /* Mach transition: sadece acceleration measurement */
        float H[1][3] = {{0.0f, 0.0f, 1.0f}};
        float z = accel;
        float y = z - kf->x[2];
        
        float HP[1][3] = {0};
        float S = 0.0f;
        
        for (int i = 0; i < 3; i++) {
            HP[0][i] = H[0][0]*kf->P[0][i] + H[0][1]*kf->P[1][i] + H[0][2]*kf->P[2][i];
        }
        
        S = HP[0][0]*H[0][0] + HP[0][1]*H[0][1] + HP[0][2]*H[0][2] + kf->measurement_noise_acc;
        
        float K[3] = {0};
        float S_inv = 1.0f / S;
        
        for (int i = 0; i < 3; i++) {
            K[i] = (kf->P[i][0]*H[0][0] + kf->P[i][1]*H[0][1] + kf->P[i][2]*H[0][2]) * S_inv;
        }
        
        for (int i = 0; i < 3; i++) {
            kf->x[i] += K[i] * y;
        }
        
        float KH[3][3] = {0};
        float IKH[3][3];
        float Pnew[3][3] = {0};
        
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                KH[i][j] = K[i] * H[0][j];
                IKH[i][j] = (i == j ? 1.0f : 0.0f) - KH[i][j];
            }
        }
        
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                Pnew[i][j] = 0.0f;
                for (int k = 0; k < 3; k++) {
                    Pnew[i][j] += IKH[i][k] * kf->P[k][j];
                }
            }
        }
        
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                kf->P[i][j] = Pnew[i][j];
            }
        }
    } else {
        /* Normal: altitude ve acceleration */
        float H[2][3] = {
            {1.0f, 0.0f, 0.0f},
            {0.0f, 0.0f, 1.0f}
        };
        
        float z[2] = {altitude, accel};
        float y_vec[2] = {z[0] - kf->x[0], z[1] - kf->x[2]};
        
        float HP[2][3] = {0};
        float S[2][2] = {0};
        float R[2][2] = {
            {kf->measurement_noise_alt, 0.0f},
            {0.0f, kf->measurement_noise_acc}
        };
        
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 3; j++) {
                HP[i][j] = 0.0f;
                for (int k = 0; k < 3; k++) {
                    HP[i][j] += H[i][k] * kf->P[k][j];
                }
            }
        }
        
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                S[i][j] = 0.0f;
                for (int k = 0; k < 3; k++) {
                    S[i][j] += HP[i][k] * H[j][k];
                }
                S[i][j] += R[i][j];
            }
        }
        
        float det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
        if (fabsf(det) < 1e-6f) {
            return;  /* Singular matrix, skip */
        }
        
        float S_inv[2][2] = {
            {S[1][1] / det, -S[0][1] / det},
            {-S[1][0] / det, S[0][0] / det}
        };
        
        float PHt[3][2] = {0};
        float K[3][2] = {0};
        
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 2; j++) {
                PHt[i][j] = 0.0f;
                for (int k = 0; k < 3; k++) {
                    PHt[i][j] += kf->P[i][k] * H[j][k];
                }
            }
        }
        
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 2; j++) {
                K[i][j] = 0.0f;
                for (int k = 0; k < 2; k++) {
                    K[i][j] += PHt[i][k] * S_inv[k][j];
                }
            }
        }
        
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 2; j++) {
                kf->x[i] += K[i][j] * y_vec[j];
            }
        }
        
        float KH[3][3] = {0};
        float IKH[3][3];
        float Pnew[3][3] = {0};
        
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                KH[i][j] = 0.0f;
                for (int k = 0; k < 2; k++) {
                    KH[i][j] += K[i][k] * H[k][j];
                }
                IKH[i][j] = (i == j ? 1.0f : 0.0f) - KH[i][j];
            }
        }
        
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                Pnew[i][j] = 0.0f;
                for (int k = 0; k < 3; k++) {
                    Pnew[i][j] += IKH[i][k] * kf->P[k][j];
                }
            }
        }
        
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                kf->P[i][j] = Pnew[i][j];
            }
        }
    }
}

/**
 * @brief Apogee detection
 */
static int KalmanFilter_DetectApogee_Internal(KalmanFilter_Testable_t* kf) {
    if (kf->x[1] < 0.0f && kf->x[1] < kf->prev_velocity) {
        kf->apogee_counter++;
        if (kf->apogee_counter >= 5) {
            kf->apogee_detected = 1;
        }
    } else {
        kf->apogee_counter = 0;
    }
    
    kf->prev_velocity = kf->x[1];
    return kf->apogee_detected;
}

/* ============================================================================
 *                         PUBLIC FUNCTIONS
 * ============================================================================ */

void KalmanFilter_Testable_Init(KalmanFilter_Testable_t* kf) {
    kf->x[0] = 0.0f;
    kf->x[1] = 0.0f;
    kf->x[2] = 0.0f;
    
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            kf->P[i][j] = 0.0f;
        }
    }
    
    kf->P[0][0] = 1.0f;
    kf->P[1][1] = 0.1f;
    kf->P[2][2] = 1.0f;
    
    kf->process_noise = 0.01f;
    kf->measurement_noise_alt = 0.005f;
    kf->measurement_noise_acc = 5.0f;
    
    kf->apogee_detected = 0;
    kf->apogee_counter = 0;
    kf->prev_velocity = 0.0f;
    kf->in_mach_transition = 0;
}

void KalmanFilter_Testable_InitWithParams(
    KalmanFilter_Testable_t* kf,
    float process_noise,
    float measurement_noise_alt,
    float measurement_noise_acc
) {
    KalmanFilter_Testable_Init(kf);
    kf->process_noise = process_noise;
    kf->measurement_noise_alt = measurement_noise_alt;
    kf->measurement_noise_acc = measurement_noise_acc;
}

float KalmanFilter_Testable_Update(
    KalmanFilter_Testable_t* kf, 
    float altitude, 
    float accel, 
    float dt
) {
    /* Mach transition check */
    if (fabsf(kf->x[1]) > 300.0f && fabsf(kf->x[1]) < 350.0f) {
        kf->in_mach_transition = 1;
    } else {
        kf->in_mach_transition = 0;
    }
    
    KalmanFilter_TimeUpdate_Internal(kf, dt);
    KalmanFilter_MeasurementUpdate_Internal(kf, altitude, accel);
    KalmanFilter_DetectApogee_Internal(kf);
    
    return kf->x[0];
}

void KalmanFilter_Testable_Predict(KalmanFilter_Testable_t* kf, float dt) {
    KalmanFilter_TimeUpdate_Internal(kf, dt);
}

void KalmanFilter_Testable_Correct(
    KalmanFilter_Testable_t* kf, 
    float altitude, 
    float accel
) {
    KalmanFilter_MeasurementUpdate_Internal(kf, altitude, accel);
}

int KalmanFilter_Testable_IsApogeeDetected(KalmanFilter_Testable_t* kf) {
    return kf->apogee_detected;
}

float KalmanFilter_Testable_GetVelocity(KalmanFilter_Testable_t* kf) {
    return kf->x[1];
}

float KalmanFilter_Testable_GetAltitude(KalmanFilter_Testable_t* kf) {
    return kf->x[0];
}

float KalmanFilter_Testable_GetAcceleration(KalmanFilter_Testable_t* kf) {
    return kf->x[2];
}

void KalmanFilter_Testable_Reset(KalmanFilter_Testable_t* kf) {
    KalmanFilter_Testable_Init(kf);
}

float KalmanFilter_Testable_GetCovarianceTrace(KalmanFilter_Testable_t* kf) {
    return kf->P[0][0] + kf->P[1][1] + kf->P[2][2];
}
