/**
 * @file hal_stub.h
 * @brief HAL fonksiyonları için stub tanımları
 * @note Unit test'te HAL kullanılmaz, bu stub'lar derleme için gerekli
 */

#ifndef HAL_STUB_H
#define HAL_STUB_H

#include <stdint.h>

/* ============================================================================
 *                         HAL_GetTick STUB
 * ============================================================================ */

/**
 * @brief Simüle edilmiş tick sayacı
 * @note Test kodu bu değeri manuel kontrol eder
 */
static uint32_t g_stub_tick_count = 0;

/**
 * @brief HAL_GetTick stub - deterministik
 */
static inline uint32_t HAL_GetTick(void) {
    return g_stub_tick_count;
}

/**
 * @brief Tick sayacını ayarla
 */
static inline void stub_set_tick(uint32_t tick) {
    g_stub_tick_count = tick;
}

/**
 * @brief Tick sayacını artır
 */
static inline void stub_advance_tick(uint32_t delta) {
    g_stub_tick_count += delta;
}

/**
 * @brief Tick sayacını sıfırla
 */
static inline void stub_reset_tick(void) {
    g_stub_tick_count = 0;
}

/* ============================================================================
 *                         HAL_Delay STUB
 * ============================================================================ */

/**
 * @brief HAL_Delay stub - hiçbir şey yapmaz
 */
static inline void HAL_Delay(uint32_t delay) {
    /* Unit test'te delay yapılmaz */
    (void)delay;
}

/* ============================================================================
 *                         HAL_GPIO STUB
 * ============================================================================ */

typedef enum {
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET = 1
} GPIO_PinState;

typedef struct {
    uint32_t dummy;
} GPIO_TypeDef;

static inline void HAL_GPIO_WritePin(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState state) {
    (void)port;
    (void)pin;
    (void)state;
}

static inline GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* port, uint16_t pin) {
    (void)port;
    (void)pin;
    return GPIO_PIN_RESET;
}

/* ============================================================================
 *                         HAL_StatusTypeDef
 * ============================================================================ */

typedef enum {
    HAL_OK = 0,
    HAL_ERROR = 1,
    HAL_BUSY = 2,
    HAL_TIMEOUT = 3
} HAL_StatusTypeDef;

/* ============================================================================
 *                         I2C STUB (gerekirse)
 * ============================================================================ */

typedef struct {
    uint32_t dummy;
} I2C_HandleTypeDef;

typedef uint32_t IRQn_Type;

#define I2C_MEMADD_SIZE_8BIT    0x01

static inline HAL_StatusTypeDef HAL_I2C_Mem_Read(
    I2C_HandleTypeDef* hi2c, 
    uint16_t addr, 
    uint16_t mem_addr, 
    uint16_t mem_size, 
    uint8_t* data, 
    uint16_t size, 
    uint32_t timeout
) {
    (void)hi2c; (void)addr; (void)mem_addr; (void)mem_size;
    (void)data; (void)size; (void)timeout;
    return HAL_OK;
}

static inline HAL_StatusTypeDef HAL_I2C_Mem_Write(
    I2C_HandleTypeDef* hi2c, 
    uint16_t addr, 
    uint16_t mem_addr, 
    uint16_t mem_size, 
    uint8_t* data, 
    uint16_t size, 
    uint32_t timeout
) {
    (void)hi2c; (void)addr; (void)mem_addr; (void)mem_size;
    (void)data; (void)size; (void)timeout;
    return HAL_OK;
}

/* NVIC stub */
static inline void HAL_NVIC_DisableIRQ(IRQn_Type irq) { (void)irq; }
static inline void HAL_NVIC_EnableIRQ(IRQn_Type irq) { (void)irq; }

/* UNUSED makrosu */
#ifndef UNUSED
#define UNUSED(x) ((void)(x))
#endif

#endif /* HAL_STUB_H */
