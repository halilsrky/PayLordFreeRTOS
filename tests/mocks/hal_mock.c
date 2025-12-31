/**
 * @file hal_mock.c
 * @brief HAL mock implementation for unit testing
 * 
 * This file provides mock implementations of HAL functions
 * that are used by production code during unit testing.
 * 
 * USAGE:
 *   Tests include test_hal_interface.h which defines UNIT_TEST_MODE
 *   When UNIT_TEST_MODE is defined, hal_get_tick() uses g_mock_tick
 *   Test code can control time via mock_set_tick(), mock_advance_tick()
 */

#include <stdint.h>

/* ============================================================================
 *                         MOCK TICK COUNTER
 * ============================================================================ */

/**
 * @brief Global mock tick counter
 * 
 * This variable is used by hal_get_tick() when UNIT_TEST_MODE is defined.
 * Tests can set this directly or use the mock control functions.
 */
uint32_t g_mock_tick = 0;

/**
 * @brief Set mock tick to specific value
 * @param tick_value New tick value in milliseconds
 */
void mock_set_tick(uint32_t tick_value)
{
    g_mock_tick = tick_value;
}

/**
 * @brief Advance mock tick by specified milliseconds
 * @param delta_ms Milliseconds to advance
 */
void mock_advance_tick(uint32_t delta_ms)
{
    g_mock_tick += delta_ms;
}

/**
 * @brief Reset mock tick to zero
 */
void mock_reset_tick(void)
{
    g_mock_tick = 0;
}

/**
 * @brief Get current mock tick value
 * @return Current mock tick in milliseconds
 */
uint32_t mock_get_tick(void)
{
    return g_mock_tick;
}
