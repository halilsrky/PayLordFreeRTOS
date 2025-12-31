/**
 * @file test_runner.c
 * @brief Unit test main entry point - PRODUCTION CODE TESTS
 * 
 * CRITICAL: "Test edilen kod uçmalı. Uçacak kod test edilmeli."
 *           These tests verify the ACTUAL production code in Core/Src/
 * 
 * Kullanım:
 *   ./test_runner.exe
 * 
 * Exit codes:
 *   0 = All tests passed
 *   N = N tests failed
 */

#include "framework/unity_minimal.h"
#include <stdio.h>

/* ============================================================================
 *                         EXTERNAL TEST GROUP RUNNERS
 * ============================================================================ */

extern void run_kalman_tests(void);
extern void run_bmi088_conversion_tests(void);

/* Note: flight_algorithm tests disabled - requires HAL mocking */
/* extern void run_flight_algorithm_tests(void); */
/* extern void run_apogee_logic_tests(void); */

/* ============================================================================
 *                         RESULT OUTPUT
 * ============================================================================ */

/**
 * @brief Test sonuçlarını stdout'a yaz
 * @note Bu tek printf kullanımı - sadece final sonuç için
 */
static void print_results(void) {
    char summary[128];
    test_format_summary(summary, sizeof(summary));
    
    printf("\n");
    printf("========================================\n");
    printf("          UNIT TEST RESULTS\n");
    printf("========================================\n");
    printf("%s\n", summary);
    printf("========================================\n");
    
    if (test_all_passed()) {
        printf("STATUS: ALL TESTS PASSED\n");
    } else {
        printf("STATUS: TESTS FAILED\n");
        if (test_get_last_fail_file() != NULL) {
            printf("Last failure:\n");
            printf("  File: %s\n", test_get_last_fail_file());
            printf("  Line: %d\n", test_get_last_fail_line());
            printf("  Reason: %s\n", test_get_last_fail_msg());
        }
    }
    printf("========================================\n\n");
}

/* ============================================================================
 *                         MAIN
 * ============================================================================ */

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;
    
    /* Initialize test framework */
    test_framework_init();
    
    printf("\n");
    printf("========================================\n");
    printf("PayLord Flight Computer - Unit Tests\n");
    printf("PRODUCTION CODE TESTING\n");
    printf("'Test edilen kod ucmali.'\n");
    printf("========================================\n\n");
    
    /* ======================================================================
     * TEST GROUP 1: Kalman Filter (Pure math - no HAL dependencies)
     * Tests: Core/Src/kalman.c
     * ====================================================================== */
    printf("[GROUP] Kalman Filter Tests (Production Code)...\n");
    run_kalman_tests();
    
    /* ======================================================================
     * TEST GROUP 2: BMI088 Conversions (Pure math - header-only)
     * Tests: Core/Inc/bmi088_conversions.h
     * ====================================================================== */
    printf("\n[GROUP] BMI088 Conversion Tests (Production Code)...\n");
    run_bmi088_conversion_tests();
    
    /* Note: Flight Algorithm tests require full HAL mocking infrastructure */
    /* TODO: Implement HAL mock layer for flight_algorithm testing */
    
    /* Print final results */
    print_results();
    
    /* Return fail count as exit code */
    return test_summary();
}
