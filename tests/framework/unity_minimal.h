/**
 * @file unity_minimal.h
 * @brief Minimal unit test framework - Safety-critical embedded systems
 * @note printf-free, dependency-free, deterministic
 * 
 * Tasarım Prensipleri:
 * - Sıfır external dependency
 * - printf/stdio KULLANILMAZ
 * - Her assert'ün fail lokasyonu kaydedilir
 * - Test izolasyonu garanti
 */

#ifndef UNITY_MINIMAL_H
#define UNITY_MINIMAL_H

#include "test_config.h"
#include <math.h>

/* ============================================================================
 *                         GLOBAL TEST STATE
 * ============================================================================ */

typedef struct {
    int total_tests;
    int passed;
    int failed;
    int skipped;
    
    /* Son fail bilgisi */
    const char* last_fail_file;
    int last_fail_line;
    const char* last_fail_msg;
    
    /* Mevcut test */
    const char* current_test;
    int current_test_failed;
} TestState_t;

extern TestState_t g_test_state;

/* ============================================================================
 *                         FRAMEWORK FUNCTIONS
 * ============================================================================ */

/**
 * @brief Framework başlat
 */
void test_framework_init(void);

/**
 * @brief Test başlamadan önce çağır
 */
void test_begin(const char* test_name);

/**
 * @brief Test bittikten sonra çağır
 * @return TEST_PASS veya TEST_FAIL
 */
TestResult_t test_end(void);

/**
 * @brief Tüm testler bittikten sonra özet
 * @return 0 = all pass, non-zero = fail count
 */
int test_summary(void);

/**
 * @brief Fail kaydet (internal)
 */
void test_fail_internal(const char* file, int line, const char* msg);

/**
 * @brief Test sonuç erişim fonksiyonları
 */
int test_get_pass_count(void);
int test_get_fail_count(void);
int test_get_total_count(void);
int test_get_skip_count(void);
const char* test_get_last_fail_file(void);
int test_get_last_fail_line(void);
const char* test_get_last_fail_msg(void);
int test_format_summary(char* buf, int buf_size);
int test_all_passed(void);

/* ============================================================================
 *                         ASSERT MAKROLARI
 * ============================================================================ */

/**
 * @brief Boolean assert
 * @example TEST_ASSERT(result == expected);
 */
#define TEST_ASSERT(condition) \
    do { \
        if (!(condition)) { \
            test_fail_internal(__FILE__, __LINE__, #condition); \
            return TEST_FAIL; \
        } \
    } while(0)

/**
 * @brief Boolean assert with message
 */
#define TEST_ASSERT_MSG(condition, msg) \
    do { \
        if (!(condition)) { \
            test_fail_internal(__FILE__, __LINE__, msg); \
            return TEST_FAIL; \
        } \
    } while(0)

/**
 * @brief Integer eşitlik
 */
#define TEST_ASSERT_EQUAL_INT(expected, actual) \
    do { \
        if ((expected) != (actual)) { \
            test_fail_internal(__FILE__, __LINE__, "INT mismatch"); \
            return TEST_FAIL; \
        } \
    } while(0)

/**
 * @brief Unsigned integer eşitlik
 */
#define TEST_ASSERT_EQUAL_UINT(expected, actual) \
    do { \
        if ((expected) != (actual)) { \
            test_fail_internal(__FILE__, __LINE__, "UINT mismatch"); \
            return TEST_FAIL; \
        } \
    } while(0)

/**
 * @brief Float eşitlik (epsilon toleranslı)
 */
#define TEST_ASSERT_FLOAT_WITHIN(tolerance, expected, actual) \
    do { \
        float _diff = fabsf((float)(expected) - (float)(actual)); \
        if (_diff > (tolerance)) { \
            test_fail_internal(__FILE__, __LINE__, "FLOAT out of tolerance"); \
            return TEST_FAIL; \
        } \
    } while(0)

/**
 * @brief Float eşitlik (varsayılan tolerance)
 */
#define TEST_ASSERT_FLOAT_EQUAL(expected, actual) \
    TEST_ASSERT_FLOAT_WITHIN(TEST_EPSILON_FLOAT, expected, actual)

/**
 * @brief Pointer NULL değil
 */
#define TEST_ASSERT_NOT_NULL(ptr) \
    do { \
        if ((ptr) == NULL) { \
            test_fail_internal(__FILE__, __LINE__, "NULL pointer"); \
            return TEST_FAIL; \
        } \
    } while(0)

/**
 * @brief Pointer NULL
 */
#define TEST_ASSERT_NULL(ptr) \
    do { \
        if ((ptr) != NULL) { \
            test_fail_internal(__FILE__, __LINE__, "Expected NULL"); \
            return TEST_FAIL; \
        } \
    } while(0)

/**
 * @brief Değer aralık içinde
 */
#define TEST_ASSERT_IN_RANGE(value, min, max) \
    do { \
        if ((value) < (min) || (value) > (max)) { \
            test_fail_internal(__FILE__, __LINE__, "Value out of range"); \
            return TEST_FAIL; \
        } \
    } while(0)

/**
 * @brief Array eşitlik (float)
 */
#define TEST_ASSERT_FLOAT_ARRAY_WITHIN(tolerance, expected, actual, len) \
    do { \
        for (size_t _i = 0; _i < (len); _i++) { \
            float _diff = fabsf((expected)[_i] - (actual)[_i]); \
            if (_diff > (tolerance)) { \
                test_fail_internal(__FILE__, __LINE__, "Array element mismatch"); \
                return TEST_FAIL; \
            } \
        } \
    } while(0)

/**
 * @brief NaN kontrolü
 */
#define TEST_ASSERT_NOT_NAN(value) \
    do { \
        if (isnan(value)) { \
            test_fail_internal(__FILE__, __LINE__, "Unexpected NaN"); \
            return TEST_FAIL; \
        } \
    } while(0)

/**
 * @brief Infinity kontrolü
 */
#define TEST_ASSERT_NOT_INF(value) \
    do { \
        if (isinf(value)) { \
            test_fail_internal(__FILE__, __LINE__, "Unexpected Inf"); \
            return TEST_FAIL; \
        } \
    } while(0)

/**
 * @brief Skip test (koşul sağlanmazsa)
 */
#define TEST_SKIP_IF(condition, reason) \
    do { \
        if (condition) { \
            g_test_state.skipped++; \
            return TEST_SKIP; \
        } \
    } while(0)

/* ============================================================================
 *                         TEST RUNNER MAKROLARI
 * ============================================================================ */

#define RUN_TEST(test_func) \
    do { \
        test_begin(#test_func); \
        TestResult_t _result = test_func(); \
        if (_result == TEST_PASS) { \
            test_end(); \
        } \
    } while(0)

#define RUN_TEST_GROUP(group_runner) \
    group_runner()

#endif /* UNITY_MINIMAL_H */
