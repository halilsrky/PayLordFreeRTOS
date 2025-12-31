/**
 * @file unity_minimal.c
 * @brief Minimal unit test framework implementasyonu
 * @note printf-free implementation
 */

#include "unity_minimal.h"

/* ============================================================================
 *                         GLOBAL STATE
 * ============================================================================ */

TestState_t g_test_state = {0};

/* Output buffer for results (no printf) */
static char output_buffer[256];
static int output_index = 0;

/* ============================================================================
 *                         INTERNAL HELPERS
 * ============================================================================ */

/**
 * @brief Integer to string (printf-free)
 */
static int int_to_str(int value, char* buf, int buf_size) {
    int i = 0;
    int is_negative = 0;
    
    if (buf_size < 2) return 0;
    
    if (value < 0) {
        is_negative = 1;
        value = -value;
    }
    
    /* Reverse digits */
    char temp[16];
    int temp_i = 0;
    
    do {
        temp[temp_i++] = '0' + (value % 10);
        value /= 10;
    } while (value > 0 && temp_i < 15);
    
    if (is_negative && i < buf_size - 1) {
        buf[i++] = '-';
    }
    
    /* Copy reversed */
    while (temp_i > 0 && i < buf_size - 1) {
        buf[i++] = temp[--temp_i];
    }
    
    buf[i] = '\0';
    return i;
}

/**
 * @brief String copy (printf-free)
 */
static int str_copy(char* dst, const char* src, int max_len) {
    int i = 0;
    while (src[i] != '\0' && i < max_len - 1) {
        dst[i] = src[i];
        i++;
    }
    dst[i] = '\0';
    return i;
}

/* ============================================================================
 *                         FRAMEWORK FUNCTIONS
 * ============================================================================ */

void test_framework_init(void) {
    g_test_state.total_tests = 0;
    g_test_state.passed = 0;
    g_test_state.failed = 0;
    g_test_state.skipped = 0;
    g_test_state.last_fail_file = NULL;
    g_test_state.last_fail_line = 0;
    g_test_state.last_fail_msg = NULL;
    g_test_state.current_test = NULL;
    g_test_state.current_test_failed = 0;
    output_index = 0;
}

void test_begin(const char* test_name) {
    g_test_state.current_test = test_name;
    g_test_state.current_test_failed = 0;
    g_test_state.total_tests++;
}

TestResult_t test_end(void) {
    if (g_test_state.current_test_failed) {
        g_test_state.failed++;
        return TEST_FAIL;
    } else {
        g_test_state.passed++;
        return TEST_PASS;
    }
}

void test_fail_internal(const char* file, int line, const char* msg) {
    g_test_state.current_test_failed = 1;
    g_test_state.last_fail_file = file;
    g_test_state.last_fail_line = line;
    g_test_state.last_fail_msg = msg;
    g_test_state.failed++;
}

int test_summary(void) {
    /* Return fail count as exit code */
    return g_test_state.failed;
}

/* ============================================================================
 *                         RESULT ACCESS FUNCTIONS
 * ============================================================================ */

/**
 * @brief Test sonuçlarını al
 */
int test_get_pass_count(void) {
    return g_test_state.passed;
}

int test_get_fail_count(void) {
    return g_test_state.failed;
}

int test_get_total_count(void) {
    return g_test_state.total_tests;
}

int test_get_skip_count(void) {
    return g_test_state.skipped;
}

const char* test_get_last_fail_file(void) {
    return g_test_state.last_fail_file;
}

int test_get_last_fail_line(void) {
    return g_test_state.last_fail_line;
}

const char* test_get_last_fail_msg(void) {
    return g_test_state.last_fail_msg;
}

/**
 * @brief Sonuç string'i oluştur
 * @param buf Output buffer
 * @param buf_size Buffer boyutu
 * @return Yazılan karakter sayısı
 */
int test_format_summary(char* buf, int buf_size) {
    int i = 0;
    
    /* "TESTS: X PASS: Y FAIL: Z" formatı */
    const char* label1 = "TESTS: ";
    i += str_copy(&buf[i], label1, buf_size - i);
    i += int_to_str(g_test_state.total_tests, &buf[i], buf_size - i);
    
    const char* label2 = " PASS: ";
    i += str_copy(&buf[i], label2, buf_size - i);
    i += int_to_str(g_test_state.passed, &buf[i], buf_size - i);
    
    const char* label3 = " FAIL: ";
    i += str_copy(&buf[i], label3, buf_size - i);
    i += int_to_str(g_test_state.failed, &buf[i], buf_size - i);
    
    if (g_test_state.skipped > 0) {
        const char* label4 = " SKIP: ";
        i += str_copy(&buf[i], label4, buf_size - i);
        i += int_to_str(g_test_state.skipped, &buf[i], buf_size - i);
    }
    
    return i;
}

/**
 * @brief Tüm testler pass mı?
 */
int test_all_passed(void) {
    return (g_test_state.failed == 0) ? 1 : 0;
}
