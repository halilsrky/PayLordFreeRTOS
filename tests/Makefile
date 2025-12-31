# PayLord Flight Computer - Unit Test Makefile
# Safety-critical embedded software test build
#
# CRITICAL: "Test edilen kod uçmalı. Uçacak kod test edilmeli."
#           Tests compile and run PRODUCTION code directly.
#
# Kullanım:
#   make          - Build and run tests
#   make clean    - Temizle
#   make build    - Sadece build
#   make test     - Build ve çalıştır

# ==============================================================================
#                              CONFIGURATION
# ==============================================================================

# Compiler
CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -g -O0
CFLAGS += -DUNIT_TEST_MODE=1

# Windows için math library
LDFLAGS = -lm

# Output
TARGET = test_runner.exe

# ==============================================================================
#                              DIRECTORIES
# ==============================================================================

# Production code directories
PROD_SRC_DIR = ../Core/Src
PROD_INC_DIR = ../Core/Inc

# Test directories
FRAMEWORK_DIR = framework
TEST_CASES_DIR = test_cases
VECTORS_DIR = vectors
MOCKS_DIR = mocks

# Include paths - Production headers first!
INCLUDES = -I$(PROD_INC_DIR) -I. -I$(FRAMEWORK_DIR) -I$(VECTORS_DIR) -I$(MOCKS_DIR)

# ==============================================================================
#                              SOURCE FILES
# ==============================================================================

# Framework
FRAMEWORK_SRCS = $(FRAMEWORK_DIR)/unity_minimal.c

# Mock implementations for HAL functions
MOCK_SRCS = $(MOCKS_DIR)/hal_mock.c

# Production code (THE CODE THAT FLIES)
# Note: Only pure-math modules that don't depend on HAL/RTOS
PRODUCTION_SRCS = \
    $(PROD_SRC_DIR)/kalman.c

# Test cases
TEST_SRCS = \
    $(TEST_CASES_DIR)/test_kalman.c \
    $(TEST_CASES_DIR)/test_bmi088_conversion.c

# Note: test_flight_algorithm.c requires mocking due to complex dependencies
# Note: test_apogee_logic.c tests Kalman-based apogee detection

# Main runner
RUNNER_SRC = test_runner.c

# All sources
ALL_SRCS = $(FRAMEWORK_SRCS) $(MOCK_SRCS) $(PRODUCTION_SRCS) $(TEST_SRCS) $(RUNNER_SRC)

# Object files
OBJS = $(ALL_SRCS:.c=.o)

# ==============================================================================
#                              TARGETS
# ==============================================================================

.PHONY: all clean build test help

all: build test

build: $(TARGET)

$(TARGET): $(OBJS)
	@echo "Linking $(TARGET)..."
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS)
	@echo "Build complete: $(TARGET)"

%.o: %.c
	@echo "Compiling $<..."
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

test: $(TARGET)
	@echo ""
	@echo "Running unit tests..."
	@echo "================================"
	@./$(TARGET)
	@echo ""

clean:
	@echo "Cleaning..."
ifeq ($(OS),Windows_NT)
	-del /Q $(TARGET) 2>nul
	-del /Q *.o 2>nul
	-del /Q $(FRAMEWORK_DIR)\*.o 2>nul
	-del /Q $(TEST_CASES_DIR)\*.o 2>nul
	-del /Q $(MOCKS_DIR)\*.o 2>nul
	-del /Q $(PROD_SRC_DIR)\*.o 2>nul
else
	-rm -f $(TARGET)
	-rm -f *.o
	-rm -f $(FRAMEWORK_DIR)/*.o
	-rm -f $(TEST_CASES_DIR)/*.o
	-rm -f $(MOCKS_DIR)/*.o
	-rm -f $(PROD_SRC_DIR)/*.o
endif
	@echo "Clean complete"

help:
	@echo "PayLord Unit Test Build System"
	@echo ""
	@echo "CRITICAL: Tests run PRODUCTION code directly!"
	@echo "          'Test edilen kod ucmali. Ucacak kod test edilmeli.'"
	@echo ""
	@echo "Targets:"
	@echo "  all    - Build and run tests (default)"
	@echo "  build  - Build test executable only"
	@echo "  test   - Run tests (builds if needed)"
	@echo "  clean  - Remove all build artifacts"
	@echo "  help   - Show this help"
	@echo ""
	@echo "Exit codes:"
	@echo "  0 = All tests passed"
	@echo "  N = N tests failed"

# ==============================================================================
#                              DEPENDENCIES
# ==============================================================================

# Framework
$(FRAMEWORK_DIR)/unity_minimal.o: $(FRAMEWORK_DIR)/unity_minimal.c $(FRAMEWORK_DIR)/unity_minimal.h

# Mocks
$(MOCKS_DIR)/hal_mock.o: $(MOCKS_DIR)/hal_mock.c

# Production code dependencies
$(PROD_SRC_DIR)/kalman.o: $(PROD_SRC_DIR)/kalman.c $(PROD_INC_DIR)/kalman.h

# Test cases - depend on production headers
$(TEST_CASES_DIR)/test_kalman.o: $(TEST_CASES_DIR)/test_kalman.c $(PROD_INC_DIR)/kalman.h $(VECTORS_DIR)/kalman_vectors.h
$(TEST_CASES_DIR)/test_bmi088_conversion.o: $(TEST_CASES_DIR)/test_bmi088_conversion.c $(PROD_INC_DIR)/bmi088_conversions.h

# Runner
test_runner.o: test_runner.c $(FRAMEWORK_DIR)/unity_minimal.h
