# PayLord Flight Computer - Unit Test Architecture

## 🎯 Temel İlke

> **"Test edilen kod uçmalı. Uçacak kod test edilmeli."**

Bu mimari, safety-critical sistemlerde kullanılan "what you test is what you fly" prensibiyle tasarlanmıştır.

## Mimari Genel Bakış

```
┌─────────────────────────────────────────────────────────────────┐
│                        Unit Test Suite                          │
│                     (tests/test_cases/*.c)                      │
└─────────────────────────────────────────┬───────────────────────┘
                                          │
                                          │ #include
                                          │ calls directly
                                          ▼
┌─────────────────────────────────────────────────────────────────┐
│                      PRODUCTION CODE                             │
│                     (Core/Src/*.c)                              │
│                     (Core/Inc/*.h)                              │
│                                                                 │
│  ┌──────────────┐  ┌────────────────┐  ┌────────────────────┐  │
│  │  kalman.c    │  │ flight_algo.c  │  │ bmi088_convert.h   │  │
│  │  (pure math) │  │ (HAL abstracted)│ │ (header-only)      │  │
│  └──────────────┘  └────────────────┘  └────────────────────┘  │
│         │                  │                                    │
│         │         hal_get_tick()                                │
│         │                  │                                    │
└─────────┼──────────────────┼────────────────────────────────────┘
          │                  │
          │                  ▼
          │    ┌──────────────────────────────┐
          │    │  test_hal_interface.h        │
          │    │  (Compile-time switch)       │
          │    │                              │
          │    │  #ifdef UNIT_TEST_MODE       │
          │    │    → use g_mock_tick         │
          │    │  #else                       │
          │    │    → use HAL_GetTick()       │
          │    └──────────────────────────────┘
          │
          └─────────────────────────────────────────────────────►
                                                    Directly testable
```

## Dosya Yapısı

```
tests/
├── Makefile                    # Build configuration
├── test_runner.c               # Main entry point
├── TEST_ARCHITECTURE.md        # Bu dosya
│
├── framework/                  # Test framework
│   ├── unity_minimal.h         # Assert macros
│   └── unity_minimal.c         # Framework impl
│
├── mocks/                      # HAL mock implementations
│   └── hal_mock.c              # g_mock_tick definition
│
├── test_cases/                 # Actual tests
│   ├── test_kalman.c           # Tests Core/Src/kalman.c
│   ├── test_bmi088_conversion.c# Tests Core/Inc/bmi088_conversions.h
│   ├── test_flight_algorithm.c # Tests Core/Src/flight_algorithm.c (pending)
│   └── test_apogee_logic.c     # Integration tests (pending)
│
├── vectors/                    # Test data
│   ├── kalman_vectors.h
│   ├── bmi088_vectors.h
│   └── flight_state_vectors.h
│
└── reference_model/            # ⚠️ NOT FOR TESTING
    └── README.md               # Warning documentation
```

## Production Code Changes

Production kodun test edilebilir olması için yapılan değişiklikler:

### 1. HAL Abstraction Layer (`Core/Inc/test_hal_interface.h`)

```c
#ifdef UNIT_TEST_MODE
    static inline uint32_t hal_get_tick(void) { return g_mock_tick; }
#else
    static inline uint32_t hal_get_tick(void) { return HAL_GetTick(); }
#endif
```

### 2. BMI088 Conversion Functions (`Core/Inc/bmi088_conversions.h`)

Pure math conversion functions extracted to header-only module:
- `bmi088_accel_raw_to_ms2()`
- `bmi088_gyro_raw_to_rads()`
- `bmi088_bytes_to_int16()`
- `bmi088_temp_raw_to_celsius()`

### 3. Modified Files

| File | Change |
|------|--------|
| `flight_algorithm.c` | `HAL_GetTick()` → `hal_get_tick()` |
| `bmi088.c` | `HAL_GetTick()` → `hal_get_tick()` |
| `kalman.c` | Already HAL-free (no changes needed) |

## Test Çalıştırma

```bash
cd tests/
make clean
make test
```

## Test Kategorileri

### ✅ Directly Testable (HAL-free)
- `kalman.c` - Pure math Kalman filter
- `bmi088_conversions.h` - Sensor unit conversions

### 🔧 Testable with Mocking
- `flight_algorithm.c` - State machine (uses `hal_get_tick()`)
- `bmi088.c` - Sensor processing (uses `hal_get_tick()`)

### ⏳ Requires Full HAL Mock (Future)
- DMA operations
- I2C/SPI communication
- GPIO control

## Standart Uyumluluğu

Bu mimari aşağıdaki safety standartlarıyla uyumludur:

| Standard | Requirement | How We Comply |
|----------|-------------|---------------|
| DO-178C | Test coverage on actual code | Tests call production code directly |
| IEC 61508 | Test = Deploy code | No "testable copies" |
| MISRA C | Code traceability | Same compilation units |

## Bakım Kuralları

1. **Yeni modül eklerken**: HAL bağımlılıklarını `test_hal_interface.h` üzerinden abstract et
2. **Test yazarken**: Her zaman production header'ı include et
3. **Asla**: `tests/reference_model/` dosyalarını test etme
4. **Her değişiklikte**: `make test` çalıştır

---
**Last Updated**: Production code testing architecture implemented
**Status**: Kalman + BMI088 tests operational, Flight Algorithm pending HAL mock completion
