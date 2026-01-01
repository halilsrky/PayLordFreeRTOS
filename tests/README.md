# Unit Test Infrastructure - PayLord Flight Computer

## Yapı Felsefesi

```
Production kodu ASLA test için bozulmaz.
Test kodu üretim binary'sine girmez.
Her test izole, deterministik ve tekrarlanabilir.
```

## Kapsam

| Modül | Test Dosyası | Öncelik |
|-------|--------------|---------|
| Kalman Filter | `test_kalman.c` | KRITIK |
| Flight State Machine | `test_flight_algorithm.c` | KRITIK |
| Apogee Detection | `test_apogee_logic.c` | KRITIK |
| BMI088 Dönüşümler | `test_bmi088_conversion.c` | YÜKSEK |

## Derleme

### Host (PC) üzerinde test
```bash
cd tests
make clean && make
./test_runner.exe
```

### CI/CD Pipeline
```bash
make test  # Exit code: 0=PASS, non-zero=FAIL
```

## Klasör Yapısı

```
tests/
├── README.md                    # Bu dosya
├── Makefile                     # Test build sistemi
├── test_runner.c                # Ana test entry point
├── test_config.h                # Test konfigürasyonu
│
├── framework/                   # Minimal test framework
│   ├── unity_minimal.h          # Assert makroları
│   └── unity_minimal.c          # Framework implementasyonu
│
├── testable/                    # Production kodun test edilebilir kopyası
│   ├── kalman_testable.c        # HAL/RTOS arındırılmış
│   ├── kalman_testable.h
│   ├── flight_algorithm_testable.c
│   ├── flight_algorithm_testable.h
│   ├── bmi088_conversion.c      # Sadece dönüşüm fonksiyonları
│   └── bmi088_conversion.h
│
├── test_cases/                  # Test implementasyonları
│   ├── test_kalman.c
│   ├── test_flight_algorithm.c
│   ├── test_apogee_logic.c
│   └── test_bmi088_conversion.c
│
├── vectors/                     # Test vektörleri
│   ├── kalman_vectors.h         # Kalman test inputları
│   ├── flight_state_vectors.h   # State machine senaryoları
│   └── bmi088_vectors.h         # Raw → Physical dönüşümler
│
└── mocks/                       # Stub/Mock tanımları (minimal)
    └── hal_stub.h               # HAL_GetTick stub
```

## Safety-Critical Notlar

- Apogee detection testleri en az 3 farklı senaryo içermeli
- Kalman divergence durumu test edilmeli
- State machine tüm transition'lar test edilmeli
- Edge case'ler (NaN, Inf, overflow) test edilmeli
