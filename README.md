# 🚀 SKYRTOS - STM32F446 Flight Computer

<div align="center">

**Rocket telemetry and flight control system - Teknofest Rocket Competition**

[Features](#-features) • [Hardware](#-hardware) • [Software](#-software-architecture) • [CI/CD](#-cicd) • [Test System](#-test-system-sitsut)

</div>

---

## 🌟 Overview

**SKYRTOS** (Sky Real-Time Operating System) is a professional flight computer firmware designed for rocket systems. It runs on STM32F446RET6 microcontroller with FreeRTOS operating system and is developed for Teknofest Rocket Competition.

### 🎯 Core Capabilities

- **Real-Time Data Processing**: FreeRTOS-based multitasking architecture
- **Sensor Fusion**: Optimal state estimation with Kalman filter
- **Flight Phase Detection**: Automatic launch, apogee, landing detection
- **Long-Range Telemetry**: 3-8km wireless communication via LoRa
- **High-Speed Data Logging**: SD card logging with FatFS
- **Test Infrastructure**: System validation with SIT/SUT modes

---

## ✨ Features

### 🎛️ Sensor Management
- ✅ **BME280**: Barometric sensor (pressure, temperature, humidity)
- ✅ **BMI088**: 6-axis IMU (3-axis accel + gyro)
- ✅ **HMC1021**: Magnetometer
- ✅ **L86 GPS**: Position and navigation
- ✅ **Kalman Filtering**: Sensor noise reduction

### 📡 Communication
- ✅ **E22 LoRa**: Long-range telemetry (433/868/915 MHz)
- ✅ **UART**: Debug and test modes
- ✅ **Packet Protocol**: Custom telemetry format

### 🧮 Algorithms
- ✅ **Quaternion Sensor Fusion**: Orientation calculation
- ✅ **Kalman Filtering**: Optimal state estimation
- ✅ **Flight Phase Detection**: 8-phase automatic detection
- ✅ **Mathematical Models**: Advanced calculations

### 💾 Data Management
- ✅ **FATFS**: FAT32 file system
- ✅ **SD Card**: High-speed SPI data logging
- ✅ **Binary/CSV Format**: Flexible log formats

### ⚡ Operating System
- ✅ **FreeRTOS**: Real-time multitasking
- ✅ **Task-Based Architecture**: Modular and scalable
- ✅ **Priority-Based Scheduling**: Critical task guarantee

### 🔍 Debugging
- ✅ **SEGGER SystemView**: Real-time system analysis
- ✅ **RTT**: High-speed debug output
- ✅ **Test Modes**: SIT/SUT hardware and software tests

---

## 🛠️ Hardware

### Microcontroller
- **MCU**: STM32F446RET6 (ARM Cortex-M4, 180MHz, 512KB Flash, 128KB RAM)
- **FPU**: Hardware floating-point unit
- **DMA**: 2x DMA controller

### Sensors
| Sensor | Model | Interface | Measurement |
|--------|-------|-----------|-------------|
| Barometric | BME280 | I²C | Pressure, temperature, humidity |
| IMU | BMI088 | SPI | 3-axis accel + gyro |
| Magnetometer | HMC1021 | Analog | 1-axis magnetic field |
| GPS | L86 | UART | Position, velocity |

### Communication
- **LoRa**: E22 module (433/868/915 MHz, 3-8km range)
- **Debug**: UART (115200 baud)
- **Storage**: SD card (SPI, FAT32)

---

## 🏗️ Software Architecture

### FreeRTOS Task Structure

| Task | Priority | Period | Function |
|------|----------|--------|----------|
| SensorTask | High | 10ms | BME280/BMI088 reading |
| FusionTask | High | 10ms | Kalman filter + sensor fusion |
| FlightTask | Medium | 100ms | Flight phase detection |
| TelemetryTask | Medium | 100ms | LoRa data transmission |
| LoggerTask | Low | 100ms | SD card data writing |
| TestModeTask | Medium | 100ms | SIT/SUT test modes |

### Layered Architecture

```
Application Layer
  ├── Flight Algorithm    (Flight phase detection)
  ├── Sensor Fusion       (Kalman + Quaternion)
  └── Data Logger         (SD card management)
        ↓
Middleware Layer
  ├── FreeRTOS           (RTOS kernel)
  ├── FATFS              (File system)
  └── Packet Protocol    (Telemetry format)
        ↓
Driver Layer
  ├── BME280/BMI088      (Sensor drivers)
  ├── E22 LoRa           (Communication driver)
  └── L86 GNSS           (GPS driver)
        ↓
HAL Layer (STM32 Hardware Abstraction)
```

---

## 📁 Project Structure

```
SKYRTOS/
├── Core/
│   ├── Inc/                      # Header files
│   │   ├── bme280.h, bmi088.h   # Sensor APIs
│   │   ├── sensor_fusion.h      # Kalman + Quaternion
│   │   ├── flight_algorithm.h   # Flight detection
│   │   ├── e22_lib.h            # LoRa driver
│   │   ├── data_logger.h        # SD card system
│   │   ├── uart_handler.h       # UART communication
│   │   └── test_modes.h         # SIT/SUT tests
│   │
│   └── Src/                      # Source files
│       ├── main.c, freertos.c   # Main application + tasks
│       ├── bme280.c, bmi088.c   # Sensor drivers
│       ├── sensor_fusion.c      # Fusion algorithms
│       ├── flight_algorithm.c   # Flight logic
│       ├── kalman.c             # Kalman filter
│       ├── quaternion.c         # Quaternion math
│       ├── packet.c             # Telemetry protocol
│       ├── data_logger.c        # Data logging system
│       ├── uart_handler.c       # Test communication
│       └── test_modes.c         # Test routines
│
├── Drivers/                      # STM32 HAL Drivers
├── FATFS/                        # FAT file system
├── Middlewares/Third_Party/
│   ├── FreeRTOS/                # RTOS kernel
│   └── SEGGER/                  # SystemView profiling
│
├── SIT_SUT/                      # Test system
│   ├── telemetry_app/           # Python Ground Station
│   ├── Datas/                   # Test CSVs
│   └── logs/                    # Test results
│
├── tests/                        # Unit tests
│   ├── test_kalman.c
│   ├── test_flight_algorithm.c
│   └── test_apogee_logic.c
│
├── docs/                         # Documentation
│   └── CI-CD-GUIDE.md           # CI/CD guide
│
├── build/                        # Build outputs
├── Makefile                      # Build system
└── README.md                     # This file
```

---

## 🎛️ Sensor Integration

### BME280 - Barometric Sensor
```c
BME_280_t sensor;
BME280_Init(&sensor, &hi2c1, BME280_I2C_ADDRESS_0);

float pressure = BME280_ReadPressure(&sensor);    // hPa
float temperature = BME280_ReadTemperature(&sensor); // °C
float altitude = BME280_CalculateAltitude(pressure); // m
```

### BMI088 - 6-Axis IMU
```c
bmi088_struct_t imu;
BMI088_Init(&imu, &hspi1);
BMI088_ReadAccel(&imu);
BMI088_ReadGyro(&imu);

float accel_z = imu.accel.z;  // m/s²
float gyro_x = imu.gyro.x;    // rad/s
```

### E22 - LoRa Module
```c
E22_Init(&huart2);
E22_SetFrequency(433.0); // MHz
E22_SetTxPower(30);      // dBm

uint8_t telemetry_packet[82];
E22_Transmit(telemetry_packet, sizeof(telemetry_packet));
```

---

## 🧮 Flight Algorithm

### Flight Phases

| Phase | Transition Criteria | Action |
|-------|---------------------|---------|
| IDLE | System ready | Waiting |
| ARMED | Sensor check OK | Ready for launch |
| POWERED | Accel > 3g | Motor burning |
| COASTING | Accel < 1.5g | Motor burnout |
| APOGEE | Velocity < 0 | Highest point |
| DROGUE_DESCENT | Apogee + delay | Deploy drogue parachute |
| MAIN_DESCENT | Altitude < 300m | Deploy main parachute |
| LANDED | Stable 5s | Landing completed |

### Sensor Fusion

**Kalman Filter (Altitude):**
```c
kalman_t kf;
kalman_init(&kf, initial_altitude);

// Every loop
float altitude_filtered = kalman_update(&kf, bme_altitude);
```

**Quaternion Fusion (Orientation):**
```c
quaternion_t q;
quaternion_init(&q);

// Update with IMU data
quaternion_update(&q, gyro_x, gyro_y, gyro_z, dt);
quaternion_normalize(&q);
```

---

## 💾 Data Logging System

### FATFS Configuration
- **File System**: FAT32
- **Sector Size**: 512 bytes
- **Write Speed**: ~500 KB/s
- **Logging Frequency**: 10Hz

### Data Format

**Binary Format:**
```c
typedef struct {
    uint32_t timestamp;
    float altitude;
    float pressure;
    float accel[3];
    float gyro[3];
    float lat, lon;
    uint8_t phase;
} __attribute__((packed)) LogEntry_t;
```

**Usage:**
```c
DataLogger_Init();
LogEntry_t entry = {/* ... */};
DataLogger_WriteEntry(&entry);
DataLogger_Close();
```

---

## 🔄 CI/CD

The project has a fully automated CI/CD pipeline with **GitHub Actions**:

### 🏗️ Build Workflow
- Compiles STM32 firmware (ARM GCC)
- Generates `.elf`, `.bin`, `.hex`
- Reports firmware size
- 30-day artifact retention

### 🔍 Code Quality Workflow
- Static analysis with Cppcheck
- Memory leak detection
- Code statistics
- Coding standard checks

### 🧪 Unit Tests Workflow
- Compiles and runs unit tests
- Test coverage analysis
- Reports test results

**Runs automatically on every push/PR:**
```bash
git push origin main
# → Build + Code Quality + Unit Tests (parallel)
```

### 📦 Artifact Download

[GitHub Actions](https://github.com/halilsrky/SKYRTOS/actions) → Successful workflow:
- `firmware-xxxxx` → Compiled firmware (`.elf`, `.bin`, `.hex`)
- `cppcheck-report` → Code analysis
- `test-results-xxxxx` → Test results

**Detailed documentation:** [docs/CI-CD-GUIDE.md](docs/CI-CD-GUIDE.md)

---

## 🧪 Test System (SIT/SUT)

SKYRTOS has two test modes for Teknofest Rocket Competition:

### 1️⃣ SIT Mode (Sensor Interface Test)

**Purpose:** Test if physical sensors are working correctly.

```
STM32 Sensors → UART → Ground Station → Real-time Display
     ↓
BME280/BMI088 (Real hardware reading)
```

**What it tests:**
- ✅ Sensor reading integrity
- ✅ UART communication
- ✅ Calibration accuracy
- ✅ Real-time telemetry

**Usage:**
```bash
cd SIT_SUT/telemetry_app
python app.py
# → Click "Start SIT" button
```

![SIT Mode](SIT_SUT/SIT_Mode.png)

---

### 2️⃣ SUT Mode (System Under Test)

**Purpose:** Test flight algorithms with synthetic data.

```
CSV File → Ground Station → UART → STM32 Algorithms
                                         ↓
                                  Flight Detection
                                         ↓
                              Status Bits (Events)
                                         ↓
                                Ground Station Display
```

**What it tests:**
- ✅ Flight phase detection algorithm
- ✅ Kalman filter performance
- ✅ Apogee detection accuracy
- ✅ Parachute deployment logic
- ✅ Algorithm robustness (noisy data)

**Events:**
| Bit | Event | Description |
|-----|-------|-------------|
| 0 | Launch Detect | Launch detected |
| 1 | Motor Burnout | Motor burnout completed |
| 2 | Apogee Detect | Highest point |
| 3 | Drogue Deploy | Drogue parachute deployed |
| 4 | Main Deploy | Main parachute deployed |
| 5 | Landing | Landing completed |

**Usage:**
```bash
cd SIT_SUT/telemetry_app
python app.py
# → Select CSV file
# → Click "Start SUT" button
# → Monitor events in real-time graph
```

![SUT Mode](SIT_SUT/SUT_Mode.png)

**Test Scenarios:**
- `az_gurultulu.csv` - Low noise (σ = 0.5m)
- `orta_gurultulu.csv` - Medium noise (σ = 2.0m)
- `cok_gurultulu.csv` - High noise (σ = 5.0m)

**Detailed documentation:** [SIT_SUT/README.md](SIT_SUT/README.md)

---

## 🔧 Development

### Build

```bash
# From command line
make clean && make -j$(nproc)

# From STM32CubeIDE
# Project → Build Project (Ctrl+B)
```

### Debug

**SEGGER SystemView:**
```c
SEGGER_SYSVIEW_Conf();
SEGGER_SYSVIEW_Start();
```

**GDB:**
```bash
arm-none-eabi-gdb build/SKYRTOS.elf
(gdb) target remote localhost:3333
(gdb) load
(gdb) continue
```

### Adding New Sensor/Driver

1. Add driver files to `Core/Inc/` and `Core/Src/`
2. Update `C_SOURCES` and `C_INCLUDES` in `Makefile`:
   ```makefile
   C_SOURCES = \
   Core/Src/main.c \
   Core/Src/new_driver.c \    # ← NEW
   ...
   ```
3. Create new task in `freertos.c` (if needed)
4. Build: `make clean && make`

---

## 📚 Documentation

- **[CI/CD Guide](docs/CI-CD-GUIDE.md)** - Automated build system
- **[SIT/SUT Test Guide](SIT_SUT/README.md)** - Test system documentation
- **[Unit Test Guide](tests/README.md)** - Unit test infrastructure

### Datasheets
- [STM32F446RE](https://www.st.com/resource/en/datasheet/stm32f446re.pdf)
- [BME280](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf)
- [BMI088](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)
- [FreeRTOS](https://www.freertos.org/Documentation/RTOS_book.html)

---


<div align="center">

**SKYRTOS** - 🚀

**Developer:** @halilsrky | **LinkedIn:** [Halil Sarıkaya](https://www.linkedin.com/in/halil-sarıkaya-3a777321b)

</div>
