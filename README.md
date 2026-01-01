# 🚀 SKYRTOS - STM32F446 Flight Computer

<div align="center">

**Roket telemetri ve uçuş kontrol sistemi - Teknofest Roket Yarışması**

[Özellikler](#-özellikler) • [Donanım](#-donanım) • [Yazılım](#-yazılım-mimarisi) • [CI/CD](#-cicd) • [Test Sistemi](#-test-sistemi-sitsut)

</div>

---

## 🌟 Genel Bakış

**SKYRTOS** (Sky Real-Time Operating System), roket sistemleri için tasarlanmış profesyonel bir uçuş bilgisayarı firmware'idir. STM32F446RET6 mikrodenetleyici üzerinde FreeRTOS işletim sistemi ile çalışır ve Teknofest Roket Yarışması için geliştirilmiştir.

### 🎯 Temel Yetenekler

- **Gerçek Zamanlı Veri İşleme**: FreeRTOS tabanlı çoklu görev mimarisi
- **Sensör Füzyonu**: Kalman filtresi ile optimal durum tahmini
- **Uçuş Fazı Algılama**: Otomatik launch, apogee, landing detection
- **Uzun Menzilli Telemetri**: LoRa ile 3-8km kablosuz iletişim
- **Yüksek Hızlı Veri Kaydı**: SD karta FatFS ile log yazma
- **Test Altyapısı**: SIT/SUT modları ile sistem doğrulama

---

## ✨ Özellikler

### 🎛️ Sensör Yönetimi
- ✅ **BME280**: Barometrik sensör (basınç, sıcaklık, nem)
- ✅ **BMI088**: 6 eksenli IMU (3-axis accel + gyro)
- ✅ **HMC1021**: Manyetometre
- ✅ **L86 GPS**: Konum ve navigasyon
- ✅ **Kalman Filtreleme**: Sensör gürültü azaltma

### 📡 İletişim
- ✅ **E22 LoRa**: Uzun menzilli telemetri (433/868/915 MHz)
- ✅ **UART**: Debug ve test modları
- ✅ **Paket Protokolü**: Özelleştirilmiş telemetri formatı

### 🧮 Algoritmalar
- ✅ **Quaternion Sensör Füzyonu**: Orientation hesaplama
- ✅ **Kalman Filtreleme**: Optimal durum tahmini
- ✅ **Uçuş Fazı Tespiti**: 8 fazlı otomatik algılama
- ✅ **Matematiksel Modeller**: İleri düzey hesaplama

### 💾 Veri Yönetimi
- ✅ **FATFS**: FAT32 dosya sistemi
- ✅ **SD Kart**: Yüksek hızlı SPI veri kaydı
- ✅ **Binary/CSV Format**: Esnek log formatları

### ⚡ İşletim Sistemi
- ✅ **FreeRTOS**: Gerçek zamanlı çoklu görev
- ✅ **Task Bazlı Mimari**: Modüler ve ölçeklenebilir
- ✅ **Öncelik Tabanlı Zamanlama**: Kritik görev garantisi

### 🔍 Debugging
- ✅ **SEGGER SystemView**: Gerçek zamanlı sistem analizi
- ✅ **RTT**: Yüksek hızlı debug çıkışı
- ✅ **Test Modları**: SIT/SUT donanım ve yazılım testleri

---

## 🛠️ Donanım

### Mikrodenetleyici
- **MCU**: STM32F446RET6 (ARM Cortex-M4, 180MHz, 512KB Flash, 128KB RAM)
- **FPU**: Hardware floating-point unit
- **DMA**: 2x DMA controller

### Sensörler
| Sensör | Model | Arayüz | Ölçüm |
|--------|-------|--------|-------|
| Barometrik | BME280 | I²C | Basınç, sıcaklık, nem |
| IMU | BMI088 | SPI | 3-axis accel + gyro |
| Manyetometre | HMC1021 | Analog | 1-axis magnetic field |
| GPS | L86 | UART | Konum, hız |

### İletişim
- **LoRa**: E22 modülü (433/868/915 MHz, 3-8km menzil)
- **Debug**: UART (115200 baud)
- **Depolama**: SD kart (SPI, FAT32)

---

## 🏗️ Yazılım Mimarisi

### FreeRTOS Task Yapısı

| Task | Öncelik | Periyot | Görev |
|------|---------|---------|-------|
| SensorTask | Yüksek | 10ms | BME280/BMI088 okuma |
| FusionTask | Yüksek | 10ms | Kalman filtre + sensör füzyon |
| FlightTask | Orta | 100ms | Uçuş fazı algılama |
| TelemetryTask | Orta | 100ms | LoRa veri gönderimi |
| LoggerTask | Düşük | 100ms | SD kart veri yazma |
| TestModeTask | Orta | 100ms | SIT/SUT test modları |

### Katmanlı Mimari

```
Application Layer
  ├── Flight Algorithm    (Uçuş fazı tespiti)
  ├── Sensor Fusion       (Kalman + Quaternion)
  └── Data Logger         (SD kart yönetimi)
        ↓
Middleware Layer
  ├── FreeRTOS           (RTOS kernel)
  ├── FATFS              (Dosya sistemi)
  └── Packet Protocol    (Telemetri formatı)
        ↓
Driver Layer
  ├── BME280/BMI088      (Sensör driver'ları)
  ├── E22 LoRa           (İletişim driver)
  └── L86 GNSS           (GPS driver)
        ↓
HAL Layer (STM32 Hardware Abstraction)
```

---

## 📁 Proje Yapısı

```
SKYRTOS/
├── Core/
│   ├── Inc/                      # Header dosyaları
│   │   ├── bme280.h, bmi088.h   # Sensör API'leri
│   │   ├── sensor_fusion.h      # Kalman + Quaternion
│   │   ├── flight_algorithm.h   # Uçuş algılama
│   │   ├── e22_lib.h            # LoRa driver
│   │   ├── data_logger.h        # SD kart sistemi
│   │   ├── uart_handler.h       # UART iletişim
│   │   └── test_modes.h         # SIT/SUT testleri
│   │
│   └── Src/                      # Kaynak dosyaları
│       ├── main.c, freertos.c   # Ana uygulama + tasks
│       ├── bme280.c, bmi088.c   # Sensör driver'ları
│       ├── sensor_fusion.c      # Füzyon algoritmaları
│       ├── flight_algorithm.c   # Uçuş mantığı
│       ├── kalman.c             # Kalman filtresi
│       ├── quaternion.c         # Quaternion matematik
│       ├── packet.c             # Telemetri protokolü
│       ├── data_logger.c        # Veri kayıt sistemi
│       ├── uart_handler.c       # Test iletişimi
│       └── test_modes.c         # Test rutinleri
│
├── Drivers/                      # STM32 HAL Driver'ları
├── FATFS/                        # FAT dosya sistemi
├── Middlewares/Third_Party/
│   ├── FreeRTOS/                # RTOS kernel
│   └── SEGGER/                  # SystemView profiling
│
├── SIT_SUT/                      # Test sistemi
│   ├── telemetry_app/           # Python Ground Station
│   ├── Datas/                   # Test CSV'leri
│   └── logs/                    # Test sonuçları
│
├── tests/                        # Unit testler
│   ├── test_kalman.c
│   ├── test_flight_algorithm.c
│   └── test_apogee_logic.c
│
├── docs/                         # Dokümantasyon
│   └── CI-CD-GUIDE.md           # CI/CD kılavuzu
│
├── build/                        # Build çıktıları
├── Makefile                      # Build sistemi
└── README.md                     # Bu dosya
```

---

## 🎛️ Sensör Entegrasyonu

### BME280 - Barometrik Sensör
```c
BME_280_t sensor;
BME280_Init(&sensor, &hi2c1, BME280_I2C_ADDRESS_0);

float pressure = BME280_ReadPressure(&sensor);    // hPa
float temperature = BME280_ReadTemperature(&sensor); // °C
float altitude = BME280_CalculateAltitude(pressure); // m
```

### BMI088 - 6 Eksenli IMU
```c
bmi088_struct_t imu;
BMI088_Init(&imu, &hspi1);
BMI088_ReadAccel(&imu);
BMI088_ReadGyro(&imu);

float accel_z = imu.accel.z;  // m/s²
float gyro_x = imu.gyro.x;    // rad/s
```

### E22 - LoRa Modülü
```c
E22_Init(&huart2);
E22_SetFrequency(433.0); // MHz
E22_SetTxPower(30);      // dBm

uint8_t telemetry_packet[82];
E22_Transmit(telemetry_packet, sizeof(telemetry_packet));
```

---

## 🧮 Uçuş Algoritması

### Uçuş Fazları

| Faz | Geçiş Kriteri | Aksiyon |
|-----|---------------|---------|
| IDLE | Sistem hazır | Beklemede |
| ARMED | Sensör kontrolü OK | Launch için hazır |
| POWERED | Accel > 3g | Motor yanıyor |
| COASTING | Accel < 1.5g | Motor söndü |
| APOGEE | Velocity < 0 | En yüksek nokta |
| DROGUE_DESCENT | Apogee + delay | Drogue paraşüt aç |
| MAIN_DESCENT | Altitude < 300m | Ana paraşüt aç |
| LANDED | Stable 5s | İniş tamamlandı |

### Sensör Füzyon

**Kalman Filtresi (Altitude):**
```c
kalman_t kf;
kalman_init(&kf, initial_altitude);

// Her döngüde
float altitude_filtered = kalman_update(&kf, bme_altitude);
```

**Quaternion Füzyon (Orientation):**
```c
quaternion_t q;
quaternion_init(&q);

// IMU verileriyle güncelle
quaternion_update(&q, gyro_x, gyro_y, gyro_z, dt);
quaternion_normalize(&q);
```

---

## 💾 Veri Kayıt Sistemi

### FATFS Konfigürasyonu
- **Dosya Sistemi**: FAT32
- **Sektör Boyutu**: 512 bytes
- **Yazma Hızı**: ~500 KB/s
- **Kayıt Frekansı**: 10Hz

### Veri Formatı

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

**Kullanım:**
```c
DataLogger_Init();
LogEntry_t entry = {/* ... */};
DataLogger_WriteEntry(&entry);
DataLogger_Close();
```

---

## 🔄 CI/CD

Proje **GitHub Actions** ile tam otomatik CI/CD pipeline'a sahiptir:

### 🏗️ Build Workflow
- STM32 firmware'ini derler (ARM GCC)
- `.elf`, `.bin`, `.hex` oluşturur
- Firmware boyutunu raporlar
- 30 gün artifact saklama

### 🔍 Code Quality Workflow
- Cppcheck ile statik analiz
- Memory leak detection
- Kod istatistikleri
- Coding standard kontrolü

### 🧪 Unit Tests Workflow
- Unit testleri derler ve çalıştırır
- Test coverage analizi
- Test sonuçlarını raporlar

**Her push/PR'da otomatik çalışır:**
```bash
git push origin main
# → Build + Code Quality + Unit Tests (paralel)
```

### 📦 Artifact İndirme

[GitHub Actions](https://github.com/halilsrky/SKYRTOS/actions) → Başarılı workflow:
- `firmware-xxxxx` → Derlenen firmware (`.elf`, `.bin`, `.hex`)
- `cppcheck-report` → Kod analizi
- `test-results-xxxxx` → Test sonuçları

**Detaylı dokümantasyon:** [docs/CI-CD-GUIDE.md](docs/CI-CD-GUIDE.md)

---

## 🧪 Test Sistemi (SIT/SUT)

SKYRTOS, Teknofest Roket Yarışması için iki test moduna sahiptir:

### 1️⃣ SIT Mode (Sensor Interface Test)

**Amaç:** Fiziksel sensörlerin doğru çalışıp çalışmadığını test etmek.

```
STM32 Sensors → UART → Ground Station → Real-time Display
     ↓
BME280/BMI088 (Gerçek donanım okuma)
```

**Ne test eder:**
- ✅ Sensör okuma bütünlüğü
- ✅ UART iletişim
- ✅ Kalibrasyon doğruluğu
- ✅ Real-time telemetri

**Kullanım:**
```bash
cd SIT_SUT/telemetry_app
python app.py
# → "Start SIT" butonuna tıkla
```

![SIT Mode](SIT_SUT/SIT_Mode.png)

---

### 2️⃣ SUT Mode (System Under Test)

**Amaç:** Uçuş algoritmalarını sentetik verilerle test etmek.

```
CSV File → Ground Station → UART → STM32 Algorithms
                                         ↓
                                  Flight Detection
                                         ↓
                              Status Bits (Events)
                                         ↓
                                Ground Station Display
```

**Ne test eder:**
- ✅ Uçuş fazı algılama algoritması
- ✅ Kalman filtre performansı
- ✅ Apogee detection doğruluğu
- ✅ Paraşüt açma logic
- ✅ Algorithm robustness (gürültülü veri)

**Event'ler:**
| Bit | Event | Açıklama |
|-----|-------|----------|
| 0 | Launch Detect | Kalkış algılandı |
| 1 | Motor Burnout | Motor yanması bitti |
| 2 | Apogee Detect | En yüksek nokta |
| 3 | Drogue Deploy | İlk paraşüt açıldı |
| 4 | Main Deploy | Ana paraşüt açıldı |
| 5 | Landing | İniş yapıldı |

**Kullanım:**
```bash
cd SIT_SUT/telemetry_app
python app.py
# → CSV dosyası seç
# → "Start SUT" butonuna tıkla
# → Event'leri real-time grafikte izle
```

![SUT Mode](SIT_SUT/SUT_Mode.png)

**Test Senaryoları:**
- `az_gurultulu.csv` - Düşük gürültü (σ = 0.5m)
- `orta_gurultulu.csv` - Orta gürültü (σ = 2.0m)
- `cok_gurultulu.csv` - Yüksek gürültü (σ = 5.0m)

**Detaylı dokümantasyon:** [SIT_SUT/README.md](SIT_SUT/README.md)

---

## 🔧 Geliştirme

### Derleme

```bash
# Komut satırından
make clean && make -j$(nproc)

# STM32CubeIDE'den
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

### Yeni Sensör/Driver Ekleme

1. `Core/Inc/` ve `Core/Src/` içine driver dosyalarını ekleyin
2. `Makefile`'da `C_SOURCES` ve `C_INCLUDES` güncelleyin:
   ```makefile
   C_SOURCES = \
   Core/Src/main.c \
   Core/Src/yeni_driver.c \    # ← YENİ
   ...
   ```
3. `freertos.c` içinde yeni task oluşturun (gerekirse)
4. Build edin: `make clean && make`

---

## 📚 Dokümantasyon

- **[CI/CD Kılavuzu](docs/CI-CD-GUIDE.md)** - Otomatik build sistemi
- **[SIT/SUT Test Rehberi](SIT_SUT/README.md)** - Test sistemi dokümantasyonu
- **[Unit Test Rehberi](tests/README.md)** - Unit test altyapısı

### Datasheet'ler
- [STM32F446RE](https://www.st.com/resource/en/datasheet/stm32f446re.pdf)
- [BME280](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf)
- [BMI088](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)
- [FreeRTOS](https://www.freertos.org/Documentation/RTOS_book.html)

---


<div align="center">

**SKYRTOS** - 🚀

**Geliştirici:** @halilsrky | **LinkedIn:** [Halil Sarıkaya](https://www.linkedin.com/in/halil-sarıkaya-3a777321b)

</div>
