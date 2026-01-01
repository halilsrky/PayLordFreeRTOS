# 🚀 SKYRTOS - STM32F446 Flight Computer

<div align="center">

![STM32](https://img.shields.io/badge/STM32-F446RET6-blue?style=for-the-badge&logo=stmicroelectronics)
![FreeRTOS](https://img.shields.io/badge/FreeRTOS-Real--Time-green?style=for-the-badge)
![LoRa](https://img.shields.io/badge/LoRa-E22-orange?style=for-the-badge)
![Build](https://img.shields.io/badge/Build-Passing-brightgreen?style=for-the-badge)

**Roket telemetri ve uçuş kontrol sistemi - Teknofest Roket Yarışması**

[Özellikler](#-özellikler) • [Donanım](#%EF%B8%8F-donanım) • [Yazılım](#-yazılım-mimarisi) • [CI/CD](#-cicd) • [Test Sistemi](#-test-sistemi-sitsut)

</div>

---

## 📋 İçindekiler

- [Genel Bakış](#-genel-bakış)
- [Özellikler](#-özellikler)
- [Donanım](#%EF%B8%8F-donanım)
- [Yazılım Mimarisi](#-yazılım-mimarisi)
- [Proje Yapısı](#-proje-yapısı)
- [Sensör Entegrasyonu](#-sensör-entegrasyonu)
- [Uçuş Algoritması](#-uçuş-algoritması)
- [Veri Kayıt Sistemi](#-veri-kayıt-sistemi)
- [CI/CD](#-cicd)
- [Test Sistemi (SIT/SUT)](#-test-sistemi-sitsut)
- [Geliştirme](#-geliştirme)

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
- ✅ **BME280**: Çevresel sensör (sıcaklık, nem, basınç)
- ✅ **BMI088**: 6 eksenli IMU (3 eksen ivmeölçer + 3 eksen jiroskop)
- ✅ **HMC1021**: Tek eksenli manyetometre
- ✅ **L86 GPS/GNSS**: Konum ve navigasyon verileri
- ✅ **Kalman Filtreleme**: Sensör gürültü azaltma ve veri iyileştirme

### 📡 İletişim ve Telemetri
- ✅ **E22 LoRa Modülü**: Uzun menzilli kablosuz iletişim (433MHz/868MHz/915MHz)
- ✅ **UART**: Debug ve telemetri çıkışı
- ✅ **Paket Protokolü**: Özelleştirilmiş telemetri paketi formatı
- ✅ **Gerçek Zamanlı Veri Akışı**: Düşük gecikmeli telemetri

### 🧮 Algoritmalar ve İşleme
- ✅ **Quaternion Tabanlı Sensör Füzyonu**: Orientation hesaplama
- ✅ **Kalman Filtreleme**: Optimal durum tahmini
- ✅ **Uçuş Fazı Tespiti**: Otomatik uçuş durumu algılama
- ✅ **Matematiksel Modeller**: İleri düzey hesaplama kütüphaneleri

### 💾 Veri Yönetimi
- ✅ **FATFS**: FAT dosya sistemi desteği
- ✅ **SD Kart**: Yüksek hızlı veri kayıt sistemi
- ✅ **SPI İletişimi**: Hızlı veri transferi
- ✅ **Yapılandırılabilir Log Formatları**: Esnek veri kayıt seçenekleri

### ⚡ İşletim Sistemi
- ✅ **FreeRTOS**: Gerçek zamanlı çoklu görev yönetimi
- ✅ **Task Bazlı Mimari**: Modüler ve ölçeklenebilir tasarım
- ✅ **Öncelik Tabanlı Zamanlama**: Kritik görevlerin garanti edilmesi
- ✅ **Inter-Task İletişim**: Queue ve semaphore mekanizmaları

### 🔍 Hata Ayıklama ve İzleme
- ✅ **SEGGER SystemView**: Gerçek zamanlı sistem analizi
- ✅ **RTT (Real-Time Transfer)**: Yüksek hızlı debug çıkışı
- ✅ **Test Modları**: Donanım ve sensör test rutinleri

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

## 🚀 Hızlı Başlangıç

### 📋 Gereksinimler

#### Yazılım
- **STM32CubeIDE** v1.14.1 veya üzeri
- **STM32CubeMX** (opsiyonel, zaten yapılandırılmış)
- **ARM GCC Toolchain** (STM32CubeIDE ile birlikte gelir)
- **Git** (versiyon kontrolü için)

#### Donanım
- **STM32F446RET6** geliştirme kartı veya özel PCB
- **ST-Link V2** veya **J-Link** programlayıcı
- **USB Kablo** (programlama için)
- Sensör modülleri (BME280, BMI088, L86, E22)
- **SD Kart** (veri kaydı için)

### 📥 Kurulum

#### 1. Projeyi Klonlama

```bash
git clone https://github.com/halilsrky/PayLord.git
cd PayLord
```

#### 2. STM32CubeIDE'de Açma

1. STM32CubeIDE'yi başlatın
2. **File → Open Projects from File System...**
3. Proje dizinini seçin
4. **Finish** tıklayın

#### 3. Derleme

```bash
# Komut satırından (opsiyonel)
cd Debug
make clean
make all

# Veya IDE'den
# Project → Build Project (Ctrl+B)
```

#### 4. Yükleme

1. STM32 kartı ST-Link ile bilgisayara bağlayın
2. **Run → Debug** (F11) veya **Run → Run** (Ctrl+F11)
3. Firmware otomatik olarak yüklenecektir

### ⚙️ İlk Yapılandırma

#### Pin Bağlantıları

**I²C1 (BME280 için):**
```
PB8  → SCL
PB9  → SDA
```

**SPI1 (BMI088 için):**
```
PA5  → SCK
PA6  → MISO
PA7  → MOSI
PA4  → CS_ACCEL
PB0  → CS_GYRO
```

**SPI2 (SD Kart için):**
```
PB13 → SCK
PB14 → MISO
PB15 → MOSI
PB12 → CS
```

**UART (E22 LoRa & Debug):**
```
PA2  → UART2_TX (E22)
PA3  → UART2_RX (E22)
PA9  → UART1_TX (Debug)
PA10 → UART1_RX (Debug)
```

**UART (L86 GPS):**
```
PC10 → UART3_TX
PC11 → UART3_RX
```

Detaylı pin yapılandırması için [PayLordFreeRTOS.ioc](PayLordFreeRTOS.ioc) dosyasına bakın.

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

### BME280 - Çevresel Sensör

**Özellikler:**
- **Basınç**: 300-1100 hPa (±1 hPa hassasiyet)
- **Sıcaklık**: -40°C ile +85°C (±1°C hassasiyet)
- **Nem**: 0-100% (±3% hassasiyet)
- **Arayüz**: I²C (0x76 veya 0x77 adresi)

**Kullanım:**
```c
#include "bme280.h"

BME_280_t sensor;
BME280_Init(&sensor, &hi2c1, BME280_I2C_ADDRESS_0);

float pressure = BME280_ReadPressure(&sensor);
float temperature = BME280_ReadTemperature(&sensor);
float humidity = BME280_ReadHumidity(&sensor);
float altitude = BME280_CalculateAltitude(pressure);
```

### BMI088 - 6 Eksenli IMU

**Özellikler:**
- **İvmeölçer**: ±3g/±6g/±12g/±24g (16-bit)
- **Jiroskop**: ±125°/s - ±2000°/s (16-bit)
- **Veri Hızı**: 1600Hz (accel), 2000Hz (gyro)
- **Arayüz**: SPI (20MHz'e kadar)

**Kullanım:**
```c
#include "bmi088.h"

bmi088_struct_t imu;
BMI088_Init(&imu, &hspi1);

BMI088_ReadAccel(&imu);
BMI088_ReadGyro(&imu);

float accel_x = imu.accel.x;  // g cinsinden
float gyro_z = imu.gyro.z;    // rad/s cinsinden
```

### L86 - GPS/GNSS Modülü

**Özellikler:**
- **Hassasiyet**: 2.5m CEP
- **Update Rate**: 1Hz (varsayılan), 10Hz'e kadar destekler
- **Hot Start**: < 1s
- **Cold Start**: < 32s
- **Protokol**: NMEA 0183

**Kullanım:**
```c
#include "l86_gnss.h"

L86_Data_t gps_data;
L86_Init(&huart3);

if (L86_ParseData(&gps_data)) {
    float latitude = gps_data.latitude;
    float longitude = gps_data.longitude;
    float altitude = gps_data.altitude;
    float speed = gps_data.speed;
}
```

### E22 - LoRa Modülü

**Özellikler:**
- **Frekans**: 433MHz / 868MHz / 915MHz
- **Menzil**: 3km (şehir), 8km (açık alan)
- **TX Power**: 30dBm (1W)
- **Veri Hızı**: 0.3-19.2kbps
- **Arayüz**: UART

**Kullanım:**
```c
#include "e22_lib.h"

E22_Init(&huart2);
E22_SetFrequency(433.0); // MHz
E22_SetTxPower(30);      // dBm

uint8_t data[32];
E22_Transmit(data, sizeof(data));
```

---

## 📡 İletişim Protokolleri

### Telemetri Paketi Formatı

```c
typedef struct {
    // Header (8 bytes)
    uint16_t sync_word;       // 0xAA55
    uint16_t packet_id;       // Paket numarası
    uint32_t timestamp;       // ms cinsinden zaman

    // Sensör Verileri (48 bytes)
    float accel[3];           // m/s² (x, y, z)
    float gyro[3];            // rad/s (roll, pitch, yaw)
    float mag[3];             // µT (x, y, z)
    float pressure;           // hPa
    float temperature;        // °C
    float altitude;           // m
    
    // GPS Verileri (16 bytes)
    float latitude;           // derece
    float longitude;          // derece
    float gps_altitude;       // m
    float speed;              // m/s
    
    // Durum Verileri (8 bytes)
    uint8_t flight_phase;     // Uçuş fazı
    uint8_t system_status;    // Sistem durumu
    uint16_t battery_voltage; // mV
    uint32_t flight_time;     // ms
    
    // CRC (2 bytes)
    uint16_t crc;             // CRC16 checksum
} __attribute__((packed)) TelemetryPacket_t;
```

**Toplam Paket Boyutu**: 82 bytes

### LoRa İletim Parametreleri

| Parametre | Değer | Açıklama |
|-----------|-------|----------|
| **Frekans** | 433 MHz | ISM bandı |
| **Bandwidth** | 125 kHz | Spektrum genişliği |
| **Spreading Factor** | 9 | Menzil/hız dengesi |
| **Coding Rate** | 4/5 | Hata düzeltme |
| **TX Power** | 20 dBm | İletim gücü |
| **Veri Hızı** | ~1000 bps | Efektif hız |

---

## 🧮 Uçuş Algoritması

### Uçuş Fazları

```c
typedef enum {
    PHASE_IDLE,          // Beklemede
    PHASE_ARMED,         // Hazır (ateşleme için)
    PHASE_POWERED,       // Motor yanıyor
    PHASE_COASTING,      // Serbest yükselme
    PHASE_APOGEE,        // En yüksek nokta
    PHASE_DROGUE_DESCENT,// Drogue paraşüt açık
    PHASE_MAIN_DESCENT,  // Ana paraşüt açık
    PHASE_LANDED         // İniş tamamlandı
} FlightPhase_t;
```

### Faz Geçiş Kriterleri

#### IDLE → ARMED
```c
// Manuel arming veya otomatik sensör kontrolü
if (all_sensors_ok && system_ready) {
    phase = PHASE_ARMED;
}
```

#### ARMED → POWERED
```c
// Yüksek ivme tespit edildi (motor ateşlendi)
if (total_accel > LAUNCH_THRESHOLD) {  // > 3g
    phase = PHASE_POWERED;
    flight_start_time = HAL_GetTick();
}
```

#### POWERED → COASTING
```c
// İvme düştü (motor söndü)
if (total_accel < BURNOUT_THRESHOLD) {  // < 1.5g
    phase = PHASE_COASTING;
}
```

#### COASTING → APOGEE
```c
// Dikey hız negatife döndü
if (vertical_velocity < 0) {
    phase = PHASE_APOGEE;
    max_altitude = current_altitude;
}
```

#### APOGEE → DROGUE_DESCENT
```c
// Drogue paraşüt ayrılması (otomatik veya manuel)
if (time_since_apogee > DROGUE_DELAY) {
    phase = PHASE_DROGUE_DESCENT;
    // Pyro kanal aktivasyonu
}
```

#### DROGUE_DESCENT → MAIN_DESCENT
```c
// Ana paraşüt açılma irtifası
if (altitude < MAIN_DEPLOY_ALTITUDE) {  // < 300m
    phase = PHASE_MAIN_DESCENT;
    // Ana paraşüt pyro aktivasyonu
}
```

#### MAIN_DESCENT → LANDED
```c
// Yer tespiti
if (altitude_change < 1.0 && time_stable > 5000) {
    phase = PHASE_LANDED;
}
```

### Sensör Füzyon Algoritması

**Quaternion Tabanlı Füzyon:**

```c
void sensor_fusion_update(sensor_fusion_t* sf, 
                         float ax, float ay, float az,
                         float gx, float gy, float gz,
                         float dt) {
    // 1. Jiroskop entegrasyonu
    quaternion_t q_gyro = integrate_gyroscope(gx, gy, gz, dt);
    
    // 2. İvmeölçer düzeltmesi
    quaternion_t q_accel = accel_correction(ax, ay, az);
    
    // 3. Tamamlayıcı filtre
    sf->orientation = complementary_filter(q_gyro, q_accel, 0.98);
    
    // 4. Euler açıları hesaplama
    quaternion_to_euler(&sf->orientation, 
                       &sf->roll, &sf->pitch, &sf->yaw);
}
```

**Kalman Filtresi (İrtifa):**

```c
void kalman_update(kalman_t* kf, float measurement) {
    // Prediction step
    kf->x = kf->x + kf->v * dt;        // x = x + v*dt
    kf->P = kf->P + kf->Q;             // P = P + Q
    
    // Update step
    float K = kf->P / (kf->P + kf->R); // Kalman gain
    kf->x = kf->x + K * (measurement - kf->x);
    kf->P = (1 - K) * kf->P;
}
```

---

## 💾 Veri Kayıt Sistemi

### FATFS Konfigürasyonu

**Dosya Sistemi**: FAT32  
**Sektör Boyutu**: 512 bytes  
**Cluster Boyutu**: 4KB (önerilen)  
**Maksimum Dosya Boyutu**: 4GB

### Veri Kayıt Formatı

#### CSV Format (İnsan Okunabilir)
```csv
Timestamp,Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z,Pressure,Temp,Alt,Lat,Lon,Phase
1000,0.05,0.03,9.81,0.001,0.002,0.003,1013.25,25.4,100.5,41.0082,28.9784,1
1100,0.06,0.04,9.82,0.002,0.003,0.004,1012.10,25.3,115.2,41.0083,28.9785,1
```

#### Binary Format (Kompakt)
```c
typedef struct {
    uint32_t timestamp;
    float accel[3];
    float gyro[3];
    float mag[3];
    float pressure;
    float temperature;
    float altitude;
    float latitude;
    float longitude;
    uint8_t phase;
} __attribute__((packed)) LogEntry_t;
```

### Kullanım

```c
#include "data_logger.h"

// Başlat
DataLogger_Init();

// Veri yaz
LogEntry_t entry;
entry.timestamp = HAL_GetTick();
// ... diğer alanları doldur
DataLogger_WriteEntry(&entry);

// Dosyayı kapat
DataLogger_Close();
```

**Performans:**
- Yazma Hızı: ~500 KB/s
- Buffer Boyutu: 512 bytes
- Kayıt Frekansı: 10Hz (her 100ms)

---

## 🔧 Geliştirme

### Derleme Ortamı

**Toolchain:**
```
arm-none-eabi-gcc (GNU Arm Embedded Toolchain)
Version: 10.3.1
```

**Derleme Bayrakları:**
```makefile
CFLAGS = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS += -O2 -g3 -Wall -Wextra
CFLAGS += -DUSE_HAL_DRIVER -DSTM32F446xx
CFLAGS += -DARM_MATH_CM4 -D__FPU_PRESENT=1
```

### Debug Konfigürasyonu

**SEGGER SystemView:**

1. RTT buffer konfigürasyonu:
```c
#define SEGGER_SYSVIEW_RTT_BUFFER_SIZE 1024
#define SEGGER_SYSVIEW_RTT_CHANNEL 1
```

2. SystemView başlatma:
```c
SEGGER_SYSVIEW_Conf();
SEGGER_SYSVIEW_Start();
```

3. Task izleme:
```c
SEGGER_SYSVIEW_OnTaskCreate(task_handle);
SEGGER_SYSVIEW_OnTaskStartExec(task_id);
```

**GDB Debug:**
```bash
arm-none-eabi-gdb PayLordFreeRTOS.elf
(gdb) target remote localhost:3333
(gdb) monitor reset halt
(gdb) load
(gdb) continue
```

### Test Modları

**Sensör Test:**
```c
void test_sensors(void) {
    printf("Testing BME280...\n");
    BME280_Test(&BME280_sensor);
    
    printf("Testing BMI088...\n");
    BMI088_Test(&BMI_sensor);
    
    printf("Testing L86 GPS...\n");
    L86_Test(&gps_data);
}
```

**LoRa Range Test:**
```c
void test_lora_range(void) {
    for (int i = 0; i < 100; i++) {
        E22_Transmit((uint8_t*)&i, sizeof(i));
        HAL_Delay(1000);
    }
}
```

---

## 🔄 CI/CD

Proje **3 adet GitHub Actions workflow** ile tam otomatik CI/CD pipeline'a sahiptir:

### 🏗️ 1. Build Workflow
![Build Status](https://github.com/halilsrky/PayLord/actions/workflows/build.yml/badge.svg)

**Ne yapar:**
- STM32 firmware'ini derler (ARM GCC)
- `.elf`, `.bin`, `.hex` dosyalarını oluşturur
- Firmware boyutunu raporlar
- Artifact olarak saklar (30 gün)

**Ne zaman:** Her push, PR, veya manuel tetikleme

### 🔍 2. Code Quality Workflow
![Code Quality](https://github.com/halilsrky/PayLord/actions/workflows/code-quality.yml/badge.svg)

**Ne yapar:**
- Cppcheck ile statik kod analizi
- Memory leak detection
- Coding standard kontrolü
- Kod istatistikleri (satır sayısı, dosya sayısı)

**Ne zaman:** Her push ve PR

### 🧪 3. Unit Tests Workflow
![Tests](https://github.com/halilsrky/PayLord/actions/workflows/unit-tests.yml/badge.svg)

**Ne yapar:**
- Unit testleri derler
- Testleri çalıştırır
- Test sonuçlarını raporlar
- Test coverage analizi

**Ne zaman:** Her push, PR, veya manuel tetikleme

### 📦 Artifact İndirme

1. [GitHub Actions](https://github.com/halilsrky/PayLord/actions) → Başarılı workflow
2. "Artifacts" bölümünden:
   - `firmware-xxxxx` → Derlenen firmware
   - `cppcheck-report` → Kod analizi
   - `test-results-xxxxx` → Test sonuçları

### 📚 Detaylı Dokümantasyon

**Kapsamlı CI/CD rehberi için:**  
👉 [docs/CI-CD-COMPLETE-GUIDE.md](docs/CI-CD-COMPLETE-GUIDE.md)

Bu rehber içerir:
- CI/CD temel kavramları
- Her workflow'un detaylı açıklaması
- Hızlı başlangıç kılavuzu
- Sorun giderme (troubleshooting)
- Makefile kullanımı
- Best practices
- İleri seviye özellikler

---

## 🧪 Test ve Hata Ayıklama

### Birim Testleri

```c
// Test fonksiyonları
void test_quaternion_operations(void);
void test_kalman_filter(void);
void test_sensor_fusion(void);
void test_packet_protocol(void);
```

### Sistem Testleri

**1. Sensör Kalibrasyon:**
```c
BMI088_CalibrateGyro(&BMI_sensor);
BME280_Calibrate(&BME280_sensor);
```

**2. SD Kart Test:**
```c
test_sd_card_write_speed();
test_fatfs_operations();
```

**3. LoRa İletişim:**
```c
test_lora_ping_pong();
test_lora_packet_loss();
```

### Performans Profiling

**CPU Kullanımı:**
```c
osThreadId_t tasks[10];
uint32_t task_count = uxTaskGetNumberOfTasks();
vTaskList((char*)tasks);
```

**Stack Kullanımı:**
```c
UBaseType_t high_water = uxTaskGetStackHighWaterMark(NULL);
printf("Free stack: %lu bytes\n", high_water * 4);
```

---

## 📚 Dokümantasyon

### Ekstra Kaynaklar

- **[Hızlı Başlangıç Kılavuzu](docs/QUICK-START.md)**: 5 dakikada sistemi çalıştırın
- **[Mimari Dokümantasyon](docs/ARCHITECTURE.md)**: Detaylı sistem mimarisi
- **[CI/CD Kılavuzu](docs/CI-CD-GUIDE.md)**: Otomatik build sistemi

### Datasheet'ler

- [STM32F446RE Datasheet](https://www.st.com/resource/en/datasheet/stm32f446re.pdf)
- [BME280 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf)
- [BMI088 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)
- [FreeRTOS Documentation](https://www.freertos.org/Documentation/RTOS_book.html)

### API Referansları

Her modül için detaylı API dokümantasyonu header dosyalarında mevcuttur:
- [Core/Inc/bme280.h](Core/Inc/bme280.h) - BME280 API
- [Core/Inc/bmi088.h](Core/Inc/bmi088.h) - BMI088 API
- [Core/Inc/sensor_fusion.h](Core/Inc/sensor_fusion.h) - Sensör Füzyon API
- [Core/Inc/flight_algorithm.h](Core/Inc/flight_algorithm.h) - Uçuş Algoritması API

---

## 🤝 Katkıda Bulunma

Katkılarınızı bekliyoruz! Lütfen şu adımları izleyin:

### Katkı Süreci

1. **Fork**: Projeyi fork edin
2. **Branch**: Yeni bir feature branch oluşturun
   ```bash
   git checkout -b feature/amazing-feature
   ```
3. **Commit**: Değişikliklerinizi commit edin
   ```bash
   git commit -m 'feat: Add amazing feature'
   ```
4. **Push**: Branch'inizi push edin
   ```bash
   git push origin feature/amazing-feature
   ```
5. **Pull Request**: PR oluşturun

### Commit Message Format

```
<type>(<scope>): <subject>

<body>

<footer>
```

**Types:**
- `feat`: Yeni özellik
- `fix`: Bug düzeltme
- `docs`: Dokümantasyon
- `style`: Kod formatı
- `refactor`: Kod yeniden yapılandırma
- `test`: Test ekleme
- `chore`: Build/tool değişiklikleri


---

## 👥 İletişim ve Destek

### Geliştirici
**Halil Sarıkaya**
- GitHub: [@halilsrky](https://github.com/halilsrky)
- Proje: [PayLord](https://github.com/halilsrky/PayLord)
- LinkedIn: www.linkedin.com/in/halil-sarıkaya-3a777321b

### Destek Kanalları
- **Issues**: Hata raporları ve özellik istekleri için [GitHub Issues](https://github.com/halilsrky/PayLord/issues)
- **Discussions**: Sorular ve tartışmalar için [GitHub Discussions](https://github.com/halilsrky/PayLord/discussions)

---

## 🎯 Roadmap

### v1.0 (Mevcut)
- ✅ Temel sensör entegrasyonu
- ✅ FreeRTOS task yönetimi
- ✅ LoRa telemetri
- ✅ SD kart veri kaydı
- ✅ Uçuş fazı tespiti

### v1.1 (Planlanan)
- ⏳ Gelişmiş Kalman filtreleme
- ⏳ Manyetometre kalibrasyonu
- ⏳ Pyro kanal kontrolü
- ⏳ Gelişmiş hata kurtarma

### v2.0 (Gelecek)
- 📅 Ground station yazılımı
- 📅 Gerçek zamanlı grafik arayüzü
- 📅 OTA (Over-The-Air) firmware güncelleme
- 📅 Multi-roket koordinasyonu
- 📅 AI tabanlı uçuş optimizasyonu

---

## 🙏 Teşekkürler

Bu proje aşağıdaki açık kaynak projeleri kullanmaktadır:

- **[FreeRTOS](https://www.freertos.org/)** - Real-time operating system
- **[FATFS](http://elm-chan.org/fsw/ff/)** - FAT file system module
- **[SEGGER SystemView](https://www.segger.com/products/development-tools/systemview/)** - Real-time analysis tool
- **[STM32CubeF4](https://www.st.com/en/embedded-software/stm32cubef4.html)** - STM32 HAL library

Ayrıca, bu projeye katkıda bulunan tüm geliştiricilere teşekkür ederiz! 🎉

---

<div align="center">

**⭐ Projeyi beğendiyseniz yıldız vermeyi unutmayın! ⭐**

Made with ❤️ and lots of ☕

[🔝 Başa Dön](#-paylord---skylord2-uçuş-bilgisayarı)

</div>