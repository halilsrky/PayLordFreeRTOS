# 🚀 PayLord FreeRTOS

[![STM32 Build CI](https://github.com/halilsrky/PayLord/actions/workflows/build.yml/badge.svg)](https://github.com/halilsrky/PayLord/actions/workflows/build.yml)
[![Code Quality](https://github.com/halilsrky/PayLord/actions/workflows/code-quality.yml/badge.svg)](https://github.com/halilsrky/PayLord/actions/workflows/code-quality.yml)

STM32F446RET6 tabanlı FreeRTOS projesi - Sensör füzyonu, veri loglama ve telemetri sistemi.

## 📋 Özellikler

- **FreeRTOS** işletim sistemi
- **BMI088** IMU sensörü (Gyro + Accelerometer)
- **BME280** Barometrik basınç ve sıcaklık sensörü
- **L86 GNSS** GPS modülü
- **E22** LoRa kablosuz iletişim
- **FATFS** SD kart dosya sistemi
- **Kalman Filter** sensör füzyonu
- **Quaternion** tabanlı yönelim hesaplama
- **Flight Algorithm** uçuş algoritması

## 🛠️ Geliştirme Ortamı

- **IDE:** STM32CubeIDE 1.14.1
- **MCU:** STM32F446RET6
- **Toolchain:** GNU ARM GCC 13.3.rel1
- **RTOS:** FreeRTOS
- **HAL:** STM32 HAL Driver

## 🔧 Derleme

### STM32CubeIDE ile
1. Projeyi STM32CubeIDE'de açın
2. `Debug` konfigürasyonunu seçin
3. `Build Project` (Ctrl+B)

### Komut Satırından
```bash
cd Debug
make clean
make -j$(nproc)
```

### Binary dosyaları oluşturma
```bash
arm-none-eabi-objcopy -O binary PayLordFreeRTOS.elf PayLordFreeRTOS.bin
arm-none-eabi-objcopy -O ihex PayLordFreeRTOS.elf PayLordFreeRTOS.hex
```

## 🤖 CI/CD Pipeline

Bu proje GitHub Actions kullanarak otomatik derleme ve test süreçlerine sahiptir:

### Build Pipeline
- Her commit'te otomatik derleme
- `.elf`, `.bin`, `.hex` dosyaları artifact olarak saklanır
- Firmware boyutu raporlanır

### Code Quality Pipeline
- Cppcheck ile statik kod analizi
- Kod istatistikleri ve metrikleri

### Workflow Durumu
CI/CD pipeline'ların durumunu [Actions](https://github.com/halilsrky/PayLord/actions) sekmesinden takip edebilirsiniz.

## 📦 Artifact İndirme

Her başarılı build'den sonra firmware dosyalarını Actions sayfasından indirebilirsiniz:
1. [Actions](https://github.com/halilsrky/PayLord/actions) sekmesine gidin
2. Son başarılı workflow'u seçin
3. "Artifacts" bölümünden `firmware-xxx` dosyasını indirin

## 📊 Proje Yapısı

```
PayLordFreeRTOS/
├── Core/
│   ├── Inc/          # Header dosyaları
│   ├── Src/          # Kaynak dosyalar
│   └── Startup/      # Startup assembly
├── Drivers/          # STM32 HAL ve CMSIS
├── FATFS/            # FatFS dosya sistemi
├── Middlewares/      # FreeRTOS ve FatFS
├── Debug/            # Build output
└── .github/
    └── workflows/    # CI/CD pipeline'lar
```

## 🔌 Kullanılan Kütüphaneler

- STM32 HAL Driver
- FreeRTOS v10.x
- FatFS
- SEGGER SystemView (debug)

## 📝 Lisans

Bu proje [MIT License](LICENSE) altında lisanslanmıştır.

## 👨‍💻 Geliştirici

**Halil İbrahim Sarıkaya**  
GitHub: [@halilsrky](https://github.com/halilsrky)

---

⭐ Bu projeyi faydalı bulduysanız yıldız vermeyi unutmayın!
