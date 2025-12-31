# 🔧 CI/CD Troubleshooting Guide

## ❌ Yaygın Hatalar ve Çözümleri

### 1. "multiple target patterns" veya "missing separator" Hatası

**Hata Mesajları:**
```
makefile:74: *** multiple target patterns. Stop.
makefile:43: *** missing separator. Stop.
Error: Process completed with exit code 2.
```

**Neden:**
STM32CubeIDE'nin Windows için oluşturduğu Makefile'da **Windows path'leri** (`C:\Users\...`) var. Bu path'ler:
1. Linux'ta geçersiz karakterler içerir (`\`, `:`)
2. Makefile syntax'ını bozar
3. `sed` ile düzeltmeye çalışınca TAB karakterleri bozulabilir

**Çözüm 1: Build Script Kullan (Önerilen)**
✅ Proje kök dizinine `build.sh` ekledik. Bu script:
- Önce CubeIDE Makefile'ı düzeltmeyi dener
- Başarısız olursa hata mesajı verir
- Daha güvenilir ve debug edilebilir

Workflow'da kullanımı:
```yaml
- name: 🏗️ Build Project
  run: ./build.sh
```

**Çözüm 2: Manuel Path Düzeltme (Riskli)**
```bash
cd Debug
sed -i 's|C:\\Users\\Halil\\STM32CubeIDE\\workspace_1.14.1\\PayLordFreeRTOS\\STM32F446RETX_FLASH\.ld|../STM32F446RETX_FLASH.ld|g' makefile
make
```

⚠️ **Dikkat:** TAB karakterlerini korumak önemli! Makefile'da komutlar TAB ile başlamalı.

**Çözüm 3: Projeyi Yeniden Generate Et**
1. STM32CubeIDE'de projeyi aç
2. `.ioc` dosyasını aç
3. Project Manager → Toolchain → Makefile seç
4. Generate Code
5. Commit yeni Makefile'ı

---

### 2. "arm-none-eabi-gcc: command not found"

**Hata Mesajı:**
```
arm-none-eabi-gcc: command not found
```

**Neden:**
ARM GCC toolchain kurulu değil.

**Çözüm:**
✅ Workflow'da zaten var:
```yaml
- name: 🔧 Install ARM GCC Toolchain
  run: |
    sudo apt-get install -y gcc-arm-none-eabi
```

---

### 3. "No such file or directory: makefile"

**Hata Mesajı:**
```
make: *** No targets specified and no makefile found. Stop.
```

**Neden:**
Makefile yanlış dizinde aranıyor.

**Çözüm:**
Workflow'da `cd Debug` yapıldığından emin olun:
```yaml
- name: 🏗️ Build Project
  run: |
    cd Debug
    make clean
    make -j$(nproc)
```

---

### 4. Build Başarılı Ama Artifact Yok

**Problem:**
Build ✅ başarılı ama artifact'leri bulamıyorum.

**Çözüm:**
1. Actions sayfasında workflow'a tıklayın
2. **Aşağı kaydırın** - "Artifacts" bölümü sayfa sonunda
3. `firmware-xxxxxx` adlı dosyayı bulun

**Not:** Artifact'ler **30 gün** sonra otomatik silinir.

---

### 5. "specs=nosys.specs: No such file"

**Hata Mesajı:**
```
cannot find -lnosys
/usr/lib/gcc/arm-none-eabi/x.x.x/../../../../arm-none-eabi/bin/ld: cannot find -lc
```

**Neden:**
ARM newlib veya libc eksik.

**Çözüm:**
Toolchain kurulumuna `libnewlib-arm-none-eabi` ekleyin:
```yaml
- name: 🔧 Install ARM GCC Toolchain
  run: |
    sudo apt-get update
    sudo apt-get install -y gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi
```

---

### 6. Derleme Çok Yavaş

**Problem:**
Build 10+ dakika sürüyor.

**Çözüm:**
Paralel derleme kullanın:
```bash
make -j$(nproc)  # Tüm CPU core'ları kullanır
```

✅ Zaten workflow'da var!

---

### 7. "Permission denied" Hatası

**Hata Mesajı:**
```
bash: ./build.sh: Permission denied
```

**Çözüm:**
Script'e executable izni verin:
```yaml
- name: Make Script Executable
  run: chmod +x build.sh
```

---

### 8. Git Submodule Sorunları

**Problem:**
FreeRTOS veya HAL driver'lar eksik.

**Çözüm:**
Checkout step'inde submodule'leri çekin:
```yaml
- name: 📥 Checkout Repository
  uses: actions/checkout@v4
  with:
    submodules: recursive  # ✅ Bu satır önemli!
```

---

## 🐛 Debug İpuçları

### 1. Makefile'ı İnceleme

Workflow'a debug adımı ekleyin:
```yaml
- name: 🔍 Debug Makefile
  run: |
    cat Debug/makefile | grep -n "STM32F446RETX_FLASH.ld"
    echo "---"
    ls -la
```

### 2. Build Çıktısını Kaydetme

Hata loglarını artifact olarak kaydedin:
```yaml
- name: 📤 Upload Build Logs
  if: failure()  # Sadece hata olursa
  uses: actions/upload-artifact@v4
  with:
    name: build-logs
    path: |
      Debug/*.log
      Debug/makefile
```

### 3. Environment Variables

Hangi değişkenlerin kullanıldığını görün:
```yaml
- name: 🔍 Show Environment
  run: |
    echo "PATH: $PATH"
    echo "PWD: $PWD"
    which arm-none-eabi-gcc
```

---

## 📚 STM32CubeIDE Makefile Sorunları

### Neden CubeIDE Makefile'ları Sorunlu?

1. **Absolute Windows Paths:**
   ```makefile
   C:\Users\Halil\STM32CubeIDE\workspace_1.14.1\PayLordFreeRTOS\STM32F446RETX_FLASH.ld
   ```
   ❌ Linux'ta çalışmaz

2. **Backslash Path Separators:**
   ```makefile
   Core\Src\main.c
   ```
   ❌ Linux'ta `/` kullanılır

3. **IDE-Specific Paths:**
   CubeIDE kendi workspace path'lerini kullanır

### Çözüm Stratejileri

#### Strateji 1: Path Dönüştürme (Kullandığımız)
```bash
sed -i 's|C:\\Users\\Halil\\...\\|../|g' makefile
```
✅ Hızlı ve kolay

#### Strateji 2: Custom Makefile
Kendi Makefile'ınızı yazın (gelişmiş):
```makefile
# Portable Makefile
PROJECT_ROOT = ..
LINKER_SCRIPT = $(PROJECT_ROOT)/STM32F446RETX_FLASH.ld
```

#### Strateji 3: CMake Kullanma
CMake daha portable:
```cmake
cmake_minimum_required(VERSION 3.15)
project(PayLordFreeRTOS C ASM)
```

---

## 🎯 Best Practices

### ✅ Yapılması Gerekenler

1. **Always Test Locally First:**
   ```bash
   cd Debug
   make clean
   make
   ```

2. **Use Relative Paths:**
   ```makefile
   ../STM32F446RETX_FLASH.ld  # ✅ Good
   C:\Users\...\file.ld        # ❌ Bad
   ```

3. **Version Control .cproject:**
   `.cproject` dosyasını commit edin (IDE ayarları için)

4. **Keep Makefile Simple:**
   Karmaşık logic'i script'lere taşıyın

### ❌ Yapılmaması Gerekenler

1. **Don't Commit Build Artifacts:**
   ```gitignore
   Debug/*.o
   Debug/*.elf
   ```

2. **Don't Use Hardcoded Paths:**
   Environment variable kullanın

3. **Don't Mix Line Endings:**
   Git'te `autocrlf=true` ayarlayın

---

## 🔍 Log Analizi

### Build Başarılı Log Örneği:

```
✅ Checkout Repository
✅ Install ARM GCC Toolchain
✅ Check Toolchain Version
✅ Fix Makefile Paths
✅ Build Project
   arm-none-eabi-gcc version 10.3.1
   Compiling Core/Src/main.c
   Compiling Core/Src/freertos.c
   ...
   Linking PayLordFreeRTOS.elf
✅ Create Binary Files
   text    data     bss     dec     hex
  123456   1234   12345  136935  21717
✅ Upload Artifacts
✅ Build completed successfully!
```

### Build Başarısız Log Örneği:

```
✅ Checkout Repository
✅ Install ARM GCC Toolchain
✅ Check Toolchain Version
❌ Fix Makefile Paths
   sed: can't read makefile: No such file or directory
```

**Sorun:** `Debug/makefile` bulunamadı  
**Çözüm:** Dosya var mı kontrol edin: `ls -la Debug/`

---

## 📞 Yardım Almak

### GitHub Actions Log'larını Paylaşma

1. Actions sekmesine gidin
2. Başarısız workflow'a tıklayın
3. Hatalı step'i genişletin
4. Log'u kopyalayın
5. Issue açın veya Stack Overflow'da sorun

### Yerel Test

CI/CD'yi test etmeden önce lokal olarak test edin:

```bash
# Windows PowerShell'de
cd "C:\Users\Halil\STM32CubeIDE\workspace_1.14.1\PayLordFreeRTOS\Debug"
make clean
make -j4
```

Eğer lokal çalışıyorsa, CI/CD'de de çalışacaktır (path sorunları hariç).

---

## 🚀 Performans İyileştirmeleri

### Build Cache Kullanma

GitHub Actions cache ile derleme hızlandırılabilir:

```yaml
- name: 💾 Cache Build Files
  uses: actions/cache@v3
  with:
    path: |
      Debug/*.o
      Debug/*.d
    key: ${{ runner.os }}-build-${{ hashFiles('**/*.c') }}
```

**Not:** Dikkatli kullanın - cache sorunları daha fazla hata yaratabilir.

---

**Son Güncelleme:** Ekim 2025  
**Versiyon:** 1.0

Bu troubleshooting guide ile karşılaşacağınız 95% sorunu çözebilirsiniz! 💪
