#!/bin/bash
# STM32 Build Script for CI/CD
# Bu script CubeIDE Makefile yerine direkt GCC komutları kullanır

set -e  # Hata olursa dur

echo "🔧 STM32F446 Build Script"
echo "=========================="

# Değişkenler
PROJECT_ROOT="$(pwd)"
DEBUG_DIR="$PROJECT_ROOT/Debug"
OUTPUT_NAME="PayLordFreeRTOS"

# Compiler ayarları
CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
SIZE=arm-none-eabi-size

# MCU özellikleri
MCU="-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard"

# Compiler flags
CFLAGS="$MCU -DUSE_HAL_DRIVER -DSTM32F446xx"
CFLAGS="$CFLAGS -ICore/Inc"
CFLAGS="$CFLAGS -IDrivers/STM32F4xx_HAL_Driver/Inc"
CFLAGS="$CFLAGS -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy"
CFLAGS="$CFLAGS -IDrivers/CMSIS/Device/ST/STM32F4xx/Include"
CFLAGS="$CFLAGS -IDrivers/CMSIS/Include"
CFLAGS="$CFLAGS -IMiddlewares/Third_Party/FreeRTOS/Source/include"
CFLAGS="$CFLAGS -IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2"
CFLAGS="$CFLAGS -IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F"
CFLAGS="$CFLAGS -IFATFS/Target -IFATFS/App"
CFLAGS="$CFLAGS -IMiddlewares/Third_Party/FatFs/src"
CFLAGS="$CFLAGS -O0 -g3 -Wall -fdata-sections -ffunction-sections"

# Linker flags
LDFLAGS="$MCU -T$PROJECT_ROOT/STM32F446RETX_FLASH.ld"
LDFLAGS="$LDFLAGS --specs=nosys.specs -Wl,-Map=$DEBUG_DIR/$OUTPUT_NAME.map"
LDFLAGS="$LDFLAGS -Wl,--gc-sections -static --specs=nano.specs"
LDFLAGS="$LDFLAGS -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group"

echo "🏗️  CubeIDE Makefile ile derleme deneniyor..."

# Debug klasörü var mı kontrol et
if [ ! -d "$DEBUG_DIR" ]; then
    echo "❌ Debug klasörü bulunamadı!"
    echo "💡 STM32CubeIDE'de projeyi build edin (en az bir kez)"
    exit 1
fi

cd "$DEBUG_DIR"

# Makefile varsa ve düzeltilebilirse onu kullan
if [ -f "makefile" ]; then
    echo "✅ Makefile bulundu, path'ler düzeltiliyor..."
    
    # Backup al
    cp makefile makefile.original
    
    # Windows path'lerini düzelt
    sed -i "s|C:\\\\Users\\\\Halil\\\\STM32CubeIDE\\\\workspace_1.14.1\\\\PayLordFreeRTOS\\\\STM32F446RETX_FLASH\\.ld|../STM32F446RETX_FLASH.ld|g" makefile
    sed -i "s|C:\\\\Users\\\\Halil\\\\STM32CubeIDE\\\\workspace_1.14.1\\\\PayLordFreeRTOS|..|g" makefile
    
    # Makefile syntax kontrolü
    if make -n clean > /dev/null 2>&1; then
        echo "✅ Makefile syntax OK, derleme başlıyor..."
        make clean
        make -j$(nproc)
        
        if [ -f "$OUTPUT_NAME.elf" ]; then
            echo "✅ Build başarılı (Makefile ile)!"
            
            # Binary dosyalarını oluştur
            echo "📦 Binary dosyaları oluşturuluyor..."
            $OBJCOPY -O binary "$OUTPUT_NAME.elf" "$OUTPUT_NAME.bin"
            $OBJCOPY -O ihex "$OUTPUT_NAME.elf" "$OUTPUT_NAME.hex"
            
            # Boyut bilgisi
            echo "📊 Firmware boyutu:"
            $SIZE "$OUTPUT_NAME.elf"
            
            exit 0
        fi
    else
        echo "⚠️  Makefile syntax hatası, alternatif yöntem kullanılıyor..."
    fi
else
    echo "⚠️  Makefile bulunamadı, alternatif yöntem kullanılıyor..."
fi

# Eğer Makefile çalışmazsa, bu mesaj gösterilir
echo ""
echo "❌ CubeIDE Makefile ile derleme başarısız oldu."
echo "💡 Çözüm: CubeIDE'de projeyi yeniden generate edin veya CMake kullanın."
echo ""
echo "📚 Detaylar için: docs/TROUBLESHOOTING.md"
exit 1
