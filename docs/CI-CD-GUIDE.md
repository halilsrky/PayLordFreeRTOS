# 🚀 SKYRTOS CI/CD Kılavuzu

**Her push'ta otomatik build, test ve kod analizi.**

## 📌 İçindekiler

1. [Workflow'lar](#-projedeki-workflow-lar)
2. [Hızlı Başlangıç](#-hızlı-başlangıç)
3. [Sorun Giderme](#-sorun-giderme)
4. [Best Practices](#-best-practices)

---

## 📦 Projedeki Workflow'lar

3 adet GitHub Actions workflow:

### 1️⃣ Build Workflow (`build.yml`)
- STM32 firmware'ini derler
- `.elf`, `.bin`, `.hex` oluşturur
- Artifact: `firmware-<sha>`
- **Ne zaman:** main/develop branch'e push, PR

### 2️⃣ Code Quality (`code-quality.yml`)
- Cppcheck ile statik analiz
- Kod istatistikleri
- Artifact: `cppcheck-report`
- **Ne zaman:** main/develop push, PR

### 3️⃣ Unit Tests (`unit-tests.yml`)
- Unit testleri çalıştırır
- Test sonuçlarını raporlar
- Artifact: `test-results-<sha>`
- **Ne zaman:** main/develop push, PR

## 🚀 Hızlı Başlangıç

### 1. Kod Yaz ve Push Et

```bash
git add .
git commit -m "feat: yeni özellik"
git push origin main
```

### 2. GitHub Actions'ı İzle

https://github.com/halilsrky/SKYRTOS/actions

**Çıktısı:**
```
✅ STM32 Build CI         (3-5 dk)
✅ Code Quality           (1-2 dk)
✅ Unit Tests             (2-3 dk)
```

### 3. Artifact'leri İndir

Başarılı workflow → "Artifacts" → İndir:
- `firmware-xxxxx` → Derlenen firmware
- `cppcheck-report` → Kod analizi
- `test-results-xxxxx` → Test sonuçları

---

## � Sorun Giderme

| Hata | Sebep | Çözüm |
|------|-------|-------|
| Build başarısız | Syntax hatası | Log'a bakın, kodu düzeltin |
| `make: No rule to make target` | Yeni dosya Makefile'a eklenmemiş | `C_SOURCES`'ta ekleyin |
| `undefined reference to 'func'` | Eksik `.c` dosyası | Dosyayı `C_SOURCES`'a ekleyin |
| Cppcheck uyarısı | Kullanılmayan değişken | Değişkeni kaldırın veya kullanın |
| Test fail | Assert hatası | Test dosyasını lokal çalıştırın, kodu düzeltin |

### Hızlı Kontrol

```bash
# Lokal olarak test et
make clean && make -j4

# Testleri çalıştır
cd tests && make test

# Kod analizi
cppcheck --enable=all Core/Src/
```

---

## 🎯 Best Practices

### ✅ Yapılması Gerekenler
- Küçük, anlamlı commit'ler yaz
- `feat:`, `fix:`, `test:` prefiksi kullan
- PR ile geliştir (main'e direkt push yapma)
- Lokal test et → Push et → CI/CD otomatik çalışır

### ❌ Yapılmaması Gerekenler
- Build artifact'lerini commit etme (`.o`, `.elf`, `.bin`)
- Broken code push etme
- Büyük değişiklikler tek commit'te
- Secret'ları kod içine koyma

---

## 📝 Makefile'a Dosya Ekleme

**C dosyası eklemek:**
```makefile
C_SOURCES = \
Core/Src/main.c \
Core/Src/yeni_dosya.c \    # ← YENİ
...
```

**Include dizini eklemek:**
```makefile
C_INCLUDES = \
-ICore/Inc \
-IYeni/Dizin \             # ← YENİ
...
```

Sonra:
```bash
make clean
make -j$(nproc)
```

---

## 📊 Workflow Özeti

| Workflow | Görev | Süre | Artifact |
|----------|-------|------|----------|
| Build | Firmware derle | 3-5 dk | `.elf`, `.bin`, `.hex` |
| Code Quality | Statik analiz | 1-2 dk | Cppcheck raporu |
| Unit Tests | Testleri çalıştır | 2-3 dk | Test sonuçları |

**Merge kuralı:** Tüm 3 workflow ✅ olmalı
