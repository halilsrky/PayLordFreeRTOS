# 🚀 CI/CD Quick Start Guide

## 5 Dakikada CI/CD'ye Başlayın!

### ✅ Ön Koşullar
- [x] Git kurulu
- [x] GitHub hesabı
- [x] Proje zaten GitHub'da

### 📝 Adımlar

#### 1️⃣ Dosyaları GitHub'a Gönder

Terminali açın ve şu komutları çalıştırın:

```powershell
# Proje dizinine gidin
cd "c:\Users\Halil\STM32CubeIDE\workspace_1.14.1\PayLordFreeRTOS"

# Yeni dosyaları stage'e alın
git add .github/
git add README.md
git add docs/
git add .gitignore

# Commit yapın
git commit -m "ci: GitHub Actions CI/CD pipeline eklendi"

# GitHub'a gönderin
git push origin main
```

#### 2️⃣ GitHub Actions'ı Kontrol Edin

1. Tarayıcıda açın: https://github.com/halilsrky/PayLord
2. **"Actions"** sekmesine tıklayın
3. İlk workflow'un çalıştığını göreceksiniz! 🎉

#### 3️⃣ Build Sonucunu Bekleyin

- 🟡 **Sarı nokta:** Build devam ediyor (2-5 dakika sürer)
- ✅ **Yeşil check:** Build başarılı!
- ❌ **Kırmızı X:** Build başarısız (log'lara bakın)

#### 4️⃣ Firmware'i İndirin

Build başarılı olduysa:

1. Actions sayfasında başarılı workflow'a tıklayın
2. Aşağı kaydırın → **"Artifacts"** bölümünü bulun
3. **`firmware-xxxxxx`** dosyasını indirin
4. ZIP'i açın → içinde `.elf`, `.bin`, `.hex` dosyaları var! 🎊

### 🎯 Artık Ne Olacak?

Her kod değişikliğinde:
```
Kod yaz → Commit → Push → Otomatik build → Firmware hazır!
```

**Hiç bir şey yapmanıza gerek yok!** 🤖

### 📊 Status Badge'i README'ye Ekleyin

README.md dosyanızın en üstüne şunu ekleyin:

```markdown
[![Build Status](https://github.com/halilsrky/PayLord/actions/workflows/build.yml/badge.svg)](https://github.com/halilsrky/PayLord/actions/workflows/build.yml)
```

Bu, GitHub sayfanızda şöyle görünür:
![Build Status Badge](https://img.shields.io/badge/build-passing-brightgreen)

### 🧪 Test Edin

Basit bir değişiklik yapın ve test edin:

```powershell
# Bir dosyayı düzenleyin (örnek)
# Core/Src/main.c'de bir yorum ekleyin

git add Core/Src/main.c
git commit -m "test: CI/CD testi"
git push origin main

# Ardından Actions sekmesine gidin ve build'i izleyin!
```

### 🎉 Tebrikler!

CI/CD pipeline'ınız hazır! Artık profesyonel bir DevOps workflow'unuz var! 🚀

---

### 🆘 Sorun mu Var?

**Build başarısız olursa:**
1. Actions → Failed workflow → Red X'e tıklayın
2. Hata mesajını okuyun
3. Hatayı düzeltin
4. Tekrar push yapın

**Yardıma mı ihtiyacınız var?**
- [CI/CD Rehberi](./CI-CD-GUIDE.md) dökümanını okuyun
- GitHub Issues açın
- Detaylı log'ları inceleyin

### 📚 Sonraki Adımlar

- [ ] Unit test'ler ekleyin
- [ ] Code coverage ölçün
- [ ] Otomatik release oluşturun
- [ ] Slack/Discord bildirimleri ekleyin
- [ ] Hardware-in-the-Loop testing

---

**Not:** Bu proje için oluşturulan CI/CD pipeline tamamen ücretsizdir (GitHub Actions public repo için ücretsiz).
