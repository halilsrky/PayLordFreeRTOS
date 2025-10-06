# 🎓 CI/CD Öğrenme Rehberi

## 📚 CI/CD Nedir?

### Continuous Integration (CI) - Sürekli Entegrasyon
**Amaç:** Kod değişikliklerini otomatik olarak test etmek ve derlemek.

**Nasıl Çalışır?**
1. Developer kod yazar ve Git'e push eder
2. Otomatik olarak bir sunucu kodu alır
3. Projeyi derler (build)
4. Testleri çalıştırır
5. Sonucu bildirir (✅ başarılı / ❌ hata)

**Faydaları:**
- Hataları erken yakalar
- Manuel derleme gerektirmez
- Takım üyeleri arasında koordinasyon sağlar
- Her değişiklik test edilir

### Continuous Deployment (CD) - Sürekli Dağıtım
**Amaç:** Test edilen kodu otomatik olarak yayınlamak.

**Nasıl Çalışır?**
1. CI testleri başarılı olur
2. Otomatik olarak artifact'ler oluşturulur
3. (Opsiyonel) Production'a deploy edilir

## 🔧 Bu Projede Ne Yaptık?

### 1. GitHub Actions Workflow'ları

#### `build.yml` - Ana Build Pipeline
```yaml
on:
  push:
    branches: [ main, develop ]
  pull_request:
    branches: [ main, develop ]
```

**Ne zaman çalışır?**
- `main` veya `develop` branch'e kod push edilince
- Pull request açılınca
- Manuel olarak tetiklenince

**Ne yapar?**
1. ✅ Kodu GitHub'dan çeker
2. ✅ ARM GCC derleyiciyi kurar
3. ✅ Projeyi derler (`make`)
4. ✅ Binary dosyaları oluşturur (`.bin`, `.hex`)
5. ✅ Firmware dosyalarını artifact olarak saklar
6. ✅ Firmware boyutunu raporlar

#### `code-quality.yml` - Kod Kalitesi Pipeline
**Ne yapar?**
1. ✅ Cppcheck ile statik kod analizi
2. ✅ Kod istatistikleri (satır sayısı, dosya sayısı)
3. ✅ Analiz raporları oluşturur

### 2. Dosya Yapısı

```
.github/
└── workflows/
    ├── build.yml           # Ana derleme pipeline'ı
    └── code-quality.yml    # Kod kalitesi kontrolü
```

## 🚀 Kullanım Kılavuzu

### Adım 1: GitHub'a Push Yapın
```bash
git add .
git commit -m "feat: yeni özellik eklendi"
git push origin main
```

### Adım 2: Actions Sekmesini Kontrol Edin
1. GitHub repository'nize gidin
2. "Actions" sekmesine tıklayın
3. Çalışan workflow'ları görün

### Adım 3: Build Sonuçlarını İnceleyin
- ✅ Yeşil check: Build başarılı
- ❌ Kırmızı X: Build başarısız
- 🟡 Sarı nokta: Build devam ediyor

### Adım 4: Artifact'leri İndirin
1. Başarılı bir workflow'a tıklayın
2. Aşağıdaki "Artifacts" bölümünü bulun
3. `firmware-xxx` dosyasını indirin
4. ZIP'i açın - içinde `.elf`, `.bin`, `.hex` dosyaları var!

## 🎯 CI/CD Best Practices

### ✅ Yapılması Gerekenler
1. **Küçük commitler:** Her commit bir özellik/düzeltme içermeli
2. **Anlamlı commit mesajları:** `feat:`, `fix:`, `docs:` gibi önekler kullanın
3. **Branch stratejisi:** `main` (stabil), `develop` (geliştirme)
4. **Pull Request:** Direkt main'e push etmeyin, PR kullanın
5. **Code review:** Başka birinin kodunuzu incelemesini sağlayın

### ❌ Yapılmaması Gerekenler
1. **Broken code push:** Test etmeden push yapmayın
2. **Büyük değişiklikler:** Bir commit'te çok fazla değişiklik yapmayın
3. **Build artifacts commit:** `.elf`, `.o` dosyalarını commit etmeyin
4. **Secrets:** API key'leri kodda saklamayın

## 📊 Workflow Detayları

### Build Pipeline Adımları

```
1. Checkout Repository
   ↓
2. Install ARM GCC Toolchain
   ↓
3. Check Versions
   ↓
4. Build Project (make)
   ↓
5. Create Binary Files
   ↓
6. Upload Artifacts
   ↓
7. Display Build Info
```

### Build Başarısız Olursa Ne Yapmalı?

1. **Log'ları inceleyin:**
   - Actions → Failed workflow → Hatalı step'e tıklayın
   - Hata mesajını okuyun

2. **Yaygın hatalar:**
   - Syntax error (kod hatası)
   - Missing include (eksik header)
   - Undefined reference (eksik fonksiyon)

3. **Düzeltme:**
   - Hatayı düzeltin
   - Tekrar commit & push yapın
   - CI otomatik olarak tekrar çalışır

## 🔍 Code Quality Checks

### Cppcheck Nedir?
Statik kod analiz aracı - çalıştırmadan kod hatalarını bulur.

**Ne tür hatalar bulur?**
- Memory leak'ler
- Null pointer dereference
- Array bounds overflow
- Kullanılmayan değişkenler
- Potansiyel bug'lar

## 🎓 İleri Seviye Konular

### 1. Otomatik Unit Testing
```yaml
- name: Run Unit Tests
  run: |
    cd tests
    ./run_tests.sh
```

### 2. Hardware-in-the-Loop (HIL) Testing
Gerçek STM32 board'a otomatik firmware yükleme.

### 3. Otomatik Versioning
Her build'de otomatik versiyon numarası artırma.

### 4. Release Automation
Tag push edilince otomatik GitHub Release oluşturma.

### 5. Notification
Slack/Discord'a build sonucu bildirimi gönderme.

## 📚 Öğrenme Kaynakları

### GitHub Actions
- [GitHub Actions Documentation](https://docs.github.com/en/actions)
- [Workflow Syntax](https://docs.github.com/en/actions/using-workflows/workflow-syntax-for-github-actions)

### CI/CD Genel
- [Martin Fowler - CI](https://martinfowler.com/articles/continuousIntegration.html)
- [Atlassian CI/CD Tutorial](https://www.atlassian.com/continuous-delivery/principles/continuous-integration-vs-delivery-vs-deployment)

### Embedded CI/CD
- [Embedded Artistry - CI/CD](https://embeddedartistry.com/blog/2017/12/21/jenkins-as-a-continuous-integration-server-for-embedded-projects/)

## 🎉 Sonuç

Artık projeniz CI/CD pipeline'ına sahip! Her kod değişikliğinde:
- ✅ Otomatik derleme
- ✅ Hata kontrolü
- ✅ Kod kalitesi analizi
- ✅ Artifact oluşturma

**Tebrikler!** 🎊 DevOps dünyasına hoş geldiniz!

## ❓ SSS

**S: CI/CD kullanmak zorunlu mu?**
C: Hayır, ama profesyonel projelerde standart.

**S: Build süreleri uzun mu?**
C: Genelde 2-5 dakika. Paralel build ile daha hızlı.

**S: Ücretsiz mi?**
C: GitHub Actions public repo'lar için ücretsiz. Private için 2000 dakika/ay ücretsiz.

**S: Başka CI/CD araçları var mı?**
C: Evet - Jenkins, GitLab CI, CircleCI, Travis CI vs.

---

**Hazırlayan:** GitHub Copilot 🤖  
**Tarih:** Ekim 2025
