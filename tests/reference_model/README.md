# Reference Models - DO NOT USE FOR TESTING

## ⚠️ WARNING: Bu kod UÇMAZ!

Bu klasördeki dosyalar **REFERANS MODELLERİ**dir, üretim kodu değildir.

## Amacı

Bu dosyalar şunlar için kullanılabilir:
1. **Golden Model Karşılaştırması**: Production kodunun doğruluğunu bağımsız bir implementasyonla doğrulama
2. **Algoritma Dokümantasyonu**: Karmaşık algoritmaların temiz, okunabilir versiyonları
3. **Regresyon Baseline**: Değişiklikler öncesi/sonrası karşılaştırma için referans

## ⛔ NE İÇİN KULLANILMAMALI

- **Unit Test**: Testler PRODUCTION kodunu (Core/Src/) test etmelidir
- **Validation**: Bu kod uçmuyor, bu yüzden validation değeri yok
- **Qualification**: Safety-critical sistemlerde sadece gerçek kod qualify edilir

## Doğru Test Mimarisi

```
┌─────────────────────┐
│     Unit Tests      │
│  (test_cases/*.c)   │
└──────────┬──────────┘
           │ test
           ▼
┌─────────────────────┐
│   Production Code   │  ◄── BU KOD UÇAR!
│   (Core/Src/*.c)    │
└─────────────────────┘
           │
           │ compare (optional)
           ▼
┌─────────────────────┐
│  Reference Models   │  ◄── Bu kod UÇMAZ
│ (reference_model/)  │
└─────────────────────┘
```

## İlgili Standartlar

- DO-178C: "Software accomplishment" için gerçek kod test edilmeli
- IEC 61508: Test edilmiş kod = deploy edilen kod
- MISRA: Test coverage gerçek kaynak kodla ölçülmeli

---
**"Test edilen kod uçmalı. Uçacak kod test edilmeli."**
