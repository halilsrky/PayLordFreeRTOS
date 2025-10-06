# CI/CD Kurulumu - Adım Adım

## 1. Git Durumunu Kontrol Et
git status

## 2. Yeni Dosyaları Ekle
git add .github/
git add README.md
git add .gitignore
git add docs/

## 3. Commit Yap
git commit -m "ci: GitHub Actions CI/CD pipeline eklendi

- STM32 otomatik build pipeline
- Kod kalitesi kontrolleri (cppcheck)
- Artifact yönetimi (.elf, .bin, .hex)
- Detaylı dokümantasyon
- CI/CD öğrenme rehberleri"

## 4. GitHub'a Gönder
git push origin main

## 5. GitHub Actions'ı Kontrol Et
# Tarayıcınızda açın:
# https://github.com/halilsrky/PayLord/actions

## 6. İlk Build'i İzleyin
# Actions sekmesinde çalışan workflow'u göreceksiniz!
# 2-5 dakika içinde tamamlanır.

## 7. Artifact'leri İndirin (Build başarılı olduktan sonra)
# Actions → Son workflow → Artifacts → firmware-xxxxx

## Tebrikler! 🎉
# CI/CD pipeline'ınız hazır ve çalışıyor!
