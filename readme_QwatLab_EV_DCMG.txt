QwatLab - EV D.C.M.G. (Driving Cycle Markov Generator) V1.1.4
Please read the license and usage requirements before using the software.
Dasturdan foydalanishdan avval litsenziya va dasturdan foydalanish talablarini o'qib chiqing.

🇺🇿 Oʻzbek tilidagi tavsif
QwatLab - EV D.C.M.G. V1.0.0  — real empirik safar ma'lumotlari (EGAD V1.1.4 55/29/8-ustunli fayllari, CSV, XLSX) asosida elektromobillar (EV) uchun statistik jihatdan reprezentativ, tartibdan mustaqil (Order-Invariant) va kinematik yopiq standart harakat sikllarini (Driving Cycle) sintez qiluvchi Python dasturiy platformasi.
V1.1.4 yuklandi, avvalgi hisoblashdagi xatoliklar tahrirlandi

Dasturiy platforma tavsifi
Ushbu platforma ko'p safarli real telemetriya bazasini SHA-256 kontent-xesh orqali kanonizatsiya qilib, yuklash tartibidan qat'i nazar 100% takrorlanuvchi (reproducible) sintez natijasini kafolatlaydi. Dastur gibrid Quasi-Micro-Trip, Silhouette-optimal K-Means klasterlash va 2D SAFD (Speed-Acceleration Frequency Distribution) Markov-Monte-Karlo algoritmlari asosida ishlaydi. V1.1.4 versiyasida barcha matematik dvigatellar vizual interfeysdan (GUI) 100% ajratilgan bo'lib, mahalliy HTTP JSON API orqali ham mustaqil (`--app dcmg`), ham yagona 4-dasturli **QwatLab EV Suite** (`EGAD → DCMG → MADE → Article Plotter`) tarkibida to'liq oflayn ishlaydi.

Asosiy hisoblash va tahlil yo‘nalishlari
* Ma'lumotlarni o'qish va tartibdan mustaqil kanonizatsiya: EGAD V1.1.4 (55/29/8-ustunli va 6-sahifali Excel) hamda erkin CSV/XLSX fayllarini avtomatik ustun aniqlash (auto-detect) orqali o'qish, SHA-256 asosida tartibdan mustaqil (max |Δ| = 0.000000) birlashtirish.
* Quasi-Micro-Trip ajratish va K-Means klasterlash: Harakatlanuvchi ekskursiyalar va to'xtash (idle) darchalarini yo'qotishsiz ajratish, uzoq ekskursiyalarni lokal minimumlarda bo'lish va Silhouette ko'rsatkichi bo'yicha optimal klasterlar sonini (K_c = 2..5) avtomatik tanlash.
* Simmetrik 2D SAFD matritsasi va Markov-Monte-Karlo sintezi: a = 0.000 m/s² markazida simmetrik joylashgan adaptiv 2D (v, a) SAFD panjarasi, Numba-tezlashtirilgan Markov o'tish matritsalari va kompozit maqsad funksiyasi (kinematik nisbiy xato + SAFD Bhattacharyya o'xshashligi) bo'yicha eng optimal siklni tanlash.
* 100% Yopiq Kinematik Zanjir (s ↔ v ↔ a ↔ j): Qat'iy nol-tezlik qulfi (v_k = 0 bo'lganda Δs_k = 0 va to'xtashda a_k = 0), kauzal tezlanish va siltanish (jerk) hosilalari, shuningdek, GPS tezligidan fizik sirpanish modeli κ(a) orqali OBD tezligini sintez qilish.
* Topografiya, Shamol va BMS 2RC PSD Validatsiyasi: Masofa bo'yicha vaznlangan yo'l qiyaligi θ(s) va shamol tezligi v_wind(t) momentlarini saqlash (to'xtashda θ o'zgarmas), VSP/EV energiya hisobi, Welch BMS quvvat spektral zichligi (PSD) mosligi, Kolmogorov-Smirnov (KS) testi va har bir safarning reprezentativlik reytingi.
* Vizualizatsiya va Ko'p Formatli Eksport: Empirik va sintez qilingan tezlik profili hamda taqsimotining diagnostik grafiklari, 8-ustunli MADE-tayyor CSV, 55-ustunli EGAD-mos CSV, validatsiya va reyting jadvallari, rangli Master XLSX hamda yagona ZIP arxiv eksporti.

Dasturiy arxitekturaning rivojlanishi va modullashtirish rejasi
1-bosqich. Modullarga bo'lish va GUI'ni yadrodan to'liq ajratish (0% GUI bog'liqlik):
QwatLab - EV D.C.M.G. V1.1.4 "Separation of Concerns" tamoyili asosida hech qanday GUI kutubxonasiga (Streamlit/Tkinter/PyQt/Flet) bog'lanmagan 4 ta toza backend moduli va tashqi plagin konfiguratsiyasiga ajratilgan:
1. modules/dcmg/config_bounds.py — Fizik chegaralar, avtomobil toifalari (VEHICLE_OPERATION_PRESETS), 8-ustunli MADE va 55-ustunli EGAD kontraktlari hamda plugins/dcmg/custom_presets.json yuklovchisi.
2. modules/dcmg/data_loader.py — Tabiiy raqamli saralash, CSV/XLSX fayllarni keshlangan o'qish, ustunlarni avtomatik aniqlash, validatsiya va demo sikl generatori.
3. modules/dcmg/core_engine.py — SHA-256 kanonizatsiya, Numba-tezlashtirilgan 2D SAFD, K-Means/Silhouette klasterlash, Markov-Microtrip sintezi, yopiq kinematik zanjir, VSP/BMS PSD validatsiya va rangli OpenPyXL eksport dvigateli.
4. modules/dcmg/adapter.py — Asinxron JobContext (0% → 100% progress va bekor qilish), ResultStore xotira shinasi integratsiyasi va Master ZIP yig'uvchi adapter.

2-bosqich. Mahalliy HTTP JSON API va 100% Oflayn Desktop muhiti:
core_shared/ (api_server.py, job_manager.py, result_store.py, module_loader.py, contracts.py) hamda 100% oflayn ui/ (index.html, styles.css, app.js — tashqi CDN'siz) qatlami orqali dastur Python o'rnatilmagan va internetsiz Windows kompyuterlarda .EXE formatida jonli progress indikatori bilan ishlaydi.

3-bosqich. Qayta yig'masdan yangilanish (Hot-Upgrade) va 4-dasturli Suite Integratsiyasi:
modules/dcmg/ va plugins/dcmg/ papkalari .EXE faylidan tashqarida saqlanadi, shuning uchun v1.1.5–v1.1.9 yangilanishlarida faqat kichik modul papkasini almashtirish kifoya. ResultStore xotira shinasi orqali DCMG bevosita 1-dasturdan (EGAD V1.1.4) 55-ustunli ma'lumotni qabul qilib, sintez qilingan 8-ustunli siklni diskka qayta yuklamasdan 3-dasturga (MADE V1.1.4) uzatadi.

V1.1.4 versiyasining ilmiy o‘rni
Ushbu platforma real shahar va magistral EV harakat ma'lumotlaridan Q1 ilmiy jurnallar talablariga to'liq javob beradigan, takrorlanuvchi va energiya-ekvivalent mahalliy harakat sikllarini (masalan, Toshkent harakat sikli — TDC) yaratish uchun ochiq fan (Open Science) vositasi sifatida xizmat qiladi. Tartibdan mustaqil SHA-256 sintez va BMS PSD validatsiyasi natijalarning qat'iy ilmiy ishonchliligini ta'minlaydi.

🇬🇧 English Description
Software Platform Overview
QwatLab - EV D.C.M.G. (Driving Cycle Markov Generator) V1.0.0 is a Python-based open-science platform designed to synthesize statistically representative, order-invariant, and kinematically closed electric vehicle (EV) driving cycles from empirical multi-trip telemetry (EGAD V1.1.4 55/29/8-column datasets, CSV, and XLSX). By canonicalizing empirical fleets via SHA-256 content fingerprints, the engine guarantees 100% reproducible synthesis regardless of file upload order. Powered by Hybrid Quasi-Micro-Trip extraction, Silhouette-optimal K-Means clustering, and symmetric 2D SAFD Markov-Monte-Carlo optimization, V1.1.4 operates with 0% GUI coupling in its mathematical core and runs 100% offline either standalone (`--app dcmg`) or inside the unified 4-program **QwatLab EV Suite** (`EGAD → DCMG → MADE → Article Plotter`).
Uploaded V1.1.4, corrected errors in previous calculations

Core Computational and Analytical Features
* Multi-Trip Parsing & Order-Invariant SHA-256 Canonicalization: Automatic column detection across EGAD V1.1.4 (55/29/8-col & 6-sheet Excel) and arbitrary CSV/XLSX files, with content-hash canonical sorting guaranteeing identical synthesis across any file permutation (max |Δ| = 0.000000).
* Quasi-Micro-Trip Extraction & K-Means Clustering: Lossless separation of moving excursions and idle/stop pools, local-minima splitting of overly long excursions, and automated Silhouette-optimal regime clustering (K_c = 2..5) with empirical regime transition matrices.
* Symmetric Zero-Centered 2D SAFD & Markov-Monte-Carlo Optimization: Adaptive 2D Speed-Acceleration Frequency Distribution (SAFD) grid strictly centered at a = 0.000 m/s², Numba-accelerated Markov transition CDFs, and composite objective optimization balancing kinematic relative error and SAFD Bhattacharyya similarity.
* 100% Closed Kinematic Chain (s ↔ v ↔ a ↔ j): Strict zero-speed standstill lock (Δs_k = 0 and a_k = 0 when v_k = 0), causal step acceleration and jerk derivatives, and physical tire-slip κ(a) OBD speed synthesis from GPS velocity.
* Topography, Wind & BMS 2RC PSD Validation: Distance-weighted road gradient θ(s) and headwind v_wind(t) moment preservation (constant θ during stops), VSP and EV battery power modeling, Welch Power Spectral Density (PSD) excitation matching for BMS/EKF studies, Kolmogorov-Smirnov (KS) velocity testing, and per-trip representativeness ranking.
* Visualization and Multi-Format Export: Diagnostic velocity/grade and empirical-vs-synthesized PDF plots, 8-column MADE-ready CSV, 55-column EGAD-compatible CSV, validation comparison & trip-ranking CSVs, styled multi-sheet Master XLSX, and bundled ZIP export.

Software Architecture Evolution and Modularization Plan
Stage 1. Modularization and Complete GUI-Backend Decoupling (0% GUI Coupling):
Built on strict Separation of Concerns, the V1.1.4 DCMG engine contains zero Streamlit, Tkinter, PyQt, or Flet imports and is organized into 4 isolated backend modules plus external JSON plugins:
1. modules/dcmg/config_bounds.py — Physical limits, vehicle operation presets, 8-column MADE and 55-column EGAD schemas, and plugins/dcmg/custom_presets.json loader.
2. modules/dcmg/data_loader.py — Natural numeric sorting, cached CSV/XLSX parsing, auto-column detection, single-trip validation, and demo fleet generation.
3. modules/dcmg/core_engine.py — SHA-256 canonicalization, Numba-accelerated 2D SAFD, K-Means/Silhouette clustering, Markov-Microtrip synthesis, kinematic closure, VSP/BMS PSD validation, and styled OpenPyXL workbook generation.
4. modules/dcmg/adapter.py — Asynchronous JobContext runner (live 0% → 100% progress, ETA, and cancellation), ResultStore memory-bus integration, and Master ZIP builder.

Stage 2. Local HTTP JSON API & 100% Offline Desktop Runtime:
Through core_shared/ (api_server.py, job_manager.py, result_store.py, module_loader.py, contracts.py) and a zero-CDN offline ui/ shell (index.html, styles.css, app.js), any frontend can drive the DCMG engine locally on Windows PCs without Python or internet connectivity.

Stage 3. Hot-Upgradeable Modules & Unified 4-Program Suite Integration:
Keeping modules/dcmg/ and plugins/dcmg/ external to the compiled runtime allows v1.1.5–v1.1.9 updates via lightweight module patches without rebuilding the desktop .EXE. Via the in-memory ResultStore bus, DCMG seamlessly ingests 55-column empirical trajectories directly from Program 1 (EGAD V1.1.4) and hands off synthesized 8-column cycles straight to Program 3 (MADE V1.1.4).

Scientific Role of Version 1.1.4
QwatLab EV D.C.M.G. V1.1.4 serves as a transparent, reproducible open-science framework for constructing localized, energy- and battery-excitation-equivalent EV driving cycles (such as the Tashkent Driving Cycle, TDC) suitable for Q1 peer-reviewed publications. Its SHA-256 order-invariance, 100% closed kinematic chain, and 2D SAFD / BMS PSD verification metrics ensure rigorous experimental traceability from raw GPS traces to full EV powertrain and BMS simulation.
