# 🚜 AgriNav — Система навігації агротехніки

**Курсовий проєкт** | Дисципліна: Проєктування інформаційних систем
**Університет**: КНЕУ ім. Вадима Гетьмана | **Група**: ІН-403 | **2025 р.**
**Студент**: Лопушанський Віталій Олександрович
**Керівник**: д.т.н. Артемчук Володимир Олександрович

---

## Про проєкт

Програмна реалізація мультисенсорної навігаційної системи для сільськогосподарської техніки. Система забезпечує безперервне позиціонування при деградації або повній втраті GNSS-сигналу (в тому числі від засобів РЕБ) через послідовне перемикання між трьома навігаційними режимами без зупинки техніки.

Архітектура реалізована відповідно до курсової роботи «Проєктування архітектури системи навігації агротехніки в умовах нестабільного супутникового зв'язку».

---

## Режими навігації

| Режим | Умова активації | Точність | Індикатор |
|-------|----------------|----------|-----------|
| `GNSS_RTK` | RTK Fixed, SNR > 35 дБГц | ±2 см | 🟢 |
| `DEAD_RECKONING` | Втрата GNSS 0–30 с | ±25 см / 100 м | 🟡 |
| `LIDAR_NAV` | Втрата GNSS > 30 с | ±20 см | 🟠 |
| `SAFE_STOP` | Похибка > 30 см без корекції | — | 🔴 |

---

## Архітектура

```
Симулятори сенсорів
├── GNSSSimulator     → RTK_FIXED / RTK_FLOAT / SINGLE / LOST
├── IMUSimulator      → ax, ay, gz (100 Гц)
└── LiDARSimulator    → PointCloud (16 каналів, 100 м)
		 ↓
NavigationController  → режим + сценарій
		 ↓
ExtendedKalmanFilter  → predict(IMU) + update_gnss() + update_lidar()
		 ↓
DeadReckoningModule   → інтеграція IMU при втраті GNSS
		 ↓
FastAPI WebSocket     → телеметрія 10 Гц → HMI (Leaflet + Chart.js)
```

---

## Встановлення та запуск

```bash
git clone <url>
cd Agro-Navigation

pip install -r requirements.txt

python main.py
```

Відкрити браузер: **http://localhost:8000**

### Запуск тестів

```bash
pytest tests/ -v
```

---

## Структура проєкту

```
Agro-Navigation/
├── main.py                    # FastAPI + WebSocket сервер
├── config.py                  # Параметри системи
├── navigation/
│   ├── nav_controller.py      # Оркестратор: режими, сценарії, телеметрія
│   ├── ekf.py                 # EKF: predict + update_gnss + update_lidar
│   └── dead_reckoning.py      # Інерціальна навігація (Dead Reckoning)
├── simulation/
│   ├── vehicle.py             # Кінематична модель трактора (Pure Pursuit)
│   ├── gnss_simulator.py      # Симулятор GNSS з деградацією сигналу
│   ├── imu_simulator.py       # Симулятор 6-DoF IMU (MEMS)
│   └── lidar_simulator.py     # Симулятор LiDAR (16-канальний)
├── static/
│   └── index.html             # HMI (Leaflet.js + Chart.js, темна тема)
├── tests/
│   ├── test_requirements.py   # Верифікація вимог FR/NFR/BR
│   ├── test_nav_controller.py # Тести контролера навігації
│   ├── test_ekf.py            # Тести алгоритму EKF
│   ├── test_dead_reckoning.py # Тести Dead Reckoning
│   ├── test_gnss_simulator.py # Тести симулятора GNSS
│   └── test_smoke.py          # Smoke тест
└── requirements.txt
```

---

## Верифікація вимог

| ID | Вимога | Тест | Статус |
|----|--------|------|--------|
| FR-01 | Обробка GNSS даних | `test_FR01_gnss_data_processing` | ✅ |
| FR-02 | Детекція втрати < 0.1 с | `test_FR02_gnss_loss_detection_speed` | ✅ |
| FR-03 | Sensor Fusion (EKF) | `TestEKFGNSSUpdate` | ✅ |
| FR-04 | Dead Reckoning при втраті GNSS | `test_FR04_dead_reckoning_on_gnss_loss` | ✅ |
| FR-05 | LiDAR навігація > 30 с | `test_extended_loss_triggers_lidar` | ✅ |
| FR-06 | Дані для візуалізації | `test_FR06_visualization_data_available` | ✅ |
| NFR-PER-01 | Точність RTK ≤ ±2 см | `test_rtk_accuracy_within_2cm` | ✅ |
| NFR-PER-02 | Похибка DR ≤ 30 см / 100 м | `test_NFR_PER_02_dr_accuracy_30cm_per_100m` | ✅ |
| NFR-PER-03 | Затримка ≤ 50 мс | `test_NFR_PER_03_latency_50ms` | ✅ |
| BR-01 | Немає зупинки при втраті GNSS | `test_BR01_no_complete_stop_on_gnss_loss` | ✅ |

---

## Сценарії тестування (HMI)

Кнопки в інтерфейсі запускають наступні сценарії:

- **Короткочасна втрата** — GNSS деградує до LOST на 10 с → активується Dead Reckoning → відновлення RTK
- **Тривала втрата** — GNSS LOST на 60 с → Dead Reckoning (0–30 с) → LiDAR Nav (30–60 с) → відновлення RTK

---

## Залежності

| Пакет | Версія | Призначення |
|-------|--------|-------------|
| fastapi | ≥0.104 | REST API + WebSocket |
| uvicorn | ≥0.24 | ASGI сервер |
| numpy | ≥1.24 | Матричні обчислення EKF |
| pytest | ≥7.4 | Тестування |
| pytest-asyncio | ≥0.21 | Async тести |
```
