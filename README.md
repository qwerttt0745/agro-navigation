# 🚜 Система навігації агротехніки

**Дипломна робота**  
Тема: «Розробка програмного забезпечення системи навігації агротехніки в умовах нестабільного супутникового зв'язку»

## Опис системи

Програмне забезпечення реалізує мультисенсорну навігаційну систему для сільськогосподарської техніки, яка забезпечує безперервне позиціонування при деградації або повній втраті сигналу GNSS.

### Режими навігації

| Режим | Умова активації | Точність | Колір |
|-------|----------------|----------|-------|
| GNSS_RTK | GNSS доступний | ±2 см | 🟢 Зелений |
| DEAD_RECKONING | Втрата GNSS 0-30 с | ±15 см | 🟡 Жовтий |
| LIDAR_NAV | Втрата GNSS >30 с | ±20 см | 🟠 Помаранчевий |
| SAFE_STOP | Похибка >30 см | — | 🔴 Червоний |

### Алгоритм Sensor Fusion

Система використовує Розширений фільтр Калмана (EKF) для комплексування даних:
- **GNSS/RTK**: абсолютна позиція, σ = 2 см
- **IMU (MEMS)**: кутова швидкість 100-400 Гц, дрейф <10°/год
- **LiDAR**: корекція за рядками культур, дальність 150 м

## Встановлення та запуск

```bash
# 1. Клонувати репозиторій
git clone <url>
cd Agro-Navigation

# 2. Встановити залежності
pip install -r requirements.txt

# 3. Запустити систему
python main.py

# 4. Відкрити у браузері
# http://localhost:8000
```

## Запуск тестів

```bash
# Всі тести
pytest tests/ -v

# Тільки верифікація вимог
pytest tests/test_requirements.py -v

# З виміром покриття
pytest tests/ --cov=navigation --cov-report=html
```

## Структура проєкту

```
Agro-Navigation/
├── main.py                    # FastAPI сервер (точка входу)
├── config.py                  # Конфігурація системи
├── navigation/
│   ├── nav_controller.py      # Головний контролер навігації
│   ├── ekf.py                 # Розширений фільтр Калмана (EKF)
│   └── dead_reckoning.py      # Інерціальна навігація
├── simulation/
│   ├── vehicle.py             # Кінематична модель трактора
│   ├── gnss_simulator.py      # Симулятор GNSS з деградацією
│   ├── imu_simulator.py       # Симулятор IMU
│   ├── lidar_simulator.py     # Симулятор LiDAR
│   └── scenario.py            # Сценарії втрати сигналу
├── static/
│   └── index.html             # Веб-інтерфейс (Leaflet + Chart.js)
├── tests/
│   ├── test_ekf.py            # Тести EKF алгоритму
│   ├── test_nav_controller.py # Тести контролера навігації
│   └── test_requirements.py   # Верифікація вимог курсової
├── logs/                      # Логи роботи системи
└── requirements.txt
```

## Верифікація вимог

| ID | Вимога | Тест | Результат |
|----|--------|------|-----------|
| FR-01 | Обробка GNSS даних | test_FR01_gnss_data_processing | ✅ |
| FR-02 | Детекція втрати <0.1 с | test_FR02_gnss_loss_detection_speed | ✅ |
| FR-03 | Sensor Fusion (EKF) | TestEKFGNSSUpdate | ✅ |
| FR-04 | Dead Reckoning | test_FR04_dead_reckoning | ✅ |
| FR-05 | LiDAR навігація | test_extended_loss_triggers_lidar | ✅ |
| FR-06 | Візуалізація | test_FR06_visualization_data_available | ✅ |
| NFR-PER-02 | Похибка DR ≤30 см/100 м | test_NFR_PER_02 | ✅ |
| NFR-PER-03 | Затримка ≤50 мс | test_NFR_PER_03_latency_50ms | ✅ |
| BR-01 | Без зупинки при втраті GPS | test_BR01_no_complete_stop | ✅ |
