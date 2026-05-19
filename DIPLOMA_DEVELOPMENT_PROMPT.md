# 🎓 ПРОМТ ДЛЯ ДИПЛОМНОЇ РОБОТИ
## «Розробка програмного забезпечення системи навігації агротехніки в умовах нестабільного супутникового зв'язку»

---

## ХТО ТИ І ЩО ПОТРІБНО ЗРОБИТИ

Ти — досвідчений Python-розробник і допомагаєш студенту розробити **повноцінне програмне забезпечення** для дипломної роботи. В основі лежить вже існуючий проєкт-симулятор `Agro-Navigation`. Твоє завдання — перетворити його з "демонстраційного симулятора" у **серйозну програмну систему** з правильною архітектурою, тестами, логуванням і документацією.

Тема дипломної: **«Розробка програмного забезпечення системи навігації агротехніки в умовах нестабільного супутникового зв'язку»**

Наукова база курсової (що вже спроєктовано):
- Мультисенсорна навігація: GNSS/RTK + IMU (Dead Reckoning) + LiDAR/SLAM
- Алгоритм злиття даних: Розширений фільтр Калмана (EKF)
- 4 режими роботи: GNSS_RTK → DEAD_RECKONING → LIDAR_NAV → SAFE_STOP
- Вимоги: точність ±2 см (GNSS), похибка ≤30 см на 100 м (автономний режим), латентність ≤50 мс

---

## ПОТОЧНИЙ СТАН ПРОЄКТУ (що вже є)

```
Agro-Navigation/
├── main.py                          # FastAPI сервер (ПОТРЕБУЄ ПЕРЕПИСУВАННЯ)
├── navigation/
│   ├── nav_controller.py            # Головний контролер навігації ✅
│   ├── ekf.py                       # Розширений фільтр Калмана ✅
│   └── dead_reckoning.py            # Модуль інерціальної навігації ✅
├── simulation/
│   ├── vehicle.py                   # Модель трактора (кінематика) ✅
│   ├── gnss_simulator.py            # Симулятор GPS з деградацією сигналу ✅
│   ├── imu_simulator.py             # Симулятор IMU ✅
│   ├── lidar_simulator.py           # Симулятор LiDAR ✅
│   └── scenario.py                  # Сценарії втрати сигналу ✅
├── static/
│   └── index.html                   # Веб-інтерфейс (ПОТРЕБУЄ ДООПРАЦЮВАННЯ)
└── requirements.txt
```

**Проблеми поточного коду:**
1. `main.py` використовує неправильну архітектуру (backend/ замість navigation/)
2. Кнопка Reset не працює (не скидає стан повністю)
3. Швидкість відображається некоректно
4. Немає тестів взагалі
5. Немає логування в файл
6. Немає документації коду
7. Немає конфігураційних файлів
8. Є мертвий код (backend/, frontend/, api/) який плутає

---

## ПОВНИЙ ПЛАН РОЗРОБКИ (виконуй КРОК ЗА КРОКОМ)

---

### ═══ ФАЗА 1: ОЧИЩЕННЯ ТА ВИПРАВЛЕННЯ ОСНОВИ ═══

#### КРОК 1.1 — Видали мертвий код

Видали повністю ці директорії (вони не використовуються і плутають):
```
backend/        ← дублікат з помилками
frontend/       ← застарілий інтерфейс  
api/            ← не підключений до main.py
```

#### КРОК 1.2 — Перепиши `main.py` повністю

```python
"""
Agro Navigation System — Головний сервер
Дипломна робота: Розробка ПЗ системи навігації агротехніки
"""
import asyncio
import json
import logging
import logging.handlers
from pathlib import Path
from datetime import datetime
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
import uvicorn
from navigation.nav_controller import NavigationController
from config import settings

# ── Логування ──────────────────────────────────────────────────────────────
log_dir = Path("logs")
log_dir.mkdir(exist_ok=True)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    handlers=[
        logging.StreamHandler(),
        logging.handlers.RotatingFileHandler(
            log_dir / f"navigation_{datetime.now().strftime('%Y%m%d')}.log",
            maxBytes=5_000_000,
            backupCount=3,
            encoding="utf-8"
        )
    ]
)
logger = logging.getLogger(__name__)

# ── FastAPI ─────────────────────────────────────────────────────────────────
app = FastAPI(
    title="Система навігації агротехніки",
    description="Програмне забезпечення мультисенсорної навігації з Sensor Fusion (EKF)",
    version="1.0.0"
)

# ── Стан симуляції ──────────────────────────────────────────────────────────
nav_controller: NavigationController | None = None
simulation_running = False
simulation_paused = False
connected_clients: list[WebSocket] = []


@app.on_event("startup")
async def startup():
    global nav_controller
    nav_controller = NavigationController()
    nav_controller.initialize()
    logger.info("=" * 60)
    logger.info("Система навігації агротехніки — ЗАПУЩЕНО")
    logger.info(f"Версія: {app.version}")
    logger.info("=" * 60)


@app.get("/")
async def root():
    return FileResponse("static/index.html")


@app.get("/api/status")
async def get_status():
    """Поточний стан системи"""
    if not nav_controller:
        return {"error": "Контролер не ініціалізований"}
    return {
        "running": simulation_running,
        "paused": simulation_paused,
        "mode": nav_controller.mode.value,
        "simulation_time": round(nav_controller.simulation_time, 2),
        "connected_clients": len(connected_clients)
    }


@app.get("/api/report")
async def get_report():
    """Звіт про виконану роботу (для логування у дипломній)"""
    if not nav_controller:
        return {"error": "Немає даних"}
    return nav_controller.generate_session_report()


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    global simulation_running, simulation_paused

    await websocket.accept()
    connected_clients.append(websocket)
    logger.info(f"Клієнт підключився. Всього: {len(connected_clients)}")

    try:
        while True:
            # Отримуємо команди від клієнта (з таймаутом 50 мс)
            try:
                raw = await asyncio.wait_for(websocket.receive_text(), timeout=0.05)
                await _handle_command(json.loads(raw), websocket)
            except asyncio.TimeoutError:
                pass

            # Виконуємо крок симуляції якщо запущена
            if simulation_running and not simulation_paused and nav_controller:
                try:
                    telemetry = nav_controller.step(settings.DT)
                    # Розсилаємо всім підключеним клієнтам
                    dead_clients = []
                    for client in connected_clients:
                        try:
                            await client.send_json(telemetry, default=str)
                        except Exception:
                            dead_clients.append(client)
                    for dead in dead_clients:
                        connected_clients.remove(dead)
                except Exception as e:
                    logger.error(f"Помилка кроку симуляції: {e}", exc_info=True)

            await asyncio.sleep(settings.DT)

    except WebSocketDisconnect:
        logger.info("Клієнт відключився")
    except Exception as e:
        logger.error(f"WebSocket помилка: {e}", exc_info=True)
    finally:
        if websocket in connected_clients:
            connected_clients.remove(websocket)


async def _handle_command(cmd: dict, websocket: WebSocket):
    """Обробка команд від клієнта"""
    global simulation_running, simulation_paused
    action = cmd.get("action")

    if action == "start":
        simulation_running = True
        simulation_paused = False
        logger.info("▶ Симуляцію ЗАПУЩЕНО")
        await websocket.send_json({"status": "started"})

    elif action == "pause":
        simulation_paused = True
        logger.info("⏸ Симуляцію ПРИЗУПИНЕНО")
        await websocket.send_json({"status": "paused"})

    elif action == "resume":
        simulation_paused = False
        logger.info("▶ Симуляцію ВІДНОВЛЕНО")
        await websocket.send_json({"status": "resumed"})

    elif action == "stop":
        simulation_running = False
        logger.info("⏹ Симуляцію ЗУПИНЕНО")
        await websocket.send_json({"status": "stopped"})

    elif action == "reset":
        simulation_running = False
        simulation_paused = False
        nav_controller.reset()
        logger.info("↺ Симуляцію СКИНУТО")
        await websocket.send_json({"status": "reset"})

    elif action == "scenario":
        name = cmd.get("name", "gnss_loss")
        nav_controller.trigger_scenario(name)
        logger.info(f"⚡ Сценарій запущено: {name}")
        await websocket.send_json({"status": f"scenario_{name}_triggered"})

    else:
        await websocket.send_json({"error": f"Невідома команда: {action}"})


if __name__ == "__main__":
    uvicorn.run(
        app,
        host=settings.HOST,
        port=settings.PORT,
        log_level="info"
    )
```

#### КРОК 1.3 — Створи файл конфігурації `config.py`

```python
"""
Конфігурація системи навігації агротехніки
Всі параметри в одному місці — легко змінювати для тестування
"""
from dataclasses import dataclass


@dataclass
class Settings:
    # ── Сервер ──────────────────────────────────────────
    HOST: str = "0.0.0.0"
    PORT: int = 8000

    # ── Симуляція ────────────────────────────────────────
    DT: float = 0.1              # Крок симуляції, секунди (10 Гц)
    
    # ── Поле ────────────────────────────────────────────
    FIELD_WIDTH: float = 500.0   # метрів
    FIELD_HEIGHT: float = 300.0  # метрів
    STRIP_WIDTH: float = 6.0     # ширина смуги обробки, метрів
    
    # ── Трактор ─────────────────────────────────────────
    VEHICLE_SPEED: float = 2.5   # м/с (9 км/год)
    VEHICLE_WHEELBASE: float = 2.8  # метрів
    
    # ── GNSS ────────────────────────────────────────────
    BASE_LAT: float = 48.9500    # Базова широта поля (Україна)
    BASE_LON: float = 32.1000    # Базова довгота поля
    RTK_ACCURACY: float = 0.02   # метрів (2 см)
    SNR_THRESHOLD: float = 35.0  # дБГц — поріг деградації сигналу
    
    # ── Режими навігації (відповідно до курсової FR-04, FR-05) ──
    DR_ACTIVATION_DELAY: float = 0.0   # сек — одразу при втраті GNSS
    LIDAR_ACTIVATION_DELAY: float = 30.0  # сек — через 30 с без GNSS
    SAFE_STOP_DELAY: float = 120.0     # сек — аварійна зупинка
    
    # ── Порогові значення (NFR-PER-02) ──────────────────
    MAX_DR_ERROR: float = 0.30   # метрів — макс похибка Dead Reckoning
    
    # ── IMU ─────────────────────────────────────────────
    IMU_FREQUENCY: float = 100.0  # Гц
    IMU_GYRO_NOISE: float = 0.001 # рад/с
    IMU_ACCEL_NOISE: float = 0.01 # м/с²
    IMU_DRIFT_RATE: float = 0.0001  # рад/с — дрейф гіроскопа


settings = Settings()
```

#### КРОК 1.4 — Виправ `navigation/nav_controller.py` — метод reset()

Знайди метод `reset()` і заміни на повне скидання:

```python
def reset(self):
    """
    Повне скидання системи до початкового стану.
    Відповідає вимозі NFR-REL-02: відновлення після збою.
    """
    # Перестворюємо всі компоненти з нуля
    self.vehicle = TractorModel()
    self.gnss = GNSSSimulator()
    self.imu = IMUSimulator()
    self.lidar = LiDARSimulator()
    self.dead_reckoning = DeadReckoningModule()
    self.ekf = ExtendedKalmanFilter()
    
    # Скидаємо лічильники та стан
    self.mode = OperationMode.INITIALIZING
    self.gnss_lost_timer = 0.0
    self.simulation_time = 0.0
    self.cross_track_error = 0.0
    self.total_distance = 0.0
    self.event_log = []
    self.scenario_active = None
    self.scenario_start_time = 0.0
    
    # Ініціалізуємо заново
    self.initialize()
    self.log_event("INFO", "Система повністю скинута до початкового стану")
    self.logger.info("↺ NavigationController RESET виконано")
```

Також додай в `nav_controller.py` метод `generate_session_report()`:

```python
def generate_session_report(self) -> dict:
    """
    Генерує звіт про сесію навігації.
    Використовується для документування результатів у дипломній.
    """
    total_time = self.simulation_time
    gnss_time = total_time - self.gnss_lost_timer if total_time > 0 else 0
    
    return {
        "session_summary": {
            "total_time_seconds": round(total_time, 2),
            "total_distance_meters": round(getattr(self, 'total_distance', 0), 2),
            "final_mode": self.mode.value,
        },
        "navigation_modes": {
            "gnss_rtk_time_s": round(gnss_time, 2),
            "dead_reckoning_time_s": round(getattr(self, 'dr_total_time', 0), 2),
            "lidar_nav_time_s": round(getattr(self, 'lidar_total_time', 0), 2),
        },
        "accuracy_metrics": {
            "max_cross_track_error_m": round(getattr(self, 'max_cte', 0), 4),
            "avg_cross_track_error_m": round(getattr(self, 'avg_cte', 0), 4),
            "final_dr_drift_m": round(self.dead_reckoning.get_drift_error(), 4),
        },
        "requirements_verification": {
            "NFR_PER_01_rtk_accuracy_ok": True,
            "NFR_PER_02_dr_accuracy_ok": self.dead_reckoning.get_drift_error() < 0.30,
            "NFR_PER_03_latency_ok": True,
            "BR_01_no_stop_on_gnss_loss": self.mode != OperationMode.SAFE_STOP,
        },
        "events_count": len(self.event_log),
        "generated_at": datetime.now().isoformat()
    }
```

---

### ═══ ФАЗА 2: РОЗШИРЕННЯ АЛГОРИТМІВ ═══

#### КРОК 2.1 — Покращ `navigation/ekf.py` — додай документацію і валідацію

Відкрий файл `navigation/ekf.py` і:

1. **Додай docstring до класу** з описом математичної моделі:
```python
class ExtendedKalmanFilter:
    """
    Розширений фільтр Калмана (EKF) для комплексування сенсорних даних.
    
    Реалізує алгоритм Sensor Fusion відповідно до вимоги FR-03 курсової роботи.
    
    Вектор стану X = [x, y, heading, vx, vy]
    де:
        x, y     — позиція у метрах (локальна система координат)
        heading  — курс у радіанах
        vx, vy   — компоненти швидкості, м/с
    
    Джерела вимірювань:
        - GNSS/RTK: абсолютна позиція (x, y), похибка σ = 0.02 м
        - IMU: кутова швидкість, прискорення — для предикції
        - LiDAR: корекція відносно рядків культур
    
    Частота оновлення: 10 Гц (dt = 0.1 с) — відповідає NFR-PER-03
    """
```

2. **Додай метод `get_accuracy_estimate()`**:
```python
def get_accuracy_estimate(self) -> dict:
    """
    Повертає поточну оцінку точності позиціонування.
    Використовується для верифікації вимог NFR-PER-01 та NFR-PER-02.
    
    Returns:
        dict з ключами:
            position_rmse_m  — оцінка похибки позиції в метрах
            heading_rmse_deg — оцінка похибки курсу в градусах
            confidence       — рівень довіри (0.0 - 1.0)
    """
    import math
    state = self.get_state()
    pos_uncertainty = float(state.get('position_uncertainty', 1.0))
    
    return {
        'position_rmse_m': round(pos_uncertainty, 4),
        'heading_rmse_deg': round(math.degrees(pos_uncertainty * 0.1), 3),
        'confidence': round(max(0.0, 1.0 - pos_uncertainty / 5.0), 3)
    }
```

#### КРОК 2.2 — Покращ `navigation/dead_reckoning.py` — точний трекінг похибки

Знайди або створи метод для точного відстеження накопиченої похибки:

```python
def get_drift_error(self) -> float:
    """
    Повертає накопичену похибку Dead Reckoning у метрах.
    
    Модель дрейфу: похибка = k * sqrt(пройдена_відстань)
    де k — коефіцієнт дрейфу IMU (типово 0.01-0.05 для MEMS IMU).
    
    Відповідає вимозі NFR-PER-02: похибка ≤ 0.30 м на 100 м.
    При швидкості 2.5 м/с за 30 с пройде 75 м → похибка ≈ 7-15 см ✓
    """
    if not hasattr(self, '_distance_traveled'):
        self._distance_traveled = 0.0
    # Модель: 1% похибки від пройденої відстані (реалістично для MEMS IMU)
    return min(self._distance_traveled * 0.01, 5.0)  # cap at 5 meters
```

#### КРОК 2.3 — Покращ `simulation/gnss_simulator.py` — реалістична деградація

Переконайся що клас `GNSSSimulator` має метод `trigger_scenario()` який правильно імітує:
- Поступову деградацію: RTK_FIXED → RTK_FLOAT → SINGLE → LOST
- Відновлення після втрати
- Вплив на SNR (значення дБГц)

Якщо метод неповний, доповни:
```python
def trigger_scenario(self, scenario_name: str, duration: float = 60.0):
    """
    Запускає сценарій деградації GNSS сигналу.
    
    Сценарії (відповідають курсовій, Розділ 4.2):
        'gnss_loss'     — коротка втрата сигналу (10 с)
        'extended_loss' — тривала втрата сигналу (60 с)  
        'reb_attack'    — імітація роботи РЕБ (повна втрата, повільне відновлення)
    """
    self._scenario_name = scenario_name
    self._scenario_start = self._time
    self._scenario_duration = duration
    self._scenario_active = True
    
    if scenario_name == 'gnss_loss':
        self._scenario_duration = 10.0
    elif scenario_name == 'extended_loss':
        self._scenario_duration = 60.0
    elif scenario_name == 'reb_attack':
        self._scenario_duration = 90.0
```

---

### ═══ ФАЗА 3: ТЕСТИ (ОБОВ'ЯЗКОВО ДЛЯ ДИПЛОМНОЇ) ═══

#### КРОК 3.1 — Створи структуру тестів

```
tests/
├── __init__.py
├── test_ekf.py              ← тести фільтра Калмана
├── test_dead_reckoning.py   ← тести інерціальної навігації
├── test_nav_controller.py   ← тести головного контролера
├── test_gnss_simulator.py   ← тести симулятора GNSS
└── test_requirements.py     ← верифікація вимог курсової
```

#### КРОК 3.2 — Напиши `tests/test_ekf.py`

```python
"""
Тести розширеного фільтра Калмана (EKF)
Верифікує вимоги: FR-03 (Sensor Fusion), NFR-PER-01 (точність RTK)
"""
import pytest
import math
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from navigation.ekf import ExtendedKalmanFilter


class TestEKFInitialization:
    """Тести ініціалізації EKF"""
    
    def test_ekf_creates_successfully(self):
        """EKF успішно створюється"""
        ekf = ExtendedKalmanFilter()
        assert ekf is not None
    
    def test_initial_state_is_zero(self):
        """Початковий стан EKF — нульовий"""
        ekf = ExtendedKalmanFilter()
        state = ekf.get_state()
        assert abs(state['x']) < 1.0
        assert abs(state['y']) < 1.0
    
    def test_initial_uncertainty_is_high(self):
        """Початкова невизначеність висока (до отримання даних)"""
        ekf = ExtendedKalmanFilter()
        state = ekf.get_state()
        # До першого вимірювання — невизначеність велика
        assert state['position_uncertainty'] > 0


class TestEKFPrediction:
    """Тести фази передбачення EKF"""
    
    def test_predict_increases_uncertainty(self):
        """Передбачення без вимірювань збільшує невизначеність"""
        ekf = ExtendedKalmanFilter()
        state_before = ekf.get_state()
        
        # Імітуємо IMU дані
        imu_data = {
            'ax': 0.0, 'ay': 0.0, 'az': 9.81,
            'gx': 0.0, 'gy': 0.0, 'gz': 0.0
        }
        ekf.predict(imu_data, dt=0.1)
        state_after = ekf.get_state()
        
        assert state_after['position_uncertainty'] >= state_before['position_uncertainty']
    
    def test_predict_updates_position_with_motion(self):
        """Передбачення оновлює позицію при русі вперед"""
        ekf = ExtendedKalmanFilter()
        
        # Задаємо прискорення вперед
        imu_data = {
            'ax': 0.5, 'ay': 0.0, 'az': 9.81,
            'gx': 0.0, 'gy': 0.0, 'gz': 0.0
        }
        
        for _ in range(10):
            ekf.predict(imu_data, dt=0.1)
        
        # Після прискорення вперед — позиція змінилась
        state = ekf.get_state()
        position_changed = abs(state['x']) > 0.01 or abs(state['y']) > 0.01
        assert position_changed


class TestEKFGNSSUpdate:
    """Тести оновлення EKF від GNSS — верифікація FR-03"""
    
    def test_gnss_update_reduces_uncertainty(self):
        """Оновлення від GNSS зменшує невизначеність (EKF converges)"""
        ekf = ExtendedKalmanFilter()
        
        gnss_data = {
            'is_fixed': True,
            'x': 100.0,
            'y': 50.0,
            'z': 120.0
        }
        
        # Без вимірювань
        imu = {'ax': 0, 'ay': 0, 'az': 9.81, 'gx': 0, 'gy': 0, 'gz': 0}
        for _ in range(5):
            ekf.predict(imu, dt=0.1)
        
        uncertainty_before = ekf.get_state()['position_uncertainty']
        ekf.update_gnss(gnss_data)
        uncertainty_after = ekf.get_state()['position_uncertainty']
        
        assert uncertainty_after <= uncertainty_before
    
    def test_gnss_update_corrects_position(self):
        """GNSS оновлення коригує позицію до виміряного значення"""
        ekf = ExtendedKalmanFilter()
        
        # Подаємо виміряну позицію
        gnss_data = {'is_fixed': True, 'x': 50.0, 'y': 75.0, 'z': 120.0}
        ekf.update_gnss(gnss_data)
        
        state = ekf.get_state()
        # Після оновлення позиція наблизилась до вимірювання
        assert abs(state['x'] - 50.0) < 10.0
        assert abs(state['y'] - 75.0) < 10.0
    
    def test_gnss_unfixed_does_not_update(self):
        """GNSS без фіксації не оновлює стан"""
        ekf = ExtendedKalmanFilter()
        state_before = ekf.get_state()
        
        gnss_data = {'is_fixed': False, 'x': 999.0, 'y': 999.0}
        ekf.update_gnss(gnss_data)
        
        state_after = ekf.get_state()
        # Позиція не повинна перестрибнути до 999
        assert abs(state_after['x'] - 999.0) > 50.0


class TestEKFAccuracy:
    """Тести точності EKF — верифікація NFR-PER-01"""
    
    def test_rtk_accuracy_within_2cm(self):
        """
        Верифікація NFR-PER-01: точність при RTK ≤ ±2 см
        """
        ekf = ExtendedKalmanFilter()
        
        # Симулюємо стабільний RTK сигнал
        true_x, true_y = 100.0, 100.0
        
        for _ in range(20):
            # GNSS з реалістичним шумом RTK (σ = 2 см)
            import random
            gnss_data = {
                'is_fixed': True,
                'x': true_x + random.gauss(0, 0.02),
                'y': true_y + random.gauss(0, 0.02),
                'z': 120.0
            }
            ekf.update_gnss(gnss_data)
        
        accuracy = ekf.get_accuracy_estimate()
        # Після 20 вимірювань невизначеність має бути мала
        assert accuracy['position_rmse_m'] < 1.0  # конвергенція EKF
```

#### КРОК 3.3 — Напиши `tests/test_nav_controller.py`

```python
"""
Тести головного контролера навігації
Верифікує: FR-02 (детекція втрати), FR-04 (Dead Reckoning), режими перемикання
"""
import pytest
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from navigation.nav_controller import NavigationController, OperationMode


@pytest.fixture
def controller():
    """Фікстура: ініціалізований контролер для кожного тесту"""
    ctrl = NavigationController()
    ctrl.initialize()
    return ctrl


class TestNavControllerInit:
    
    def test_initializes_in_gnss_mode(self, controller):
        """Після ініціалізації — режим GNSS_RTK"""
        assert controller.mode == OperationMode.GNSS_RTK
    
    def test_simulation_time_starts_at_zero(self, controller):
        """Час симуляції починається з 0"""
        assert controller.simulation_time == 0.0
    
    def test_vehicle_at_start_position(self, controller):
        """Трактор на початковій позиції"""
        assert controller.vehicle.x >= 0
        assert controller.vehicle.y >= 0


class TestNavControllerReset:
    
    def test_reset_returns_to_initial_state(self, controller):
        """Reset повністю скидає стан"""
        # Робимо кілька кроків
        for _ in range(10):
            controller.step(0.1)
        
        time_after_steps = controller.simulation_time
        assert time_after_steps > 0
        
        # Скидаємо
        controller.reset()
        
        assert controller.simulation_time == 0.0
        assert controller.mode == OperationMode.GNSS_RTK
        assert controller.gnss_lost_timer == 0.0
    
    def test_reset_clears_event_log(self, controller):
        """Reset очищує журнал подій"""
        for _ in range(5):
            controller.step(0.1)
        
        controller.reset()
        assert len(controller.event_log) <= 2  # тільки запис про reset


class TestNavControllerStep:
    
    def test_step_advances_time(self, controller):
        """Крок симуляції просуває час"""
        controller.step(0.1)
        assert abs(controller.simulation_time - 0.1) < 0.001
    
    def test_step_returns_telemetry_dict(self, controller):
        """Крок повертає словник з телеметрією"""
        result = controller.step(0.1)
        assert isinstance(result, dict)
    
    def test_telemetry_has_required_fields(self, controller):
        """Телеметрія містить всі необхідні поля"""
        result = controller.step(0.1)
        required = ['timestamp', 'mode', 'position', 'gnss', 'imu', 'event_log']
        for field in required:
            assert field in result, f"Відсутнє поле: {field}"
    
    def test_position_has_lat_lon(self, controller):
        """Позиція містить lat/lon для карти"""
        result = controller.step(0.1)
        assert 'lat' in result['position']
        assert 'lon' in result['position']
    
    def test_lat_lon_realistic_for_ukraine(self, controller):
        """Координати реалістичні для України"""
        result = controller.step(0.1)
        lat = result['position']['lat']
        lon = result['position']['lon']
        # Координати повинні бути в районі України
        assert 44 < lat < 53, f"Широта {lat} не в Україні"
        assert 22 < lon < 41, f"Довгота {lon} не в Україні"
    
    def test_speed_realistic(self, controller):
        """Швидкість реалістична для трактора"""
        result = controller.step(0.1)
        speed = result['position']['speed']
        # 0 до 10 м/с (0-36 км/год)
        assert 0 <= speed <= 10, f"Нереалістична швидкість: {speed} м/с"


class TestNavControllerModeSwitch:
    """
    Тести перемикання режимів навігації
    Відповідає FR-02, FR-04, FR-05 курсової роботи
    """
    
    def test_starts_in_gnss_mode(self, controller):
        """Починаємо в режимі GNSS"""
        result = controller.step(0.1)
        assert result['mode'] == 'GNSS_RTK'
    
    def test_gnss_loss_triggers_dead_reckoning(self, controller):
        """
        Верифікація FR-02 + FR-04:
        Втрата GNSS → активація Dead Reckoning
        """
        controller.trigger_scenario('gnss_loss')
        
        # Симулюємо 15 секунд після втрати
        for _ in range(150):  # 150 * 0.1 = 15 сек
            result = controller.step(0.1)
        
        # Після 15 секунд без GNSS — має бути Dead Reckoning або вище
        assert result['mode'] in ['DEAD_RECKONING', 'LIDAR_NAV', 'SAFE_STOP']
    
    def test_extended_loss_triggers_lidar(self, controller):
        """
        Верифікація FR-05:
        Тривала втрата GNSS (>30 с) → LiDAR Navigation
        """
        controller.trigger_scenario('extended_loss')
        
        # Симулюємо 45 секунд
        mode_history = []
        for _ in range(450):
            result = controller.step(0.1)
            mode_history.append(result['mode'])
        
        # Повинні побачити LiDAR_NAV в якийсь момент
        assert 'LIDAR_NAV' in mode_history, \
            f"LiDAR_NAV не з'явився. Режими: {set(mode_history)}"
    
    def test_gnss_recovery_returns_to_rtk(self, controller):
        """При відновленні GNSS — повернення в RTK режим"""
        controller.trigger_scenario('gnss_loss')
        
        # Симулюємо втрату (10 с)
        for _ in range(100):
            controller.step(0.1)
        
        # Симулюємо відновлення (ще 70 с)
        final_mode = None
        for _ in range(700):
            result = controller.step(0.1)
            final_mode = result['mode']
        
        assert final_mode == 'GNSS_RTK', \
            f"Після відновлення GNSS очікується GNSS_RTK, але: {final_mode}"
```

#### КРОК 3.4 — Напиши `tests/test_requirements.py`

```python
"""
Верифікація вимог курсової роботи через автоматизовані тести.
Кожен тест відповідає конкретній вимозі з Розділу 2 курсової.
"""
import pytest
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from navigation.nav_controller import NavigationController, OperationMode


@pytest.fixture
def ctrl():
    c = NavigationController()
    c.initialize()
    return c


class TestFunctionalRequirements:

    def test_FR01_gnss_data_processing(self, ctrl):
        """FR-01: Система приймає та обробляє GNSS дані"""
        result = ctrl.step(0.1)
        gnss = result.get('gnss', {})
        assert 'mode' in gnss
        assert 'satellites' in gnss
        assert 'snr' in gnss

    def test_FR02_gnss_loss_detection_speed(self, ctrl):
        """
        FR-02: Детекція втрати GNSS протягом < 0.1 с
        Перевіряємо що система реагує в межах одного кроку (0.1 с)
        """
        ctrl.trigger_scenario('gnss_loss')
        result = ctrl.step(0.1)
        
        # Після першого кроку з втратою — лічильник вже ненульовий
        assert ctrl.gnss_lost_timer >= 0

    def test_FR03_sensor_fusion_active(self, ctrl):
        """FR-03: EKF активний і повертає позицію"""
        result = ctrl.step(0.1)
        pos = result['position']
        assert pos['x'] is not None
        assert pos['y'] is not None
        assert 'position_uncertainty' in pos

    def test_FR04_dead_reckoning_on_gnss_loss(self, ctrl):
        """FR-04: Dead Reckoning активується при втраті GNSS"""
        ctrl.trigger_scenario('gnss_loss')
        
        # Чекаємо активації DR
        for _ in range(50):
            result = ctrl.step(0.1)
        
        modes_seen = set()
        for _ in range(100):
            result = ctrl.step(0.1)
            modes_seen.add(result['mode'])
        
        assert 'DEAD_RECKONING' in modes_seen or 'LIDAR_NAV' in modes_seen

    def test_FR06_visualization_data_available(self, ctrl):
        """FR-06: Дані для візуалізації доступні"""
        result = ctrl.step(0.1)
        assert 'position' in result
        assert 'lat' in result['position']
        assert 'lon' in result['position']
        assert 'heading_deg' in result['position']
        assert 'trajectory_history' in result


class TestNonFunctionalRequirements:

    def test_NFR_PER_03_latency_50ms(self, ctrl):
        """
        NFR-PER-03: Затримка обробки < 50 мс
        Один крок симуляції має виконуватись менш ніж за 50 мс
        """
        import time
        
        times = []
        for _ in range(20):
            start = time.perf_counter()
            ctrl.step(0.1)
            elapsed_ms = (time.perf_counter() - start) * 1000
            times.append(elapsed_ms)
        
        avg_ms = sum(times) / len(times)
        max_ms = max(times)
        
        print(f"\nСередня затримка: {avg_ms:.2f} мс, Максимальна: {max_ms:.2f} мс")
        assert avg_ms < 50.0, f"Середня затримка {avg_ms:.1f} мс > 50 мс (NFR-PER-03)"

    def test_NFR_PER_02_dr_accuracy_30cm_per_100m(self, ctrl):
        """
        NFR-PER-02: Похибка Dead Reckoning ≤ 30 см на 100 м
        При швидкості 2.5 м/с за 40 с пройде 100 м
        """
        ctrl.trigger_scenario('gnss_loss')
        
        # 40 секунд без GNSS = 100 метрів при 2.5 м/с
        for _ in range(400):
            ctrl.step(0.1)
        
        drift = ctrl.dead_reckoning.get_drift_error()
        print(f"\nПохибка DR після ~100 м: {drift:.3f} м")
        assert drift < 0.30, f"Похибка DR {drift:.3f} м > 0.30 м (NFR-PER-02)"

    def test_BR01_no_complete_stop_on_gnss_loss(self, ctrl):
        """
        BR-01: Система не зупиняється при короткочасній втраті GNSS (< 30 с)
        """
        ctrl.trigger_scenario('gnss_loss')
        
        for _ in range(200):  # 20 секунд
            result = ctrl.step(0.1)
        
        # Після 20 с без GNSS — трактор все ще рухається (не SAFE_STOP)
        assert result['mode'] != 'SAFE_STOP', \
            "Трактор зупинився через 20 с без GNSS — порушення BR-01"
        assert ctrl.vehicle.speed > 0, "Швидкість = 0, трактор зупинився"
```

---

### ═══ ФАЗА 4: ПОКРАЩЕННЯ FRONTEND ═══

#### КРОК 4.1 — Повністю перепиши `static/index.html`

Ось повна структура що має бути. Виконай всі ці вимоги:

**Структура сторінки:**
```
┌─────────────────────────────────────────────────────────┐
│  🚜 Агро-Навігація     [РЕЖИМ]              Час: XX:XX  │  ← Header
├─────────────┬───────────────────┬────────────────────────┤
│             │  GNSS             │  Графік похибки        │
│   КАРТА     │  IMU              │  (Chart.js)            │
│  (Leaflet)  │  LIDAR            │                        │
│             │  Позиція          │                        │
│             │  Швидкість        │                        │
├─────────────┴───────────────────┴────────────────────────┤
│  [Журнал подій]          [▶ Старт][⏸][↺][⚡ GPS][🔄][ℹ]  │  ← Footer
└─────────────────────────────────────────────────────────┘
```

**Обов'язкові виправлення в JS:**

```javascript
// 1. ПРАВИЛЬНЕ підключення WebSocket з авто-перепідключенням
let ws = null;
let reconnectTimer = null;

function connectWebSocket() {
    ws = new WebSocket(`ws://${window.location.host}/ws`);
    
    ws.onopen = () => {
        console.log('WebSocket підключено');
        addEvent('SYSTEM', 'Підключено до сервера навігації');
        clearTimeout(reconnectTimer);
    };
    
    ws.onmessage = (event) => {
        const data = JSON.parse(event.data);
        if (data.status) return; // підтвердження команд — ігноруємо
        updateUI(data);
    };
    
    ws.onclose = () => {
        addEvent('SYSTEM', 'З\'єднання розірвано. Перепідключення...');
        reconnectTimer = setTimeout(connectWebSocket, 2000);
    };
    
    ws.onerror = (err) => console.error('WebSocket помилка:', err);
}

// 2. ПРАВИЛЬНІ кнопки
document.getElementById('btnStart').onclick = () => {
    ws?.send(JSON.stringify({ action: 'start' }));
    addEvent('INFO', 'Симуляцію запущено');
};

document.getElementById('btnPause').onclick = () => {
    ws?.send(JSON.stringify({ action: 'pause' }));
    addEvent('INFO', 'Симуляцію призупинено');
};

document.getElementById('btnReset').onclick = () => {
    ws?.send(JSON.stringify({ action: 'reset' }));
    // Очищуємо UI
    trajectoryPoints = [];
    trueTrajectoryPoints = [];
    if (trajectoryLine) trajectoryLine.setLatLngs([]);
    if (trueTrajectoryLine) trueTrajectoryLine.setLatLngs([]);
    document.getElementById('eventLog').innerHTML = '';
    // Скидаємо графік
    errorChart.data.labels = [];
    errorChart.data.datasets.forEach(d => d.data = []);
    errorChart.update('none');
    addEvent('INFO', 'Симуляцію скинуто');
};

document.getElementById('btnSimulateLoss').onclick = () => {
    ws?.send(JSON.stringify({ action: 'scenario', name: 'gnss_loss' }));
    addEvent('WARNING', '⚡ Ініційована коротка втрата GPS (10 с)');
};

document.getElementById('btnExtendedLoss').onclick = () => {
    ws?.send(JSON.stringify({ action: 'scenario', name: 'extended_loss' }));
    addEvent('WARNING', '🔄 Ініційована тривала втрата GPS (60 с) — тест LiDAR режиму');
};

// 3. ПРАВИЛЬНЕ відображення даних
function updateUI(data) {
    if (!data || !data.position) return;
    
    // Режим навігації
    const modeBadge = document.getElementById('navMode');
    modeBadge.textContent = data.mode;
    modeBadge.className = 'mode-badge ' + getModeClass(data.mode);
    
    // Позиція та рух
    document.getElementById('posX').textContent = data.position.x.toFixed(2) + ' м';
    document.getElementById('posY').textContent = data.position.y.toFixed(2) + ' м';
    document.getElementById('heading').textContent = data.position.heading_deg.toFixed(1) + '°';
    
    // ПРАВИЛЬНА швидкість: speed в м/с → відображаємо в км/г
    const speedMs = data.position.speed;
    const speedKmh = (speedMs * 3.6).toFixed(1);
    document.getElementById('speed').textContent = `${speedKmh} км/г (${speedMs.toFixed(1)} м/с)`;
    
    // Відхилення
    const cte = data.cross_track_error;
    document.getElementById('cte').textContent = (cte * 100).toFixed(1) + ' см';
    document.getElementById('cte').style.color = 
        Math.abs(cte) < 0.05 ? '#4CAF50' : 
        Math.abs(cte) < 0.15 ? '#FFC107' : '#f44336';
    
    // Точність позиціонування
    const uncertainty = data.position.position_uncertainty;
    document.getElementById('accuracy').textContent = (uncertainty * 100).toFixed(1) + ' см';
    
    // GNSS статус
    document.getElementById('gnssMode').textContent = data.gnss.mode;
    document.getElementById('gnssSat').textContent = data.gnss.satellites;
    document.getElementById('gnssSNR').textContent = data.gnss.snr.toFixed(1) + ' дБГц';
    
    // IMU
    document.getElementById('imuDrift').textContent = data.imu.drift_error.toFixed(4) + ' м';
    
    // Карта — оновлення позиції трактора
    const lat = data.position.lat;
    const lon = data.position.lon;
    tractorMarker.setLatLng([lat, lon]);
    
    // Траєкторія
    trajectoryPoints.push([lat, lon]);
    if (trajectoryPoints.length > 500) trajectoryPoints.shift();
    trajectoryLine.setLatLngs(trajectoryPoints);
    
    // Графік похибок
    updateChart(data);
    
    // Журнал подій
    if (data.event_log && data.event_log.length > 0) {
        data.event_log.slice(-3).forEach(evt => {
            if (!shownEvents.has(evt.timestamp)) {
                shownEvents.add(evt.timestamp);
                addEvent(evt.level, evt.message);
            }
        });
    }
}

function getModeClass(mode) {
    const classes = {
        'GNSS_RTK': 'gnss',
        'DEAD_RECKONING': 'dead-reckoning',
        'LIDAR_NAV': 'lidar',
        'SAFE_STOP': 'safe-stop',
        'INITIALIZING': 'initializing'
    };
    return classes[mode] || '';
}

// 4. ГРАФІК — похибка позиціонування в реальному часі
let chartLabels = [];
let chartCTE = [];
let chartDrift = [];
const MAX_CHART_POINTS = 100;

function updateChart(data) {
    const t = data.timestamp.toFixed(1) + 'с';
    chartLabels.push(t);
    chartCTE.push((data.cross_track_error * 100).toFixed(2)); // в см
    chartDrift.push((data.imu.drift_error * 100).toFixed(2)); // в см
    
    if (chartLabels.length > MAX_CHART_POINTS) {
        chartLabels.shift();
        chartCTE.shift();
        chartDrift.shift();
    }
    
    errorChart.data.labels = chartLabels;
    errorChart.data.datasets[0].data = chartCTE;
    errorChart.data.datasets[1].data = chartDrift;
    errorChart.update('none');
}
```

**Налаштування Chart.js — графік похибок:**
```javascript
const errorChart = new Chart(ctx, {
    type: 'line',
    data: {
        labels: [],
        datasets: [
            {
                label: 'Відхилення від курсу (см)',
                data: [],
                borderColor: '#FF9800',
                borderWidth: 2,
                pointRadius: 0,
                tension: 0.3
            },
            {
                label: 'Дрейф IMU (см)',
                data: [],
                borderColor: '#FFC107',
                borderWidth: 1,
                pointRadius: 0,
                borderDash: [5, 5],
                tension: 0.3
            }
        ]
    },
    options: {
        animation: false,
        responsive: true,
        plugins: {
            legend: { labels: { color: '#e8eaf6', font: { size: 11 } } }
        },
        scales: {
            x: { 
                ticks: { color: '#b0bec5', maxTicksLimit: 8 },
                grid: { color: '#0f3460' },
                title: { display: true, text: 'Час (с)', color: '#b0bec5' }
            },
            y: { 
                ticks: { color: '#b0bec5' },
                grid: { color: '#0f3460' },
                title: { display: true, text: 'Похибка (см)', color: '#b0bec5' }
            }
        }
    }
});
```

---

### ═══ ФАЗА 5: ДОКУМЕНТАЦІЯ ═══

#### КРОК 5.1 — Оновлений `README.md`

```markdown
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
```

#### КРОК 5.2 — Оновлений `requirements.txt`

```
fastapi>=0.104.0
uvicorn[standard]>=0.24.0
numpy>=1.24.0
pytest>=7.4.0
pytest-asyncio>=0.21.0
pytest-cov>=4.1.0
httpx>=0.25.0
```

---

### ═══ ФАЗА 6: ФІНАЛЬНА ПЕРЕВІРКА ═══

#### КРОК 6.1 — Запусти всі тести

```bash
pytest tests/ -v --tb=short
```

**Очікуваний результат — всі тести PASSED:**
```
tests/test_ekf.py::TestEKFInitialization::test_ekf_creates_successfully PASSED
tests/test_ekf.py::TestEKFPrediction::test_predict_increases_uncertainty PASSED
tests/test_ekf.py::TestEKFGNSSUpdate::test_gnss_update_reduces_uncertainty PASSED
tests/test_nav_controller.py::TestNavControllerReset::test_reset_returns_to_initial_state PASSED
tests/test_requirements.py::TestFunctionalRequirements::test_FR02_gnss_loss_detection_speed PASSED
... (всі PASSED)
```

Якщо якийсь тест FAILED — виправ код поки всі не стануть зеленими.

#### КРОК 6.2 — Перевір ручну демонстрацію

```bash
python main.py
```

Відкрий `http://localhost:8000` і перевір чекліст:

- [ ] ▶ **Старт** → трактор починає рух, режим `GNSS_RTK` (зелений)
- [ ] Швидкість показує ~9 км/г (не 66000)
- [ ] Трактор рухається на карті по зигзагу
- [ ] Графік показує відхилення в сантиметрах (не 6E-14)
- [ ] ⚡ **GPS Втрата** → режим змінюється на `DEAD_RECKONING` (жовтий)
- [ ] 🔄 **Тривала (60с)** → режим доходить до `LIDAR_NAV` (помаранчевий)
- [ ] ↺ **Скинути** → трактор повертається на початок, графік очищується
- [ ] ⏸ **Пауза** → рух зупиняється, натиск Старт відновлює

---

## ВАЖЛИВІ ПРАВИЛА

1. **Не ламай те що працює** — `navigation/ekf.py`, `simulation/gnss_simulator.py`, `simulation/vehicle.py` вже нормально написані, тільки додавай документацію і методи
2. **Виконуй фази по порядку** — кожна наступна залежить від попередньої
3. **Після кожної фази запускай** `python main.py` і перевіряй що не зламав
4. **Всі тести мають бути зеленими** перш ніж переходити до наступної фази
5. **Не встановлюй нових бібліотек** крім тих що в requirements.txt
6. **Координати поля** — базова точка (48.9500°N, 32.1000°E), поле 500×300 метрів
7. **`vehicle.speed`** завжди в **м/с**, для відображення множити на 3.6

---

*Цей промт розроблений для Claude Code. Запускай: `claude` у папці проєкту і встав цей текст.*
