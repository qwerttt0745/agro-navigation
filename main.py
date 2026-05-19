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
from contextlib import asynccontextmanager
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
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
@asynccontextmanager
async def lifespan(app: FastAPI):
    global nav_controller
    nav_controller = NavigationController()
    nav_controller.initialize()
    logger.info("=" * 60)
    logger.info("Система навігації агротехніки — ЗАПУЩЕНО")
    logger.info(f"Версія: {app.version}")
    logger.info("=" * 60)
    yield


app = FastAPI(
    title="Система навігації агротехніки",
    description="Програмне забезпечення мультисенсорної навігації з Sensor Fusion (EKF)",
    version="1.0.0",
    lifespan=lifespan
)

# ── Стан симуляції ──────────────────────────────────────────────────────────
nav_controller: NavigationController | None = None
simulation_running = False
simulation_paused = False
connected_clients: list[WebSocket] = []


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
            try:
                raw = await asyncio.wait_for(websocket.receive_text(), timeout=0.05)
                await _handle_command(json.loads(raw), websocket)
            except asyncio.TimeoutError:
                pass

            if simulation_running and not simulation_paused and nav_controller:
                try:
                    telemetry = nav_controller.step(settings.DT)
                    dead_clients = []
                    for client in connected_clients:
                        try:
                            await client.send_text(json.dumps(telemetry, default=str))
                        except Exception as e:
                            logger.error(f"WebSocket send failed: {e}", exc_info=True)
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
        await websocket.send_text(json.dumps({"status": "started"}))

    elif action == "pause":
        simulation_paused = True
        logger.info("⏸ Симуляцію ПРИЗУПИНЕНО")
        await websocket.send_text(json.dumps({"status": "paused"}))

    elif action == "resume":
        simulation_paused = False
        logger.info("▶ Симуляцію ВІДНОВЛЕНО")
        await websocket.send_text(json.dumps({"status": "resumed"}))

    elif action == "stop":
        simulation_running = False
        logger.info("⏹ Симуляцію ЗУПИНЕНО")
        await websocket.send_text(json.dumps({"status": "stopped"}))

    elif action == "reset":
        simulation_running = False
        simulation_paused = False
        if nav_controller:
            nav_controller.reset()
        logger.info("↺ Симуляцію СКИНУТО")
        await websocket.send_text(json.dumps({"status": "reset"}))

    elif action == "scenario":
        name = cmd.get("name", "gnss_loss")
        if nav_controller:
            nav_controller.trigger_scenario(name)
        logger.info(f"⚡ Сценарій запущено: {name}")
        await websocket.send_text(json.dumps({"status": f"scenario_{name}_triggered"}))

    else:
        await websocket.send_text(json.dumps({"error": f"Невідома команда: {action}"}))


if __name__ == "__main__":
    uvicorn.run(
        app,
        host=settings.HOST,
        port=settings.PORT,
        log_level="info"
    )
