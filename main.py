"""
Agro Navigation System - Main server
Diploma project: Navigation software for agricultural machinery
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

# Logging
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

# FastAPI
app = FastAPI(
    title="Система навігації агротехніки",
    description="Мультисенсорна навігація з Sensor Fusion (EKF)",
    version="1.0.0"
)
app.mount("/static", StaticFiles(directory="static"), name="static")

# Simulation state
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
    logger.info("Navigation system started")
    logger.info(f"Version: {app.version}")
    logger.info("=" * 60)


@app.get("/")
async def root():
    return FileResponse("static/index.html")


@app.get("/api/status")
async def get_status():
    """Current system status"""
    if not nav_controller:
        return {"error": "Controller not initialized"}
    return {
        "running": simulation_running,
        "paused": simulation_paused,
        "mode": nav_controller.mode.value,
        "simulation_time": round(nav_controller.simulation_time, 2),
        "connected_clients": len(connected_clients)
    }


@app.get("/api/report")
async def get_report():
    """Session report for diploma logging"""
    if not nav_controller:
        return {"error": "No data"}
    return nav_controller.generate_session_report()


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    global simulation_running, simulation_paused

    await websocket.accept()
    connected_clients.append(websocket)
    logger.info(f"Client connected. Total: {len(connected_clients)}")

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
                            await client.send_json(telemetry, default=str)
                        except Exception:
                            dead_clients.append(client)
                    for dead in dead_clients:
                        connected_clients.remove(dead)
                except Exception as e:
                    logger.error(f"Simulation step error: {e}", exc_info=True)

            await asyncio.sleep(settings.DT)

    except WebSocketDisconnect:
        logger.info("Client disconnected")
    except Exception as e:
        logger.error(f"WebSocket error: {e}", exc_info=True)
    finally:
        if websocket in connected_clients:
            connected_clients.remove(websocket)


async def _handle_command(cmd: dict, websocket: WebSocket):
    """Handle client commands"""
    global simulation_running, simulation_paused
    action = cmd.get("action")

    if action == "start":
        simulation_running = True
        simulation_paused = False
        logger.info("Simulation STARTED")
        await websocket.send_json({"status": "started"})

    elif action == "pause":
        simulation_paused = True
        logger.info("Simulation PAUSED")
        await websocket.send_json({"status": "paused"})

    elif action == "resume":
        simulation_paused = False
        logger.info("Simulation RESUMED")
        await websocket.send_json({"status": "resumed"})

    elif action == "stop":
        simulation_running = False
        logger.info("Simulation STOPPED")
        await websocket.send_json({"status": "stopped"})

    elif action == "reset":
        simulation_running = False
        simulation_paused = False
        if nav_controller:
            nav_controller.reset()
        logger.info("Simulation RESET")
        await websocket.send_json({"status": "reset"})

    elif action == "scenario":
        name = cmd.get("name", "gnss_loss")
        if nav_controller:
            nav_controller.trigger_scenario(name)
        logger.info(f"Scenario triggered: {name}")
        await websocket.send_json({"status": f"scenario_{name}_triggered"})

    else:
        await websocket.send_json({"error": f"Unknown action: {action}"})


if __name__ == "__main__":
    uvicorn.run(
        app,
        host=settings.HOST,
        port=settings.PORT,
        log_level="info"
    )
