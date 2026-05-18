"""FastAPI server for Agro Navigation System."""
import asyncio
import json
import logging
from typing import List

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
import uvicorn

from navigation.nav_controller import NavigationController

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(title="Агро-Навігація: Система навігації агротехніки")

nav_controller = NavigationController()
simulation_running = False
simulation_paused = False
connected_clients: List[WebSocket] = []


@app.on_event("startup")
async def startup_event() -> None:
    nav_controller.initialize()
    logger.info("Navigation Controller initialized")


@app.get("/")
async def root():
    return FileResponse("static/index.html")


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    global simulation_running, simulation_paused

    await websocket.accept()
    connected_clients.append(websocket)

    try:
        while True:
            try:
                raw = await asyncio.wait_for(websocket.receive_text(), timeout=0.05)
                cmd = json.loads(raw)
                action = cmd.get("action")

                if action == "start":
                    simulation_running = True
                    simulation_paused = False
                    logger.info("Simulation STARTED")
                elif action == "stop":
                    simulation_running = False
                    logger.info("Simulation STOPPED")
                elif action == "pause":
                    simulation_paused = True
                    logger.info("Simulation PAUSED")
                elif action == "resume":
                    simulation_paused = False
                    logger.info("Simulation RESUMED")
                elif action == "reset":
                    nav_controller.reset()
                    simulation_running = False
                    simulation_paused = False
                    logger.info("Simulation RESET")
                elif action == "scenario":
                    scenario = cmd.get("name", "gnss_loss")
                    nav_controller.trigger_scenario(scenario)
                    logger.info("Scenario triggered: %s", scenario)

            except asyncio.TimeoutError:
                pass

            if simulation_running and not simulation_paused:
                try:
                    telemetry = nav_controller.step(0.1)
                    for client in connected_clients[:]:
                        try:
                            await client.send_json(telemetry, default=str)
                        except Exception:
                            connected_clients.remove(client)
                except Exception as exc:
                    logger.error("Step error: %s", exc)

            await asyncio.sleep(0.1)

    except WebSocketDisconnect:
        if websocket in connected_clients:
            connected_clients.remove(websocket)
    except Exception as exc:
        logger.error("WS error: %s", exc)
        if websocket in connected_clients:
            connected_clients.remove(websocket)


@app.get("/api/status")
async def status():
    return {
        "running": simulation_running,
        "paused": simulation_paused,
        "mode": nav_controller.mode.value,
        "time": nav_controller.simulation_time,
    }


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000, reload=False)
