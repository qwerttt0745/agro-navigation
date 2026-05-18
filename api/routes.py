"""
FastAPI routes and WebSocket handler for real-time navigation telemetry
"""
import json
import asyncio
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
import logging

# Global navigation controller instance
nav_controller = None
simulation_paused = False
simulation_task = None

logger = logging.getLogger(__name__)

router = APIRouter()


@router.get("/")
async def root():
    \"\"\"Serve index.html\"\"\"
    return FileResponse("static/index.html")


@router.get("/api/status")
async def get_status():
    \"\"\"Get current system status\"\"\"
    if not nav_controller:
        return {"error": "Navigation controller not initialized"}
    
    state = nav_controller.ekf.get_state()
    return {
        "mode": nav_controller.mode.value,
        "position": state,
        "gnss_lost_timer": nav_controller.gnss_lost_timer,
        "timestamp": nav_controller.simulation_time
    }


@router.post("/api/reset")
async def reset_simulation():
    \"\"\"Reset simulation to initial state\"\"\"
    global simulation_paused
    if not nav_controller:
        return {"error": "Navigation controller not initialized"}
    
    nav_controller.reset()
    simulation_paused = False
    return {"status": "reset successful"}


@router.post("/api/scenario/{scenario_name}")
async def trigger_scenario(scenario_name: str):
    \"\"\"Trigger a predefined scenario\"\"\"
    if not nav_controller:
        return {"error": "Navigation controller not initialized"}
    
    valid_scenarios = ["normal", "gnss_loss", "extended_loss"]
    if scenario_name not in valid_scenarios:
        return {"error": f"Invalid scenario. Valid: {valid_scenarios}"}
    
    nav_controller.trigger_scenario(scenario_name)
    return {"status": f"Scenario '{scenario_name}' triggered"}


@router.get("/api/field")
async def get_field_geojson():
    \"\"\"Get field boundaries and planned tracks as GeoJSON\"\"\"
    if not nav_controller:
        return {"error": "Navigation controller not initialized"}
    
    vehicle = nav_controller.vehicle
    base_lat = nav_controller.gnss.base_lat
    base_lon = nav_controller.gnss.base_lon
    
    # Convert field corners to lat/lon
    corners = [
        (vehicle.field_offset_x, vehicle.field_offset_y),
        (vehicle.field_offset_x + vehicle.field_width, vehicle.field_offset_y),
        (vehicle.field_offset_x + vehicle.field_width, vehicle.field_offset_y + vehicle.field_height),
        (vehicle.field_offset_x, vehicle.field_offset_y + vehicle.field_height),
        (vehicle.field_offset_x, vehicle.field_offset_y),
    ]
    
    # Convert to lat/lon
    import math
    coords = []
    for x, y in corners:
        lat = base_lat + y / 111320
        lon = base_lon + x / (111320 * math.cos(math.radians(base_lat)))
        coords.append([lon, lat])
    
    return {
        "type": "FeatureCollection",
        "features": [
            {
                "type": "Feature",
                "properties": {"name": "Field boundary"},
                "geometry": {
                    "type": "Polygon",
                    "coordinates": [coords]
                }
            }
        ]
    }


@router.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    \"\"\"WebSocket endpoint for real-time telemetry streaming\"\"\"
    global simulation_paused, simulation_task
    
    await websocket.accept()
    
    try:
        # Start simulation loop in background
        if simulation_task is None or simulation_task.done():
            simulation_task = asyncio.create_task(_run_simulation_loop(websocket))
        
        # Listen for client commands
        while True:
            data = await asyncio.wait_for(websocket.receive_text(), timeout=0.1)
            command = json.loads(data)
            action = command.get("action")
            
            if action == "pause":
                simulation_paused = True
                await websocket.send_json({"status": "paused"})
            elif action == "resume":
                simulation_paused = False
                await websocket.send_json({"status": "resumed"})
            elif action == "reset":
                nav_controller.reset()
                simulation_paused = False
                await websocket.send_json({"status": "reset"})
            elif action == "scenario":
                scenario = command.get("name")
                nav_controller.trigger_scenario(scenario)
                await websocket.send_json({"status": f"scenario {scenario} triggered"})
            else:
                await websocket.send_json({"error": "Unknown action"})
    
    except WebSocketDisconnect:
        logger.info("Client disconnected")
    except asyncio.TimeoutError:
        pass
    except Exception as e:
        logger.error(f"WebSocket error: {e}")


async def _run_simulation_loop(websocket: WebSocket):
    \"\"\"Run the main simulation loop\"\"\"
    global simulation_paused
    
    dt = 0.1  # 10 Hz simulation
    
    while True:
        try:
            if not simulation_paused and nav_controller:
                # Run one step
                telemetry = nav_controller.step(dt)
                
                # Send to client
                await websocket.send_json(telemetry, default=str)
            
            # Sleep for dt
            await asyncio.sleep(dt)
        
        except Exception as e:
            logger.error(f"Simulation error: {e}")
            break


def set_nav_controller(controller):
    \"\"\"Set the navigation controller instance\"\"\"
    global nav_controller
    nav_controller = controller
