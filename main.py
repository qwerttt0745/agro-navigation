"""FastAPI backend server for Agro Navigation System"""
import asyncio
import json
import logging
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
import uvicorn
from backend.navigation.navigation_system import NavigationSystem

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create FastAPI app
app = FastAPI(title="Agro Navigation System")

# Global navigation system
nav_system = None
simulation_running = False
current_clients = []


@app.on_event("startup")
async def startup_event():
    """Initialize navigation system on startup"""
    global nav_system
    nav_system = NavigationSystem()
    nav_system.initialize()
    logger.info("Navigation System initialized")


@app.get("/")
async def root():
    """Serve main HTML"""
    return FileResponse("frontend/index.html")


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket endpoint for real-time navigation data"""
    await websocket.accept()
    current_clients.append(websocket)
    
    try:
        logger.info("WebSocket client connected")
        
        while True:
            # Receive commands from client
            try:
                data = await asyncio.wait_for(websocket.receive_text(), timeout=1.0)
                command = json.loads(data)
                
                if command.get("action") == "start":
                    global simulation_running
                    simulation_running = True
                    logger.info("Simulation started")
                
                elif command.get("action") == "stop":
                    simulation_running = False
                    logger.info("Simulation stopped")
                
                elif command.get("action") == "reset":
                    nav_system.initialize()
                    logger.info("Navigation system reset")
            
            except asyncio.TimeoutError:
                pass
            
            # Send navigation cycle if simulation running
            if simulation_running:
                try:
                    result = nav_system.run_navigation_cycle()
                    
                    # Send to all connected clients
                    for client in current_clients:
                        try:
                            await client.send_json(result)
                        except Exception as e:
                            logger.error(f"Error sending to client: {e}")
                    
                    # 100ms cycle (10 Hz)
                    await asyncio.sleep(0.1)
                
                except Exception as e:
                    logger.error(f"Error in navigation cycle: {e}")
                    await asyncio.sleep(0.1)
            else:
                await asyncio.sleep(0.1)
    
    except WebSocketDisconnect:
        if websocket in current_clients:
            current_clients.remove(websocket)
        logger.info("WebSocket client disconnected")
    
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
        if websocket in current_clients:
            current_clients.remove(websocket)


@app.get("/status")
async def get_status():
    """Get current system status"""
    if nav_system:
        report = nav_system.generate_work_report()
        return {
            "running": simulation_running,
            "status": report,
            "connected_clients": len(current_clients)
        }
    return {"error": "System not initialized"}


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
