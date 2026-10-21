"""FastAPI application: serves the dashboard and bridges browser traffic to ROS 2."""
import asyncio
import json
import os
import threading
from contextlib import asynccontextmanager

import rclpy
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field
from std_msgs.msg import Bool, String

from .ros_node import DashboardRosNode

try:
    from ament_index_python.packages import get_package_share_directory
    STATIC = __import__("pathlib").Path(get_package_share_directory("wafl_dashboard")) / "static"
except (ImportError, LookupError):
    STATIC = __import__("pathlib").Path(__file__).resolve().parents[1] / "static"
clients, clients_lock = set(), asyncio.Lock()
node = None
executor_thread = None


async def broadcast(event):
    async with clients_lock:
        peers = tuple(clients)
    stale = []
    for peer in peers:
        try: await peer.send_json(event)
        except Exception: stale.append(peer)
    if stale:
        async with clients_lock:
            for peer in stale: clients.discard(peer)


@asynccontextmanager
async def lifespan(_app):
    global node, executor_thread
    rclpy.init(args=None)
    loop = asyncio.get_running_loop()
    node = DashboardRosNode(loop, broadcast)
    executor_thread = threading.Thread(target=rclpy.spin, args=(node,), name="dashboard-ros", daemon=True)
    executor_thread.start()
    try: yield
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join(timeout=2)
        node = None


app = FastAPI(title="WAFL Dashboard API", lifespan=lifespan)
app.mount("/css", StaticFiles(directory=STATIC / "css"), name="css")
app.mount("/js", StaticFiles(directory=STATIC / "js"), name="js")


@app.get("/")
def index(): return FileResponse(STATIC / "index.html")


def ros():
    if node is None: raise HTTPException(503, "ROS 2 node is starting")
    return node


class EstopRequest(BaseModel): active: bool
class ModeRequest(BaseModel): mode: str
class LiftRequest(BaseModel): cmd: int = Field(ge=0, le=2)
class LightsRequest(BaseModel): relays: list[int]
class MissionRequest(BaseModel): pickup: str; dropoff: str; priority: int = 0
class JoyRequest(BaseModel): v: float = Field(ge=-0.5, le=0.5); w: float = Field(ge=-0.8, le=0.8)


def manual_only(n):
    if n.data["mode"] != "MANUAL": raise HTTPException(409, "Command accepted only in MANUAL mode")
    if n.data["estop"]: raise HTTPException(409, "E-stop is latched")


@app.get("/api/snapshot")
def snapshot(): return ros().snapshot()

@app.get("/api/health")
def health(): return ros().health_payload()

@app.get("/api/logs")
def logs(limit: int = 500): return list(ros().logs)[-max(0, min(limit, 500)):]

@app.get("/api/map.png")
def map_png():
    if not ros().map_png: raise HTTPException(404, "Map has not arrived yet")
    return Response(ros().map_png, media_type="image/png", headers={"Cache-Control": "no-store"})

@app.get("/api/map/meta")
def map_meta():
    if not ros().map_meta: raise HTTPException(404, "Map has not arrived yet")
    return ros().map_meta

@app.post("/api/estop")
def estop(req: EstopRequest):
    n = ros(); n.pub_estop.publish(Bool(data=req.active))
    if req.active:
        n.stop(); n.pub_mode.publish(String(data="MANUAL"))
    return {"accepted": True, "active": req.active}

@app.post("/api/mode")
def mode(req: ModeRequest):
    n = ros(); value = req.mode.upper()
    if value not in ("AUTO", "MANUAL"): raise HTTPException(422, "mode must be AUTO or MANUAL")
    if value == "AUTO" and n.data["estop"]: raise HTTPException(409, "Reset E-stop before resuming AUTO")
    if value == "AUTO": n.stop()
    n.pub_mode.publish(String(data=value))
    return {"accepted": True, "mode": value}

@app.post("/api/joy")
def joy(req: JoyRequest):
    n = ros(); manual_only(n)
    try: n.joy(req.v, req.w)
    except (TypeError, ValueError) as e: raise HTTPException(409, str(e))
    return {"accepted": True}

@app.post("/api/lift")
def lift(req: LiftRequest):
    n = ros(); manual_only(n)
    n.pub_lift.publish(Int32(data=req.cmd)); return {"accepted": True}

@app.post("/api/lights")
def lights(req: LightsRequest):
    n = ros(); manual_only(n)
    if len(req.relays) != 4: raise HTTPException(422, "exactly four relay values are required")
    if any(x not in (0, 1) for x in req.relays): raise HTTPException(422, "relay values must be 0 or 1")
    n.pub_lights.publish(Int32MultiArray(data=req.relays)); return {"accepted": True}

@app.post("/api/mission")
def mission(req: MissionRequest):
    raw = json.dumps({"pickup": req.pickup, "dropoff": req.dropoff, "priority": req.priority})
    ros().pub_mission.publish(String(data=raw)); return {"accepted": True}

async def mjpeg_frames(key):
    while True:
        image = ros().camera.get(key)
        if image:
            yield b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(image)).encode() + b"\r\n\r\n" + image + b"\r\n"
        await asyncio.sleep(1 / 15)

@app.get("/stream/camera")
async def camera(): return StreamingResponse(mjpeg_frames("camera"), media_type="multipart/x-mixed-replace; boundary=frame")

@app.get("/stream/debug")
async def debug(): return StreamingResponse(mjpeg_frames("debug"), media_type="multipart/x-mixed-replace; boundary=frame")

@app.websocket("/ws")
async def websocket(websocket: WebSocket):
    await websocket.accept()
    async with clients_lock: clients.add(websocket)
    await websocket.send_json({"type": "snapshot", "data": ros().snapshot()})
    try:
        while True:
            msg = await websocket.receive_json()
            if msg.get("type") == "joy":
                n = ros()
                try:
                    manual_only(n)
                    v, w = float(msg.get("v", 0)), float(msg.get("w", 0))
                    if not (-0.5 <= v <= 0.5 and -0.8 <= w <= 0.8): raise ValueError("Joystick values exceed dashboard limits")
                    n.joy(v, w)
                except (HTTPException, ValueError, TypeError):
                    n.stop()
    except WebSocketDisconnect: pass
    finally:
        async with clients_lock: clients.discard(websocket)
        ros().stop()


def main():
    import uvicorn
    uvicorn.run("wafl_dashboard.server:app", host="0.0.0.0",
                port=int(os.environ.get("WAFL_DASHBOARD_PORT", "8000")), workers=1)
