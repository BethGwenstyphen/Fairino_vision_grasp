import asyncio
import re
import os
import signal
import threading
import json
import uvicorn
from fastapi import FastAPI, WebSocket
from fastapi.responses import HTMLResponse

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

app = FastAPI()

LAUNCH_SEQUENCE = [
    {"name": "机械臂底层", "cmd": "ros2", "args": ["run", "fairino_hardware", "ros2_cmd_server"], "delay": 3.0},
    {"name": "图漾相机", "cmd": "ros2", "args": ["launch", "percipio_camera", "percipio_camera.launch.py"], "delay": 5.0},
    {"name": "肌肉控制", "cmd": "ros2", "args": ["run", "handle_grasp_project", "control_node"], "delay": 3.0},
    {"name": "视觉处理", "cmd": "ros2", "args": ["run", "handle_grasp_project", "vision_node"], "delay": 10.0},
    {"name": "行为树大脑", "cmd": "ros2", "args": ["run", "bt_grasp_brain", "bt_main_node"], "delay": 0.0},
]

active_processes = []
connected_websockets = []
asyncio_loop = None

async def send_to_ui(msg_type, payload, color="#607D8B"):
    data = {"type": msg_type, "payload": payload, "color": color}
    for ws in connected_websockets:
        try: await ws.send_text(json.dumps(data))
        except: pass

# ==========================================
# ROS 2 监听节点：精准捕获行为树的 JSON 心跳
# ==========================================
class BTHMIBridge(Node):
    def __init__(self):
        super().__init__('web_hmi_bt_listener')
        self.sub = self.create_subscription(String, '/bt_status', self.bt_callback, 10)

    def bt_callback(self, msg):
        try:
            data = json.loads(msg.data)
            # 将底层遥测数据无缝甩给 Web 前端
            if asyncio_loop and asyncio_loop.is_running():
                asyncio.run_coroutine_threadsafe(send_to_ui("BT_TELEMETRY", data), asyncio_loop)
        except: pass

def ros2_spin_thread():
    rclpy.init()
    node = BTHMIBridge()
    rclpy.spin(node)
    rclpy.shutdown()

@app.on_event("startup")
def startup_event():
    global asyncio_loop
    asyncio_loop = asyncio.get_running_loop()
    threading.Thread(target=ros2_spin_thread, daemon=True).start()

# ==========================================
# 流水线与进程管理
# ==========================================
async def read_stream(stream, tag):
    while True:
        line = await stream.readline()
        if not line: break
        text = line.decode('utf-8', errors='ignore').strip()
        clean = re.sub(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])', '', text)
        if clean: 
            color = "#FF3B30" if "💥" in clean else "#A9B7C6"
            await send_to_ui("LOG", f"[{tag}] {clean}", color)

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    connected_websockets.append(websocket)
    try:
        while True:
            data = await websocket.receive_text()
            if data == "START": asyncio.create_task(start_pipeline())
            elif data == "STOP": asyncio.create_task(stop_pipeline())
    except: connected_websockets.remove(websocket)

async def start_pipeline():
    if active_processes: return
    await send_to_ui("LOG", "SYSTEM: 执行阶梯拉起序列...", "#00FF00")
    for i, task in enumerate(LAUNCH_SEQUENCE):
        await send_to_ui("STARTUP_PROGRESS", {"idx": i, "status": "STARTING"})
        process = await asyncio.create_subprocess_exec(
            task["cmd"], *task["args"], stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT, preexec_fn=os.setsid
        )
        active_processes.append(process)
        asyncio.create_task(read_stream(process.stdout, task["name"]))
        if task["delay"] > 0:
            await send_to_ui("LOG", f"SYSTEM: 预热 {task['name']} ({task['delay']}s)...", "#FFD700")
            await asyncio.sleep(task["delay"])
        await send_to_ui("STARTUP_PROGRESS", {"idx": i, "status": "READY"})

async def stop_pipeline():
    for p in reversed(active_processes):
        try: os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except: pass
    active_processes.clear()
    await send_to_ui("LOG", "SYSTEM: E-STOP 触发！进程组已斩首。", "#FF3B30")

# 引入独立的前端文件
@app.get("/")
async def get():
    with open("index.html", "r", encoding="utf-8") as f:
        return HTMLResponse(f.read())

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8080)