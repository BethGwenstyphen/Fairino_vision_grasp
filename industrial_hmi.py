import sys
import re
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                               QHBoxLayout, QPushButton, QTextEdit, QLabel, QFrame)
from PySide6.QtCore import QProcess, QTimer, Qt, Signal, Slot
from PySide6.QtGui import QFont, QColor, QTextCursor, QPalette

# ==========================================
# 核心配置：严格的启动顺序与延时策略
# ==========================================
LAUNCH_SEQUENCE = [
    {"name": "⚙️ 法奥机械臂驱动", "cmd": "ros2", "args": ["run", "fairino_hardware", "ros2_cmd_server"], "delay_after": 2000},
    {"name": "📷 图漾 3D 相机", "cmd": "ros2", "args": ["launch", "percipio_camera", "percipio_camera.launch.py"], "delay_after": 4000},
    {"name": "💪 肌肉控制节点", "cmd": "ros2", "args": ["run", "handle_grasp_project", "control_node"], "delay_after": 2000},
    {"name": "👁️ 视觉处理节点", "cmd": "ros2", "args": ["run", "handle_grasp_project", "vision_node"], "delay_after": 2000},
    {"name": "🧠 行为树大脑", "cmd": "ros2", "args": ["run", "bt_grasp_brain", "bt_main_node"], "delay_after": 0},
]

# 工业风 QSS 样式表
DARK_STYLESHEET = """
QMainWindow { background-color: #1E1E1E; }
QLabel { color: #E0E0E0; font-family: 'Segoe UI', Arial; }
QTextEdit { 
    background-color: #0D0D0D; color: #00FF00; font-family: 'Consolas', 'Courier New'; 
    font-size: 13px; border: 1px solid #333333; padding: 5px;
}
QPushButton {
    background-color: #3A3A3A; color: white; border-radius: 4px; padding: 10px; font-weight: bold; font-size: 16px;
}
QPushButton:hover { background-color: #505050; }
QPushButton#startBtn { background-color: #2E7D32; }
QPushButton#startBtn:hover { background-color: #388E3C; }
QPushButton#stopBtn { background-color: #C62828; font-size: 20px; }
QPushButton#stopBtn:hover { background-color: #D32F2F; }
QFrame#panel { background-color: #252525; border-radius: 8px; border: 1px solid #333; }
"""

class ProcessManager(QWidget):
    log_signal = Signal(str, str) # tag, message

    def __init__(self, task_info):
        super().__init__()
        self.task_info = task_info
        self.process = QProcess(self)
        self.process.readyReadStandardOutput.connect(self.handle_stdout)
        self.process.readyReadStandardError.connect(self.handle_stderr)
        
        # UI 组件：状态灯与标签
        self.layout = QHBoxLayout(self)
        self.layout.setContentsMargins(10, 5, 10, 5)
        
        self.status_light = QLabel("⬤")
        self.status_light.setStyleSheet("color: gray; font-size: 18px;")
        
        self.name_label = QLabel(self.task_info["name"])
        self.name_label.setFont(QFont("Arial", 12, QFont.Bold))
        
        self.layout.addWidget(self.status_light)
        self.layout.addWidget(self.name_label)
        self.layout.addStretch()

    def start_process(self):
        self.status_light.setStyleSheet("color: yellow; font-size: 18px;")
        self.log_signal.emit("SYSTEM", f"正在启动: {self.task_info['name']}...")
        self.process.start(self.task_info["cmd"], self.task_info["args"])
        self.status_light.setStyleSheet("color: #00E676; font-size: 18px;") # Green

    def stop_process(self):
        if self.process.state() == QProcess.Running:
            self.process.terminate()
            self.process.waitForFinished(1000)
            if self.process.state() == QProcess.Running:
                self.process.kill()
        self.status_light.setStyleSheet("color: gray; font-size: 18px;")

    def handle_stdout(self):
        data = self.process.readAllStandardOutput().data().decode('utf-8', errors='ignore')
        for line in data.splitlines():
            if line.strip(): self.log_signal.emit(self.task_info["name"], line)

    def handle_stderr(self):
        data = self.process.readAllStandardError().data().decode('utf-8', errors='ignore')
        for line in data.splitlines():
            if line.strip(): self.log_signal.emit(self.task_info["name"], f"<font color='red'>{line}</font>")


class IndustrialHMI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("全自动注液流水线中控平台 V1.0")
        self.resize(1200, 800)
        self.setStyleSheet(DARK_STYLESHEET)

        self.processes = []
        self.launch_index = 0
        self.launch_timer = QTimer()
        self.launch_timer.timeout.connect(self.launch_next)

        self.init_ui()

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # ====== 左侧：控制面板与状态监控 ======
        left_panel = QVBoxLayout()
        
        # 1. 标题
        title = QLabel("🤖 工业注液中控核")
        title.setFont(QFont("Microsoft YaHei", 24, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        left_panel.addWidget(title)

        # 2. 节点状态监控区
        status_frame = QFrame()
        status_frame.setObjectName("panel")
        status_layout = QVBoxLayout(status_frame)
        status_layout.addWidget(QLabel("<b>[ 核心节点状态监控 ]</b>"))
        
        for task in LAUNCH_SEQUENCE:
            pm = ProcessManager(task)
            pm.log_signal.connect(self.append_log)
            self.processes.append(pm)
            status_layout.addWidget(pm)
        left_panel.addWidget(status_frame)

        left_panel.addStretch()

        # 3. 巨型控制按钮
        self.btn_start = QPushButton("🚀 一键启动流水线")
        self.btn_start.setObjectName("startBtn")
        self.btn_start.setFixedHeight(80)
        self.btn_start.clicked.connect(self.start_all)
        
        self.btn_stop = QPushButton("🛑 紧急熔断 (E-STOP)")
        self.btn_stop.setObjectName("stopBtn")
        self.btn_stop.setFixedHeight(80)
        self.btn_stop.clicked.connect(self.stop_all)

        left_panel.addWidget(self.btn_start)
        left_panel.addWidget(self.btn_stop)

        main_layout.addLayout(left_panel, 1) # 左侧占比 1

        # ====== 右侧：全局日志终端 ======
        right_panel = QVBoxLayout()
        right_panel.addWidget(QLabel("<b>[ 实时系统终端 ]</b>"))
        
        self.terminal = QTextEdit()
        self.terminal.setReadOnly(True)
        right_panel.addWidget(self.terminal)
        
        main_layout.addLayout(right_panel, 2) # 右侧占比 2

        self.append_log("SYSTEM", "上位机初始化完成。准备就绪。")

    @Slot(str, str)
    def append_log(self, tag, message):
        # 简单的 ANSI 颜色代码去除
        clean_msg = re.sub(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])', '', message)
        time_str = QTimer().currentTime().toString("HH:mm:ss")
        # 如果是带有颜色的报错信息，直接拼接
        if "<font color='red'>" in clean_msg:
            formatted_msg = f"<span style='color:#757575;'>[{time_str}]</span> <b>[{tag}]</b> {clean_msg}"
        else:
            formatted_msg = f"<span style='color:#757575;'>[{time_str}]</span> <b>[{tag}]</b> <span style='color:#00E676;'>{clean_msg}</span>"
        
        self.terminal.append(formatted_msg)
        self.terminal.moveCursor(QTextCursor.End)

    def start_all(self):
        self.btn_start.setEnabled(False)
        self.append_log("SYSTEM", ">>> 接收到启动指令，正在拉起后台网络...")
        self.launch_index = 0
        self.launch_next()

    def launch_next(self):
        if self.launch_index < len(self.processes):
            pm = self.processes[self.launch_index]
            pm.start_process()
            
            delay = LAUNCH_SEQUENCE[self.launch_index]["delay_after"]
            self.launch_index += 1
            
            if delay > 0:
                self.launch_timer.start(delay)
            else:
                self.launch_timer.stop()
                self.append_log("SYSTEM", "✅ 所有核心节点已成功拉起，流水线正式接管。")
        else:
            self.launch_timer.stop()

    def stop_all(self):
        self.append_log("SYSTEM", "🚨 触发紧急熔断！正在切断所有进程...")
        self.launch_timer.stop()
        for pm in reversed(self.processes):
            pm.stop_process()
        self.btn_start.setEnabled(True)
        self.append_log("SYSTEM", "🛑 系统已安全停机。")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = IndustrialHMI()
    window.show()
    sys.exit(app.exec())