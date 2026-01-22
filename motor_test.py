import sys
import time
import atexit
from dataclasses import dataclass

from PyQt5 import QtCore, QtWidgets, QtGui
from pymodbus.client import ModbusSerialClient

# -----------------------
# ZLAC8030D Modbus Map
# -----------------------
class REG:
    CONTROL_MODE = 0x200D
    CONTROL_WORD = 0x200E
    SYNC_ASYNC   = 0x200F
    
    ACC_L = 0x2080; ACC_R = 0x2081
    DEC_L = 0x2082; DEC_R = 0x2083
    TARGET_VEL_L = 0x2088; TARGET_VEL_R = 0x2089
    ACT_VEL_L    = 0x20AB; ACT_VEL_R    = 0x20AC
    TPOS_H_L = 0x208A; TPOS_L_L = 0x208B
    TPOS_H_R = 0x208C; TPOS_L_R = 0x208D
    TSPD_L   = 0x208E; TSPD_R   = 0x208F
    APOS_H_L = 0x20A7; APOS_L_L = 0x20A8
    APOS_H_R = 0x20A9; APOS_L_R = 0x20AA
    TSLOPE_L = 0x2086; TSLOPE_R = 0x2087
    TTORQUE_L= 0x2090; TTORQUE_R= 0x2091
    ATORQUE_L= 0x20AD; ATORQUE_R= 0x20AE
    CLEAR_FEEDBACK_POS = 0x2005  # 현재 엔코더(피드백) 위치값을 0으로 클리어
    SET_ZERO = 0x2006            # Absolute 모드에서 영점 재설정
    
    # [안전장치] 통신 타임아웃 설정 레지스터 (0x2014 or similar depending on FW)
    # PC가 죽으면 드라이버가 알아서 멈추게 하는 핵심 설정
    COM_TIMEOUT = 0x2014 

@dataclass
class SerialCfg:
    port: str = "COM3"
    baudrate: int = 115200
    slave: int = 1

# -----------------------
# [UI] 바퀴 시각화
# -----------------------
class WheelWidget(QtWidgets.QWidget):
    def __init__(self, label="L"):
        super().__init__()
        self.setMinimumSize(80, 120)
        self.total_counts = 0
        self.label = label
        self.color = QtGui.QColor(60, 100, 200) if label == "L" else QtGui.QColor(200, 60, 60)

    def set_position(self, counts):
        self.total_counts = counts
        self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        w, h = self.width(), self.height()
        rect = QtCore.QRectF(10, 10, w-20, h-20)

        path = QtGui.QPainterPath()
        path.addRoundedRect(rect, 10, 10)
        path.setFillRule(QtCore.Qt.WindingFill)
        
        painter.setBrush(QtGui.QBrush(self.color))
        painter.setPen(QtGui.QPen(QtCore.Qt.black, 2))
        painter.drawPath(path)

        offset = -(self.total_counts / 20.0) % 30
        painter.setClipPath(path)
        painter.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 100), 4))
        
        for i in range(-1, int(h/30) + 2):
            y = i * 30 + offset
            painter.drawLine(QtCore.QPointF(10, y), QtCore.QPointF(w-10, y))

        painter.setClipping(False)
        painter.setPen(QtCore.Qt.black)
        painter.setFont(QtGui.QFont("Arial", 12, QtGui.QFont.Bold))
        painter.drawText(rect, QtCore.Qt.AlignCenter, self.label)

# -----------------------
# [UI] 키보드 상태
# -----------------------
class KeyPadWidget(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(140, 100)
        self.keys = {'up': False, 'down': False, 'left': False, 'right': False}

    def set_key_state(self, key, state):
        if key in self.keys:
            self.keys[key] = state
            self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        layout = {'up':(50,5,40,40), 'left':(5,50,40,40), 'down':(50,50,40,40), 'right':(95,50,40,40)}
        painter.setPen(QtGui.QPen(QtCore.Qt.black, 2))
        font = QtGui.QFont("Arial", 12, QtGui.QFont.Bold); painter.setFont(font)

        for key, (x, y, w, h) in layout.items():
            rect = QtCore.QRectF(x, y, w, h)
            color = QtGui.QColor(0, 150, 255) if self.keys[key] else QtGui.QColor(220, 220, 220)
            painter.setBrush(color); painter.drawRoundedRect(rect, 5, 5)
            symbol = {'up': '▲', 'down': '▼', 'left': '◀', 'right': '▶'}[key]
            painter.setPen(QtCore.Qt.white if self.keys[key] else QtCore.Qt.black)
            painter.drawText(rect, QtCore.Qt.AlignCenter, symbol)

# -----------------------
# [UI] 가상 조이스틱
# -----------------------
class VirtualJoystick(QtWidgets.QWidget):
    sig_speed = QtCore.pyqtSignal(int, int)

    def __init__(self):
        super().__init__()
        self.setMinimumSize(200, 200)
        self.moving = False 
        self.pos_x = 0.0; self.pos_y = 0.0
        self.key_x = 0.0; self.key_y = 0.0
        self.max_rpm = 50
        self.snap_threshold = 0.2
        self.current_l = 0.0; self.current_r = 0.0
        self.ramp_step = 5.0
        self.timer = QtCore.QTimer(); self.timer.timeout.connect(self.emit_speed); self.timer.start(100) 

    def set_max_rpm(self, val): self.max_rpm = val
    
    def update_keys(self, u, d, l, r):
        self.key_y = 0.0; self.key_x = 0.0
        if u: self.key_y += 1.0
        if d: self.key_y -= 1.0
        if r: self.key_x += 1.0
        if l: self.key_x -= 1.0
        self.key_x = max(-1.0, min(1.0, self.key_x))
        self.key_y = max(-1.0, min(1.0, self.key_y))

    def stop_immediate(self):
        self.current_l = 0; self.current_r = 0; self.sig_speed.emit(0, 0)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        w, h = self.width(), self.height()
        center = QtCore.QPointF(w/2, h/2); radius = min(w, h) / 2 - 10
        
        painter.setBrush(QtGui.QColor(50, 50, 50))
        painter.drawEllipse(center, radius, radius)
        painter.setPen(QtGui.QPen(QtCore.Qt.gray, 1, QtCore.Qt.DashLine))
        painter.drawLine(center.x()-radius, center.y(), center.x()+radius, center.y())
        painter.drawLine(center.x(), center.y()-radius, center.x(), center.y()+radius)

        curr_x = self.pos_x if self.moving else self.key_x
        curr_y = self.pos_y if self.moving else self.key_y
        if abs(curr_x) < self.snap_threshold: curr_x = 0
        if abs(curr_y) < self.snap_threshold: curr_y = 0

        draw_y = -curr_y 
        handle_center = QtCore.QPointF(center.x() + curr_x * radius, center.y() + draw_y * radius)
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor(50, 200, 50) if (curr_x == 0 or curr_y == 0) else QtGui.QColor(200, 50, 50))
        painter.drawEllipse(handle_center, 30, 30)

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton: self.moving = True; self.update_position(event.pos())
    def mouseMoveEvent(self, event):
        if self.moving: self.update_position(event.pos())
    def mouseReleaseEvent(self, event):
        self.moving = False; self.pos_x = 0.0; self.pos_y = 0.0; self.update()

    def update_position(self, mouse_pos):
        w, h = self.width(), self.height()
        center_x, center_y = w / 2, h / 2
        radius = min(w, h) / 2 - 10
        dx = mouse_pos.x() - center_x; dy = mouse_pos.y() - center_y
        dist = math.sqrt(dx*dx + dy*dy)
        if dist > radius: ratio = radius / dist; dx *= ratio; dy *= ratio
        self.pos_x = dx / radius; self.pos_y = -dy / radius; self.update()

    def emit_speed(self):
        raw_x = self.pos_x if self.moving else self.key_x
        raw_y = self.pos_y if self.moving else self.key_y
        if abs(raw_x) < self.snap_threshold: raw_x = 0 
        if abs(raw_y) < self.snap_threshold: raw_y = 0 

        if not self.moving and raw_x == 0 and raw_y == 0:
            if abs(self.current_l) < 1 and abs(self.current_r) < 1:
                self.current_l = 0; self.current_r = 0; self.sig_speed.emit(0, 0); self.update(); return

        throttle = raw_y * self.max_rpm
        turn = raw_x * self.max_rpm * 0.5 
        target_l = throttle + turn; target_r = throttle - turn 

        def ramp(curr, target, step):
            if curr < target:
                curr += step; 
                if curr > target: curr = target
            elif curr > target:
                curr -= step
                if curr < target: curr = target
            return curr

        self.current_l = ramp(self.current_l, target_l, self.ramp_step)
        self.current_r = ramp(self.current_r, target_r, self.ramp_step)
        self.sig_speed.emit(int(self.current_l), int(self.current_r)); self.update()


# -----------------------
# 통신 워커
# -----------------------
class MotorWorker(QtCore.QThread):
    sig_status = QtCore.pyqtSignal(str)
    sig_feedback = QtCore.pyqtSignal(dict)
    sig_error = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.cfg = SerialCfg()
        self.client = None
        self.running = False
        self.is_connected = False
        self.cmd_queue = []
        self.mutex = QtCore.QMutex()

    def connect_serial(self, port, baud, slave):
        self.cfg.port = port; self.cfg.baudrate = baud; self.cfg.slave = slave; self.start()

    def disconnect_serial(self):
        self.running = False; self.wait()
        if self.client: self.client.close(); self.is_connected = False; self.sig_status.emit("Disconnected")

    def queue_command(self, func, *args, **kwargs):
        self.mutex.lock(); self.cmd_queue.append((func, args, kwargs)); self.mutex.unlock()

    def run(self):
        self.running = True
        try:
            self.client = ModbusSerialClient(port=self.cfg.port, baudrate=self.cfg.baudrate, parity='N', stopbits=1, bytesize=8, timeout=0.1)
            if self.client.connect(): 
                self.is_connected = True; 
                self.sig_status.emit(f"Connected to {self.cfg.port}")
                
                # [중요 안전장치] 연결 즉시 Heartbeat(Watchdog) 설정
                # 500ms(0.5초) 동안 통신 없으면 드라이버가 스스로 멈춤
                self._write(REG.COM_TIMEOUT, 500) 
                self.sig_status.emit("Safety Watchdog Enabled (500ms)")
                
            else: self.sig_status.emit("Connection Failed"); self.running = False; return
        except Exception as e: self.sig_error.emit(str(e)); self.running = False; return

        while self.running:
            self.mutex.lock()
            while self.cmd_queue:
                func, args, kwargs = self.cmd_queue.pop(0)
                try: func(*args, **kwargs)
                except Exception as e: self.sig_error.emit(f"CMD Error: {e}")
            self.mutex.unlock()
            try: 
                if self.is_connected: self._read_status()
            except Exception: pass
            self.msleep(100)

    def _write(self, addr, val):
        self.client.write_register(addr, int(val) & 0xFFFF, slave=self.cfg.slave)

    def _write_multi(self, addr, vals):
        self.client.write_registers(addr, [int(v) & 0xFFFF for v in vals], slave=self.cfg.slave)

    def _s16(self, val): return val - 0x10000 if val & 0x8000 else val
    def _s32(self, high, low): 
        val = (high << 16) | low
        return val - 0x100000000 if val & 0x80000000 else val

    def _read_status(self):
        fb = {}
        slave = self.cfg.slave
        # 통신 에러가 나도 죽지 않게 예외처리 강화
        try:
            rr = self.client.read_holding_registers(REG.ACT_VEL_L, 2, slave=slave)
            if not rr.isError(): fb['vl'] = self._s16(rr.registers[0]) / 10.0; fb['vr'] = self._s16(rr.registers[1]) / 10.0
            rr = self.client.read_holding_registers(REG.APOS_H_L, 4, slave=slave)
            if not rr.isError(): fb['pl'] = self._s32(rr.registers[0], rr.registers[1]); fb['pr'] = self._s32(rr.registers[2], rr.registers[3])
            rr = self.client.read_holding_registers(REG.ATORQUE_L, 2, slave=slave)
            if not rr.isError(): fb['tl'] = self._s16(rr.registers[0]) / 10.0; fb['tr'] = self._s16(rr.registers[1]) / 10.0
            self.sig_feedback.emit(fb)
        except:
            pass # 읽기 에러는 무시 (쓰기는 큐에서 처리)

    def cmd_enable(self, enable: bool): self._write(REG.CONTROL_WORD, 0x08 if enable else 0x07)
    def cmd_clear_fault(self): self._write(REG.CONTROL_WORD, 0x06)

    def _safe_change_mode(self, mode, sync_val=1):
        self._write(REG.CONTROL_WORD, 0x07); self.msleep(50)
        self._write(REG.CONTROL_WORD, 0x00); self.msleep(100)
        self._write(REG.CONTROL_MODE, mode); self._write(REG.SYNC_ASYNC, sync_val); self.msleep(100)
        self._write(REG.CONTROL_WORD, 0x08)

    def cmd_set_mode_vel(self, acc, dec):
        self._safe_change_mode(3, 1)
        self._write(REG.ACC_L, acc); self._write(REG.ACC_R, acc)
        self._write(REG.DEC_L, dec); self._write(REG.DEC_R, dec)

    def cmd_write_vel(self, vl, vr):
        self._write(REG.TARGET_VEL_L, int(vl)); self._write(REG.TARGET_VEL_R, int(vr))

    def cmd_set_mode_pos(self, absolute_mode, acc, dec):
        self._safe_change_mode(2 if absolute_mode else 1, 1)
        self._write(REG.ACC_L, acc); self._write(REG.ACC_R, acc)
        self._write(REG.DEC_L, dec); self._write(REG.DEC_R, dec)

    def cmd_write_pos_and_start(self, pl, pr, speed_l, speed_r):
        self._write(REG.TSPD_L, int(speed_l)); self._write(REG.TSPD_R, int(speed_r))
        def split(v): v = int(v); return ((v >> 16) & 0xFFFF, v & 0xFFFF) if v >= 0 else (((v + (1<<32)) >> 16) & 0xFFFF, (v + (1<<32)) & 0xFFFF)
        l_hi, l_lo = split(pl); r_hi, r_lo = split(pr)
        self._write_multi(REG.TPOS_H_L, [l_hi, l_lo, r_hi, r_lo])
        self.msleep(50); self._write(REG.CONTROL_WORD, 0x10) 
    
    def cmd_clear_feedback_pos(self): 
        """현재 엔코더(피드백) 위치값을 0으로 클리어 (1=L, 2=R, 3=Both)"""
        self._write(REG.CLEAR_FEEDBACK_POS, 3)
    
    def cmd_set_zero(self): 
        """Absolute 모드에서 영점 재설정 (1=L, 2=R, 3=Both)"""
        self._write(REG.SET_ZERO, 3)

    def cmd_set_mode_torque(self, slope):
        self._write(REG.CONTROL_WORD, 0x07); self.msleep(50)
        self._write(REG.TTORQUE_L, 0); self._write(REG.TTORQUE_R, 0) 
        self._safe_change_mode(4, 1)
        self._write(REG.TSLOPE_L, slope); self._write(REG.TSLOPE_R, slope)

    def cmd_write_torque(self, tl, tr):
        self._write(REG.TTORQUE_L, int(tl)); self._write(REG.TTORQUE_R, int(tr))


# -----------------------
# GUI Main Window
# -----------------------
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ZLAC8030D Pro Controller V18 (Safety Watchdog)")
        self.resize(950, 850)
        
        self.worker = MotorWorker()
        self.worker.sig_status.connect(self.on_status)
        self.worker.sig_feedback.connect(self.update_feedback)
        self.worker.sig_error.connect(lambda e: self.lblStatus.setText(f"Error: {e}"))

        self.init_ui()
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        
        # [안전] 프로그램 종료 시 정지 신호 전송 시도
        atexit.register(self.emergency_stop)

    def emergency_stop(self):
        # 파이썬 종료 시 최대한 정지 시도
        if self.worker.is_connected:
            try:
                self.worker.cmd_enable(False)
            except:
                pass

    def init_ui(self):
        central = QtWidgets.QWidget(); self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)

        # 1. Connection
        h = QtWidgets.QHBoxLayout()
        self.portEdit = QtWidgets.QLineEdit("COM3"); h.addWidget(QtWidgets.QLabel("Port")); h.addWidget(self.portEdit)
        self.baudCombo = QtWidgets.QComboBox(); self.baudCombo.addItems(["115200", "57600", "38400", "9600"]); h.addWidget(QtWidgets.QLabel("Baud")); h.addWidget(self.baudCombo)
        self.btnConn = QtWidgets.QPushButton("Connect"); self.btnConn.clicked.connect(self.toggle_connect); h.addWidget(self.btnConn)
        layout.addLayout(h)

        # 2. RUN / STOP
        h_run = QtWidgets.QHBoxLayout()
        self.btnRun = QtWidgets.QPushButton("RUN (Enable)"); self.btnRun.setMinimumHeight(50); self.btnRun.setCheckable(True); self.btnRun.setStyleSheet("QPushButton:checked { background-color: #00FF00; font-weight: bold; }"); self.btnRun.clicked.connect(self.on_run_clicked)
        self.btnStop = QtWidgets.QPushButton("STOP (Disable)"); self.btnStop.setMinimumHeight(50); self.btnStop.setStyleSheet("background-color: #FF5555; font-weight: bold; color: white;"); self.btnStop.clicked.connect(self.on_stop_clicked)
        self.btnClear = QtWidgets.QPushButton("Clear Alarm"); self.btnClear.setMinimumHeight(50); self.btnClear.clicked.connect(lambda: self.worker.queue_command(self.worker.cmd_clear_fault))
        h_run.addWidget(self.btnRun); h_run.addWidget(self.btnStop); h_run.addWidget(self.btnClear)
        layout.addLayout(h_run)

        # 3. Global Settings Panel
        grp_glob = QtWidgets.QGroupBox("Global Control Settings"); layout.addWidget(grp_glob)
        h_glob = QtWidgets.QHBoxLayout(grp_glob)
        self.chkSync = QtWidgets.QCheckBox("Sync (Input L -> R follows inverted)"); self.chkSync.setChecked(True); self.chkSync.setStyleSheet("font-weight: bold; color: blue;")
        self.chkInvL = QtWidgets.QCheckBox("H/W Invert L"); self.chkInvR = QtWidgets.QCheckBox("H/W Invert R"); self.chkInvR.setChecked(True) 
        h_glob.addWidget(self.chkSync); h_glob.addStretch(); h_glob.addWidget(self.chkInvL); h_glob.addWidget(self.chkInvR)

        # 4. Tabs
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.addTab(self.ui_velocity(), "1. Velocity")
        self.tabs.addTab(self.ui_position_rel(), "2. Relative Pos")
        self.tabs.addTab(self.ui_position_abs(), "3. Absolute Pos")
        self.tabs.addTab(self.ui_torque(), "4. Torque")
        self.tabs.addTab(self.ui_graphic(), "5. Joy & Key")
        layout.addWidget(self.tabs)

        fb_layout = QtWidgets.QHBoxLayout()
        self.lblFbVel = QtWidgets.QLabel("Vel: 0.0 / 0.0 rpm"); self.lblFbPos = QtWidgets.QLabel("Pos: 0 / 0 cnt"); self.lblFbTq = QtWidgets.QLabel("Tq: 0.0 / 0.0 A")
        font = self.lblFbVel.font(); font.setPointSize(10); font.setBold(True)
        self.lblFbVel.setFont(font); self.lblFbPos.setFont(font); self.lblFbTq.setFont(font)
        fb_layout.addWidget(self.lblFbVel); fb_layout.addWidget(self.lblFbPos); fb_layout.addWidget(self.lblFbTq)
        layout.addLayout(fb_layout)
        
        # Position Reset Button
        h_reset = QtWidgets.QHBoxLayout()
        self.btnResetPos = QtWidgets.QPushButton("🔄 Reset Position (Set 0/0)")
        self.btnResetPos.setMinimumHeight(40)
        self.btnResetPos.setStyleSheet("background-color: #FFD700; font-weight: bold; color: black;")
        self.btnResetPos.clicked.connect(self.on_reset_position)
        h_reset.addStretch()
        h_reset.addWidget(self.btnResetPos)
        h_reset.addStretch()
        layout.addLayout(h_reset)
        
        self.lblStatus = QtWidgets.QLabel("Disconnected"); layout.addWidget(self.lblStatus)

    # --- 1. Velocity ---
    def ui_velocity(self):
        w = QtWidgets.QWidget(); l = QtWidgets.QFormLayout(w)
        self.vAcc = QtWidgets.QSpinBox(); self.vAcc.setRange(0,30000); self.vAcc.setValue(500)
        self.vL = QtWidgets.QSpinBox(); self.vL.setRange(-3000,3000)
        self.vR = QtWidgets.QSpinBox(); self.vR.setRange(-3000,3000)
        self.vL.lineEdit().returnPressed.connect(self.send_velocity)
        self.vR.lineEdit().returnPressed.connect(self.send_velocity)
        
        btnInit = QtWidgets.QPushButton("Set Velocity Mode (OK Button)")
        btnInit.setStyleSheet("background-color: #DDDDFF; font-weight: bold;")
        btnInit.clicked.connect(lambda: self.worker.queue_command(self.worker.cmd_set_mode_vel, self.vAcc.value(), self.vAcc.value()))
        
        l.addRow(btnInit); l.addRow("Acc(ms):", self.vAcc); l.addRow("Target L:", self.vL); l.addRow("Target R:", self.vR)
        return w

    # --- [New] MM Conversion Helper ---
    def convert_input(self, val_l, val_r, use_mm, scale):
        if use_mm:
            ratio = 1000.0 / scale
            return int(val_l * ratio), int(val_r * ratio)
        return int(val_l), int(val_r)

    # --- 2. Relative Position ---
    def ui_position_rel(self):
        w = QtWidgets.QWidget(); l = QtWidgets.QFormLayout(w)
        h_unit = QtWidgets.QHBoxLayout()
        self.prUseMM = QtWidgets.QCheckBox("Use MM Unit"); self.prUseMM.setStyleSheet("color: darkgreen; font-weight: bold;")
        self.prScale = QtWidgets.QDoubleSpinBox(); self.prScale.setRange(1.0, 10000.0); self.prScale.setValue(180.0)
        h_unit.addWidget(self.prUseMM); h_unit.addWidget(QtWidgets.QLabel("Dist(mm) for 1000p:")); h_unit.addWidget(self.prScale)
        l.addRow(h_unit)

        self.prAcc = QtWidgets.QSpinBox(); self.prAcc.setRange(0,30000); self.prAcc.setValue(500)
        self.prSpd = QtWidgets.QSpinBox(); self.prSpd.setRange(1, 3000); self.prSpd.setValue(20); self.prSpd.setSuffix(" rpm")
        
        self.prPosL = QtWidgets.QDoubleSpinBox(); self.prPosL.setRange(-2000000000, 2000000000); self.prPosL.setDecimals(1)
        self.prPosR = QtWidgets.QDoubleSpinBox(); self.prPosR.setRange(-2000000000, 2000000000); self.prPosR.setDecimals(1)
        self.prPosL.lineEdit().returnPressed.connect(lambda: self.send_position(False))
        self.prPosR.lineEdit().returnPressed.connect(lambda: self.send_position(False))

        btnInit = QtWidgets.QPushButton("Set Relative Mode (OK Button)")
        btnInit.setStyleSheet("background-color: #DDDDFF; font-weight: bold;")
        btnInit.clicked.connect(lambda: self.worker.queue_command(self.worker.cmd_set_mode_pos, False, self.prAcc.value(), self.prAcc.value())) 
        btnGo = QtWidgets.QPushButton("Go Relative [Enter]")
        btnGo.clicked.connect(lambda: self.send_position(False))

        l.addRow(btnInit); l.addRow("Acc:", self.prAcc); l.addRow("Speed:", self.prSpd); l.addRow("Move L:", self.prPosL); l.addRow("Move R:", self.prPosR); l.addRow(btnGo)
        return w

    # --- 3. Absolute Position ---
    def ui_position_abs(self):
        w = QtWidgets.QWidget(); l = QtWidgets.QFormLayout(w)
        h_unit = QtWidgets.QHBoxLayout()
        self.paUseMM = QtWidgets.QCheckBox("Use MM Unit"); self.paUseMM.setStyleSheet("color: darkgreen; font-weight: bold;")
        self.paScale = QtWidgets.QDoubleSpinBox(); self.paScale.setRange(1.0, 10000.0); self.paScale.setValue(180.0)
        h_unit.addWidget(self.paUseMM); h_unit.addWidget(QtWidgets.QLabel("Dist(mm) for 1000p:")); h_unit.addWidget(self.paScale)
        l.addRow(h_unit)

        self.paAcc = QtWidgets.QSpinBox(); self.paAcc.setRange(0,30000); self.paAcc.setValue(500)
        self.paSpd = QtWidgets.QSpinBox(); self.paSpd.setRange(1, 3000); self.paSpd.setValue(20); self.paSpd.setSuffix(" rpm")
        
        self.paPosL = QtWidgets.QDoubleSpinBox(); self.paPosL.setRange(-2000000000, 2000000000); self.paPosL.setDecimals(1)
        self.paPosR = QtWidgets.QDoubleSpinBox(); self.paPosR.setRange(-2000000000, 2000000000); self.paPosR.setDecimals(1)
        self.paPosL.lineEdit().returnPressed.connect(lambda: self.send_position(True))
        self.paPosR.lineEdit().returnPressed.connect(lambda: self.send_position(True))

        btnInit = QtWidgets.QPushButton("Set Absolute Mode (OK Button)")
        btnInit.setStyleSheet("background-color: #FFDDDD; font-weight: bold;")
        btnInit.clicked.connect(lambda: self.worker.queue_command(self.worker.cmd_set_mode_pos, True, self.paAcc.value(), self.paAcc.value()))
        btnZero = QtWidgets.QPushButton("Set Zero Point (Here=0)")
        btnZero.clicked.connect(lambda: self.worker.queue_command(self.worker.cmd_set_zero))
        btnGo = QtWidgets.QPushButton("Go Absolute [Enter]")
        btnGo.clicked.connect(lambda: self.send_position(True))

        l.addRow(btnInit); l.addRow(btnZero); l.addRow("Acc:", self.paAcc); l.addRow("Speed:", self.paSpd); l.addRow("GoTo L:", self.paPosL); l.addRow("GoTo R:", self.paPosR); l.addRow(btnGo)
        return w

    # --- 4. Torque ---
    def ui_torque(self):
        w = QtWidgets.QWidget(); l = QtWidgets.QFormLayout(w)
        self.tSlope = QtWidgets.QSpinBox(); self.tSlope.setValue(500)
        self.tL = QtWidgets.QSpinBox(); self.tL.setRange(-30000,30000); self.tL.setValue(500)
        self.tR = QtWidgets.QSpinBox(); self.tR.setRange(-30000,30000); self.tR.setValue(500)
        self.tL.lineEdit().returnPressed.connect(self.send_torque)
        self.tR.lineEdit().returnPressed.connect(self.send_torque)
        
        btnInit = QtWidgets.QPushButton("Set Torque Mode (OK Button)")
        btnInit.setStyleSheet("background-color: #DDFFDD; font-weight: bold;")
        btnInit.clicked.connect(lambda: self.worker.queue_command(self.worker.cmd_set_mode_torque, self.tSlope.value()))
        l.addRow(btnInit); l.addRow("Slope:", self.tSlope); l.addRow("L mA:", self.tL); l.addRow("R mA:", self.tR)
        l.addRow(QtWidgets.QLabel("* Note: Unit is mA. 500+ Recommended"))
        return w

    # --- 5. Joy & Key ---
    def ui_graphic(self):
        w = QtWidgets.QWidget(); vbox = QtWidgets.QVBoxLayout(w)
        h_set = QtWidgets.QHBoxLayout()
        h_set.addWidget(QtWidgets.QLabel("Max RPM:"))
        self.joyLimit = QtWidgets.QSpinBox(); self.joyLimit.setRange(10, 3000); self.joyLimit.setValue(100); self.joyLimit.setSingleStep(10)
        self.joyLimit.valueChanged.connect(self.update_joystick_settings)
        h_set.addWidget(self.joyLimit); h_set.addStretch(); vbox.addLayout(h_set)

        h_ctrl = QtWidgets.QHBoxLayout()
        self.joystick = VirtualJoystick(); self.keypad = KeyPadWidget()
        h_ctrl.addStretch(); h_ctrl.addWidget(self.joystick); h_ctrl.addWidget(self.keypad); h_ctrl.addStretch()
        vbox.addLayout(h_ctrl)
        
        self.update_joystick_settings()
        self.joystick.sig_speed.connect(self.on_joystick_move)
        
        btnInit = QtWidgets.QPushButton("Activate Joystick/Keyboard (Set Velocity Mode)")
        btnInit.clicked.connect(lambda: self.worker.queue_command(self.worker.cmd_set_mode_vel, 500, 500))
        vbox.addWidget(btnInit)
        lbl_hint = QtWidgets.QLabel("Tip: Click here to focus, then use Arrow Keys. Spacebar to Stop.")
        lbl_hint.setAlignment(QtCore.Qt.AlignCenter); vbox.addWidget(lbl_hint)
        return w

    # --- Helper: Apply Global Invert & Sync ---
    def get_synced_values(self, ui_l, ui_r):
        if not self.chkSync.isChecked(): return ui_l.value(), ui_r.value()
        sender = self.sender()
        if sender == ui_l.lineEdit():
            val = ui_l.value(); ui_r.setValue(val); return val, val
        elif sender == ui_r.lineEdit():
            val = ui_r.value(); ui_l.setValue(val); return val, val
        else: 
            val = ui_l.value(); ui_r.setValue(val); return val, val

    def apply_hw_invert(self, l, r):
        if self.chkInvL.isChecked(): l = -l
        if self.chkInvR.isChecked(): r = -r
        return l, r

    # --- Logic ---
    def toggle_connect(self):
        if not self.worker.is_connected: self.worker.connect_serial(self.portEdit.text(), int(self.baudCombo.currentText()), 1); self.btnConn.setText("Disconnect")
        else: self.worker.disconnect_serial(); self.btnConn.setText("Connect")

    def on_run_clicked(self): self.worker.queue_command(self.worker.cmd_enable, self.btnRun.isChecked())
    def on_stop_clicked(self): self.btnRun.setChecked(False); self.worker.queue_command(self.worker.cmd_enable, False)
    
    def on_reset_position(self):
        """현재 엔코더 위치값을 0/0으로 클리어"""
        self.worker.queue_command(self.worker.cmd_clear_feedback_pos)
        self.lblStatus.setText("✅ Encoder position cleared to 0/0")

    def send_velocity(self):
        if not self.check_run(): return
        vl, vr = self.get_synced_values(self.vL, self.vR)
        vl = -vl; vr = -vr 
        vl, vr = self.apply_hw_invert(vl, vr)
        self.worker.queue_command(self.worker.cmd_write_vel, vl, vr)

    def send_position(self, is_absolute):
        if not self.check_run(): return
        if is_absolute:
            val_l, val_r = self.get_synced_values(self.paPosL, self.paPosR)
            cnt_l, cnt_r = self.convert_input(val_l, val_r, self.paUseMM.isChecked(), self.paScale.value())
            spd = self.paSpd.value()
        else:
            val_l, val_r = self.get_synced_values(self.prPosL, self.prPosR)
            cnt_l, cnt_r = self.convert_input(val_l, val_r, self.prUseMM.isChecked(), self.prScale.value())
            spd = self.prSpd.value()
        
        cnt_l = -cnt_l; cnt_r = -cnt_r 
        cnt_l, cnt_r = self.apply_hw_invert(cnt_l, cnt_r)
        self.worker.queue_command(self.worker.cmd_write_pos_and_start, cnt_l, cnt_r, spd, spd)

    def send_torque(self):
        if not self.check_run(): return
        tl, tr = self.get_synced_values(self.tL, self.tR)
        tl = -tl; tr = -tr 
        tl, tr = self.apply_hw_invert(tl, tr)
        self.worker.queue_command(self.worker.cmd_write_torque, tl, tr)

    def update_joystick_settings(self):
        self.joystick.set_max_rpm(self.joyLimit.value())

    def on_joystick_move(self, l_rpm, r_rpm):
        if self.tabs.currentIndex() == 4 and self.btnRun.isChecked():
            l_rpm = -l_rpm; r_rpm = -r_rpm 
            final_l, final_r = self.apply_hw_invert(l_rpm, r_rpm)
            self.worker.queue_command(self.worker.cmd_write_vel, final_l, final_r)

    def keyPressEvent(self, event):
        if self.tabs.currentIndex() == 4:
            key = event.key()
            if key == QtCore.Qt.Key_Up:    self.keypad.set_key_state('up', True); self.joystick.update_keys(1,0,0,0)
            elif key == QtCore.Qt.Key_Down:  self.keypad.set_key_state('down', True); self.joystick.update_keys(0,1,0,0)
            elif key == QtCore.Qt.Key_Left:  self.keypad.set_key_state('left', True); self.joystick.update_keys(0,0,1,0)
            elif key == QtCore.Qt.Key_Right: self.keypad.set_key_state('right', True); self.joystick.update_keys(0,0,0,1)
            elif key == QtCore.Qt.Key_Space: 
                self.joystick.stop_immediate(); self.joystick.update_keys(0,0,0,0)
    
    def keyReleaseEvent(self, event):
        if self.tabs.currentIndex() == 4:
            key = event.key()
            if key == QtCore.Qt.Key_Up:    self.keypad.set_key_state('up', False)
            elif key == QtCore.Qt.Key_Down:  self.keypad.set_key_state('down', False)
            elif key == QtCore.Qt.Key_Left:  self.keypad.set_key_state('left', False)
            elif key == QtCore.Qt.Key_Right: self.keypad.set_key_state('right', False)
            k = self.keypad.keys
            self.joystick.update_keys(k['up'], k['down'], k['left'], k['right'])

    def check_run(self):
        if not self.btnRun.isChecked(): self.lblStatus.setText("⚠️ Press RUN first!"); return False
        return True

    def on_status(self, msg): self.lblStatus.setText(msg)
    def update_feedback(self, data):
        if 'vl' in data: self.lblFbVel.setText(f"Vel: {data['vl']:.1f} / {data['vr']:.1f} rpm")
        if 'tl' in data: self.lblFbTq.setText(f"Tq: {data['tl']:.1f} / {data['tr']:.1f} A")
        if 'pl' in data: self.lblFbPos.setText(f"Pos: {data['pl']} / {data['pr']}")
    def closeEvent(self, event): self.worker.disconnect_serial(); event.accept()

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())