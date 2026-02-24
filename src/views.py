# views.py
import math
from PyQt5 import QtCore, QtWidgets, QtGui


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
        layout = {'up': (50, 5, 40, 40), 'left': (5, 50, 40, 40), 'down': (50, 50, 40, 40), 'right': (95, 50, 40, 40)}
        painter.setPen(QtGui.QPen(QtCore.Qt.black, 2))
        painter.setFont(QtGui.QFont("Arial", 12, QtGui.QFont.Bold))

        for key, (x, y, w, h) in layout.items():
            rect = QtCore.QRectF(x, y, w, h)
            color = QtGui.QColor(0, 150, 255) if self.keys[key] else QtGui.QColor(220, 220, 220)
            painter.setBrush(color)
            painter.drawRoundedRect(rect, 5, 5)
            symbol = {'up': '▲', 'down': '▼', 'left': '◀', 'right': '▶'}[key]
            painter.setPen(QtCore.Qt.white if self.keys[key] else QtCore.Qt.black)
            painter.drawText(rect, QtCore.Qt.AlignCenter, symbol)


class VirtualJoystick(QtWidgets.QWidget):
    sig_speed = QtCore.pyqtSignal(int, int)

    def __init__(self):
        super().__init__()
        self.setMinimumSize(200, 200)
        self.moving = False
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.key_x = 0.0
        self.key_y = 0.0
        self.max_rpm = 50
        self.snap_threshold = 0.2
        self.current_l = 0.0
        self.current_r = 0.0
        self.ramp_step = 50.0  # 빠른 응답을 위해 증가
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.emit_speed)
        self.timer.start(30)  # 30ms로 단축하여 즉시 반응

    def set_max_rpm(self, val):
        self.max_rpm = val

    def update_keys(self, u, d, l, r):
        self.key_y = 0.0
        self.key_x = 0.0
        if u:
            self.key_y += 1.0
        if d:
            self.key_y -= 1.0
        if r:
            self.key_x += 1.0
        if l:
            self.key_x -= 1.0
        self.key_x = max(-1.0, min(1.0, self.key_x))
        self.key_y = max(-1.0, min(1.0, self.key_y))

    def stop_immediate(self):
        self.current_l = 0
        self.current_r = 0
        self.sig_speed.emit(0, 0)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        w, h = self.width(), self.height()
        center = QtCore.QPointF(w / 2, h / 2)
        radius = min(w, h) / 2 - 10

        painter.setBrush(QtGui.QColor(50, 50, 50))
        painter.drawEllipse(center, radius, radius)
        painter.setPen(QtGui.QPen(QtCore.Qt.gray, 1, QtCore.Qt.DashLine))
        painter.drawLine(center.x() - radius, center.y(), center.x() + radius, center.y())
        painter.drawLine(center.x(), center.y() - radius, center.x(), center.y() + radius)

        curr_x = self.pos_x if self.moving else self.key_x
        curr_y = self.pos_y if self.moving else self.key_y
        if abs(curr_x) < self.snap_threshold:
            curr_x = 0
        if abs(curr_y) < self.snap_threshold:
            curr_y = 0

        draw_y = -curr_y
        handle_center = QtCore.QPointF(center.x() + curr_x * radius, center.y() + draw_y * radius)
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor(50, 200, 50) if (curr_x == 0 or curr_y == 0) else QtGui.QColor(200, 50, 50))
        painter.drawEllipse(handle_center, 30, 30)

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.moving = True
            self.update_position(event.pos())

    def mouseMoveEvent(self, event):
        if self.moving:
            self.update_position(event.pos())

    def mouseReleaseEvent(self, event):
        self.moving = False
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.update()

    def update_position(self, mouse_pos):
        w, h = self.width(), self.height()
        center_x, center_y = w / 2, h / 2
        radius = min(w, h) / 2 - 10
        dx = mouse_pos.x() - center_x
        dy = mouse_pos.y() - center_y
        dist = math.sqrt(dx * dx + dy * dy)
        if dist > radius:
            ratio = radius / dist
            dx *= ratio
            dy *= ratio
        self.pos_x = dx / radius
        self.pos_y = -dy / radius
        self.update()

    def emit_speed(self):
        raw_x = self.pos_x if self.moving else self.key_x
        raw_y = self.pos_y if self.moving else self.key_y

        if abs(raw_x) < self.snap_threshold:
            raw_x = 0
        if abs(raw_y) < self.snap_threshold:
            raw_y = 0

        if not self.moving and raw_x == 0 and raw_y == 0:
            if abs(self.current_l) < 1 and abs(self.current_r) < 1:
                self.current_l = 0
                self.current_r = 0
                self.sig_speed.emit(0, 0)
                self.update()
                return

        throttle = raw_y * self.max_rpm
        turn = raw_x * self.max_rpm * 0.5
        target_l = throttle + turn
        target_r = throttle - turn

        def ramp(curr, target, step):
            if curr < target:
                curr += step
                if curr > target:
                    curr = target
            elif curr > target:
                curr -= step
                if curr < target:
                    curr = target
            return curr

        self.current_l = ramp(self.current_l, target_l, self.ramp_step)
        self.current_r = ramp(self.current_r, target_r, self.ramp_step)
        self.sig_speed.emit(int(self.current_l), int(self.current_r))
        self.update()


class MainWindow(QtWidgets.QMainWindow):
    """View는 'UI 생성 + 사용자 입력 시그널'까지만 담당.
    실제 로직 호출은 Controller가 한다.
    """

    def __init__(self):
        super().__init__()
        self.setWindowTitle("ZLAC8030D Pro Controller V18 (MVC)")
        self.resize(950, 850)
        self.setFocusPolicy(QtCore.Qt.StrongFocus)

        # ---- 반복 관련(Controller가 쓸 수 있게 유지) ----
        self.vel_repeat_timer = QtCore.QTimer()
        self.pos_repeat_timer = QtCore.QTimer()

        # ---- UI 구성 ----
        self._build_ui()

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)

        # 1) Connection
        h = QtWidgets.QHBoxLayout()
        self.portEdit = QtWidgets.QLineEdit("COM3")
        self.baudCombo = QtWidgets.QComboBox()
        self.baudCombo.addItems(["115200", "57600", "38400", "9600"])
        self.btnConn = QtWidgets.QPushButton("Connect")
        h.addWidget(QtWidgets.QLabel("Port")); h.addWidget(self.portEdit)
        h.addWidget(QtWidgets.QLabel("Baud")); h.addWidget(self.baudCombo)
        h.addWidget(self.btnConn)
        layout.addLayout(h)

        # 2) RUN / STOP
        h_run = QtWidgets.QHBoxLayout()
        self.btnRun = QtWidgets.QPushButton("RUN (Enable)")
        self.btnRun.setMinimumHeight(50)
        self.btnRun.setCheckable(True)
        self.btnRun.setStyleSheet("QPushButton:checked { background-color: #00FF00; font-weight: bold; }")
        self.btnStop = QtWidgets.QPushButton("STOP (Disable)")
        self.btnStop.setMinimumHeight(50)
        self.btnStop.setStyleSheet("background-color: #FF5555; font-weight: bold; color: white;")
        self.btnClear = QtWidgets.QPushButton("Clear Alarm")
        self.btnClear.setMinimumHeight(50)
        h_run.addWidget(self.btnRun); h_run.addWidget(self.btnStop); h_run.addWidget(self.btnClear)
        layout.addLayout(h_run)

        # 3) Global Settings
        grp_glob = QtWidgets.QGroupBox("Global Control Settings")
        layout.addWidget(grp_glob)
        h_glob = QtWidgets.QHBoxLayout(grp_glob)
        self.chkSync = QtWidgets.QCheckBox("Sync (Input L -> R follows inverted)")
        self.chkSync.setChecked(True)
        self.chkSync.setStyleSheet("font-weight: bold; color: blue;")
        self.chkInvL = QtWidgets.QCheckBox("H/W Invert L")
        self.chkInvR = QtWidgets.QCheckBox("H/W Invert R")
        self.chkInvR.setChecked(True)
        h_glob.addWidget(self.chkSync); h_glob.addStretch(); h_glob.addWidget(self.chkInvL); h_glob.addWidget(self.chkInvR)

        # 4) Tabs
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.addTab(self._ui_velocity(), "1. Velocity")
        self.tabs.addTab(self._ui_position_rel(), "2. Relative Pos")
        self.tabs.addTab(self._ui_graphic(), "3. Joy & Key")
        layout.addWidget(self.tabs)

        fb_layout = QtWidgets.QHBoxLayout()
        self.lblFbVel = QtWidgets.QLabel("Vel: 0.0 / 0.0 rpm")
        self.lblFbPos = QtWidgets.QLabel("Pos: 0 / 0 cnt")
        self.lblFbTq = QtWidgets.QLabel("Tq: 0.0 / 0.0 A")
        font = self.lblFbVel.font()
        font.setPointSize(10)
        font.setBold(True)
        self.lblFbVel.setFont(font); self.lblFbPos.setFont(font); self.lblFbTq.setFont(font)
        fb_layout.addWidget(self.lblFbVel); fb_layout.addWidget(self.lblFbPos); fb_layout.addWidget(self.lblFbTq)
        layout.addLayout(fb_layout)

        h_reset = QtWidgets.QHBoxLayout()
        self.btnResetPos = QtWidgets.QPushButton("🔄 Reset Position (Set 0/0)")
        self.btnResetPos.setMinimumHeight(40)
        self.btnResetPos.setStyleSheet("background-color: #FFD700; font-weight: bold; color: black;")
        h_reset.addStretch()
        h_reset.addWidget(self.btnResetPos)
        h_reset.addStretch()
        layout.addLayout(h_reset)

        self.lblStatus = QtWidgets.QLabel("Disconnected")
        layout.addWidget(self.lblStatus)

    # ---- 탭 UI들(컨트롤러가 연결할 위젯 핸들만 생성) ----
    def _ui_velocity(self):
        w = QtWidgets.QWidget()
        l = QtWidgets.QFormLayout(w)

        self.vAcc = QtWidgets.QSpinBox(); self.vAcc.setRange(0, 30000); self.vAcc.setValue(500)
        self.vL = QtWidgets.QSpinBox(); self.vL.setRange(-3000, 3000)
        self.vR = QtWidgets.QSpinBox(); self.vR.setRange(-3000, 3000)

        self.btnVelGo = QtWidgets.QPushButton("Go Velocity [Enter]")

        l.addRow(QtWidgets.QLabel("Velocity Mode (Auto-activated)"))
        l.addRow("Acc(ms):", self.vAcc)
        l.addRow("Target L:", self.vL)
        l.addRow("Target R:", self.vR)
        l.addRow(self.btnVelGo)

        l.addRow(QtWidgets.QLabel("===== Repeat Mode ====="))
        self.vRepeatCount = QtWidgets.QSpinBox(); self.vRepeatCount.setRange(1, 1000); self.vRepeatCount.setValue(5); self.vRepeatCount.setSuffix(" times")
        self.vRepeatTime = QtWidgets.QDoubleSpinBox(); self.vRepeatTime.setRange(0.1, 60.0); self.vRepeatTime.setValue(2.0); self.vRepeatTime.setSuffix(" sec")
        l.addRow("Repeat Count:", self.vRepeatCount)
        l.addRow("Time per move:", self.vRepeatTime)

        h_repeat = QtWidgets.QHBoxLayout()
        self.btnVelRepeatStart = QtWidgets.QPushButton("▶ Start Repeat")
        self.btnVelRepeatStart.setStyleSheet("background-color: #90EE90; font-weight: bold;")
        self.btnVelRepeatStop = QtWidgets.QPushButton("⏹ Stop Repeat")
        self.btnVelRepeatStop.setStyleSheet("background-color: #FFB6C1; font-weight: bold;")
        self.btnVelRepeatStop.setEnabled(False)
        h_repeat.addWidget(self.btnVelRepeatStart); h_repeat.addWidget(self.btnVelRepeatStop)
        l.addRow(h_repeat)

        self.lblVelRepeatStatus = QtWidgets.QLabel("Ready")
        self.lblVelRepeatStatus.setStyleSheet("color: gray; font-weight: bold;")
        l.addRow("Status:", self.lblVelRepeatStatus)
        return w

    def _ui_position_rel(self):
        w = QtWidgets.QWidget()
        l = QtWidgets.QFormLayout(w)

        h_unit = QtWidgets.QHBoxLayout()
        self.prUseMM = QtWidgets.QCheckBox("Use MM Unit")
        self.prUseMM.setStyleSheet("color: darkgreen; font-weight: bold;")
        self.prScale = QtWidgets.QDoubleSpinBox(); self.prScale.setRange(1.0, 10000.0); self.prScale.setValue(180.0)
        h_unit.addWidget(self.prUseMM)
        h_unit.addWidget(QtWidgets.QLabel("Dist(mm) for 1000p:"))
        h_unit.addWidget(self.prScale)
        l.addRow(h_unit)

        self.prAcc = QtWidgets.QSpinBox(); self.prAcc.setRange(0, 30000); self.prAcc.setValue(500)
        self.prSpd = QtWidgets.QSpinBox(); self.prSpd.setRange(1, 3000); self.prSpd.setValue(20); self.prSpd.setSuffix(" rpm")

        self.prPosL = QtWidgets.QDoubleSpinBox(); self.prPosL.setRange(-2000000000, 2000000000); self.prPosL.setDecimals(1)
        self.prPosR = QtWidgets.QDoubleSpinBox(); self.prPosR.setRange(-2000000000, 2000000000); self.prPosR.setDecimals(1)

        self.btnRelGo = QtWidgets.QPushButton("Go Relative [Enter]")

        l.addRow(QtWidgets.QLabel("Relative Position Mode (Auto-activated)"))
        l.addRow("Acc:", self.prAcc)
        l.addRow("Speed:", self.prSpd)
        l.addRow("Move L:", self.prPosL)
        l.addRow("Move R:", self.prPosR)
        l.addRow(self.btnRelGo)

        l.addRow(QtWidgets.QLabel("===== Repeat Mode ====="))
        self.prRepeatCount = QtWidgets.QSpinBox(); self.prRepeatCount.setRange(1, 1000); self.prRepeatCount.setValue(5); self.prRepeatCount.setSuffix(" times")
        l.addRow("Repeat Count:", self.prRepeatCount)

        h_repeat = QtWidgets.QHBoxLayout()
        self.btnPosRepeatStart = QtWidgets.QPushButton("▶ Start Repeat")
        self.btnPosRepeatStart.setStyleSheet("background-color: #90EE90; font-weight: bold;")
        self.btnPosRepeatStop = QtWidgets.QPushButton("⏹ Stop Repeat")
        self.btnPosRepeatStop.setStyleSheet("background-color: #FFB6C1; font-weight: bold;")
        self.btnPosRepeatStop.setEnabled(False)
        h_repeat.addWidget(self.btnPosRepeatStart); h_repeat.addWidget(self.btnPosRepeatStop)
        l.addRow(h_repeat)

        self.lblPosRepeatStatus = QtWidgets.QLabel("Ready")
        self.lblPosRepeatStatus.setStyleSheet("color: gray; font-weight: bold;")
        l.addRow("Status:", self.lblPosRepeatStatus)
        return w

    def _ui_graphic(self):
        w = QtWidgets.QWidget()
        vbox = QtWidgets.QVBoxLayout(w)

        h_set = QtWidgets.QHBoxLayout()
        h_set.addWidget(QtWidgets.QLabel("Max RPM:"))
        self.joyLimit = QtWidgets.QSpinBox()
        self.joyLimit.setRange(10, 3000)
        self.joyLimit.setValue(10)
        self.joyLimit.setSingleStep(10)
        h_set.addWidget(self.joyLimit)
        h_set.addStretch()
        vbox.addLayout(h_set)

        h_ctrl = QtWidgets.QHBoxLayout()
        self.joystick = VirtualJoystick()
        self.keypad = KeyPadWidget()
        h_ctrl.addStretch()
        h_ctrl.addWidget(self.joystick)
        h_ctrl.addWidget(self.keypad)
        h_ctrl.addStretch()
        vbox.addLayout(h_ctrl)

        lbl_mode = QtWidgets.QLabel("Joystick Mode (Auto-activated)")
        lbl_mode.setAlignment(QtCore.Qt.AlignCenter)
        lbl_mode.setStyleSheet("font-weight: bold; color: #0066CC;")
        vbox.addWidget(lbl_mode)

        lbl_hint = QtWidgets.QLabel("Tip: Click here to focus, then use Arrow Keys. Spacebar to Stop.")
        lbl_hint.setAlignment(QtCore.Qt.AlignCenter)
        vbox.addWidget(lbl_hint)
        return w
