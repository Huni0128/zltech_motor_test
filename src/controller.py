# controller.py
import atexit
from PyQt5 import QtCore

from .model import MotorModel


class MainController(QtCore.QObject):
    def __init__(self, view, model: MotorModel):
        super().__init__()
        self.v = view
        self.m = model

        # 반복 상태
        self.vel_repeat_count = 0
        self.vel_repeat_max = 0
        self.vel_repeat_forward = True

        self.pos_repeat_count = 0
        self.pos_repeat_max = 0
        self.pos_repeat_forward = True

        self._wire_signals()
        atexit.register(self.m.emergency_stop)

    # -----------------------
    # Wiring
    # -----------------------
    def _wire_signals(self):
        # model -> view
        self.m.sig_status.connect(self._on_status)
        self.m.sig_error.connect(self._on_error)
        self.m.sig_feedback.connect(self._on_feedback)

        # view -> controller
        self.v.btnConn.clicked.connect(self.toggle_connect)
        self.v.btnRun.clicked.connect(self.on_run_clicked)
        self.v.btnStop.clicked.connect(self.on_stop_clicked)
        self.v.btnClear.clicked.connect(lambda: self.m.queue(self.m.worker.cmd_clear_fault))
        self.v.btnResetPos.clicked.connect(self.on_reset_position)

        # tab changed -> auto mode switch
        self.v.tabs.currentChanged.connect(self.on_tab_changed)

        # velocity
        self.v.vL.lineEdit().returnPressed.connect(self.send_velocity)
        self.v.vR.lineEdit().returnPressed.connect(self.send_velocity)
        self.v.btnVelRepeatStart.clicked.connect(self.start_vel_repeat)
        self.v.btnVelRepeatStop.clicked.connect(self.stop_vel_repeat)
        self.v.vel_repeat_timer.timeout.connect(self.vel_repeat_step)

        # relative pos
        self.v.btnRelGo.clicked.connect(lambda: self.send_position(False))
        self.v.prPosL.lineEdit().returnPressed.connect(lambda: self.send_position(False))
        self.v.prPosR.lineEdit().returnPressed.connect(lambda: self.send_position(False))
        self.v.btnPosRepeatStart.clicked.connect(self.start_pos_repeat)
        self.v.btnPosRepeatStop.clicked.connect(self.stop_pos_repeat)
        self.v.pos_repeat_timer.timeout.connect(self.pos_repeat_step)

        # joystick
        self.v.joyLimit.valueChanged.connect(self.update_joystick_settings)
        self.v.joystick.sig_speed.connect(self.on_joystick_move)
        self.update_joystick_settings()

        # 키보드 이벤트(메인윈도우 이벤트 필터로 받기)
        self.v.installEventFilter(self)

    # -----------------------
    # Event Filter (키 처리)
    # -----------------------
    def eventFilter(self, obj, event):
        if obj is self.v and self.v.tabs.currentIndex() == 2:
            if event.type() == QtCore.QEvent.KeyPress:
                self._on_key_press(event)
            elif event.type() == QtCore.QEvent.KeyRelease:
                self._on_key_release(event)
        return super().eventFilter(obj, event)

    def _on_key_press(self, event):
        key = event.key()
        if key == QtCore.Qt.Key_Up:
            self.v.keypad.set_key_state('up', True); self.v.joystick.update_keys(1, 0, 0, 0)
        elif key == QtCore.Qt.Key_Down:
            self.v.keypad.set_key_state('down', True); self.v.joystick.update_keys(0, 1, 0, 0)
        elif key == QtCore.Qt.Key_Left:
            self.v.keypad.set_key_state('left', True); self.v.joystick.update_keys(0, 0, 1, 0)
        elif key == QtCore.Qt.Key_Right:
            self.v.keypad.set_key_state('right', True); self.v.joystick.update_keys(0, 0, 0, 1)
        elif key == QtCore.Qt.Key_Space:
            self.v.joystick.stop_immediate()
            self.v.joystick.update_keys(0, 0, 0, 0)

    def _on_key_release(self, event):
        key = event.key()
        if key == QtCore.Qt.Key_Up:
            self.v.keypad.set_key_state('up', False)
        elif key == QtCore.Qt.Key_Down:
            self.v.keypad.set_key_state('down', False)
        elif key == QtCore.Qt.Key_Left:
            self.v.keypad.set_key_state('left', False)
        elif key == QtCore.Qt.Key_Right:
            self.v.keypad.set_key_state('right', False)

        k = self.v.keypad.keys
        self.v.joystick.update_keys(k['up'], k['down'], k['left'], k['right'])

    # -----------------------
    # Helpers
    # -----------------------
    def check_run(self):
        if not self.v.btnRun.isChecked():
            self.v.lblStatus.setText("⚠️ Press RUN first!")
            return False
        return True

    def convert_input(self, val_l, val_r, use_mm, scale):
        if use_mm:
            ratio = 1000.0 / scale
            return int(val_l * ratio), int(val_r * ratio)
        return int(val_l), int(val_r)

    def get_synced_values(self, ui_l, ui_r):
        if not self.v.chkSync.isChecked():
            return ui_l.value(), ui_r.value()
        sender = self.v.sender()
        if sender == ui_l.lineEdit():
            val = ui_l.value()
            ui_r.setValue(val)
            return val, val
        if sender == ui_r.lineEdit():
            val = ui_r.value()
            ui_l.setValue(val)
            return val, val
        val = ui_l.value()
        ui_r.setValue(val)
        return val, val

    def apply_hw_invert(self, l, r):
        if self.v.chkInvL.isChecked():
            l = -l
        if self.v.chkInvR.isChecked():
            r = -r
        return l, r

    # -----------------------
    # Tab Auto Mode Switch
    # -----------------------
    def on_tab_changed(self, index):
        """탭 변경 시 자동으로 해당 모드로 전환"""
        if not self.m.is_connected:
            return
        
        if index == 0:  # Velocity
            self.init_velocity_mode()
            self.v.lblStatus.setText("🚀 Velocity Mode Activated")
        elif index == 1:  # Relative Pos
            self.init_relative_mode()
            self.v.lblStatus.setText("📍 Relative Position Mode Activated")
        elif index == 2:  # Joystick
            self.init_velocity_mode()
            self.v.lblStatus.setText("🎮 Joystick Mode Activated (Velocity)")

    # -----------------------
    # Connection / Run
    # -----------------------
    def toggle_connect(self):
        if not self.m.is_connected:
            port = self.v.portEdit.text()
            baud = int(self.v.baudCombo.currentText())
            self.m.connect_serial(port, baud, 1)
            self.v.btnConn.setText("Disconnect")
            # 연결 후 현재 탭에 맞는 모드로 초기화 (약간의 지연 후)
            QtCore.QTimer.singleShot(500, lambda: self.on_tab_changed(self.v.tabs.currentIndex()))
        else:
            self.m.disconnect_serial()
            self.v.btnConn.setText("Connect")

    def on_run_clicked(self):
        self.m.queue(self.m.worker.cmd_enable, self.v.btnRun.isChecked())

    def on_stop_clicked(self):
        self.v.btnRun.setChecked(False)
        self.m.queue(self.m.worker.cmd_enable, False)

    def on_reset_position(self):
        self.m.queue(self.m.worker.cmd_clear_feedback_pos)
        self.v.lblStatus.setText("Encoder position cleared to 0/0")

    # -----------------------
    # Mode init
    # -----------------------
    def init_velocity_mode(self):
        self.m.queue(self.m.worker.cmd_set_mode_vel, self.v.vAcc.value(), self.v.vAcc.value())

    def init_relative_mode(self):
        self.m.queue(self.m.worker.cmd_set_mode_pos, False, self.v.prAcc.value(), self.v.prAcc.value())

    def init_absolute_mode(self):
        self.m.queue(self.m.worker.cmd_set_mode_pos, True, self.v.paAcc.value(), self.v.paAcc.value())

    def init_torque_mode(self):
        self.m.queue(self.m.worker.cmd_set_mode_torque, self.v.tSlope.value())

    # -----------------------
    # Send commands
    # -----------------------
    def send_velocity(self):
        if not self.check_run():
            return
        vl, vr = self.get_synced_values(self.v.vL, self.v.vR)

        # 원본의 방향 반전 유지
        vl = -vl
        vr = -vr
        vl, vr = self.apply_hw_invert(vl, vr)
        self.m.queue(self.m.worker.cmd_write_vel, vl, vr)

    def send_position(self, is_absolute):
        if not self.check_run():
            return

        if is_absolute:
            val_l, val_r = self.get_synced_values(self.v.paPosL, self.v.paPosR)
            cnt_l, cnt_r = self.convert_input(val_l, val_r, self.v.paUseMM.isChecked(), self.v.paScale.value())
            spd = self.v.paSpd.value()
        else:
            val_l, val_r = self.get_synced_values(self.v.prPosL, self.v.prPosR)
            cnt_l, cnt_r = self.convert_input(val_l, val_r, self.v.prUseMM.isChecked(), self.v.prScale.value())
            spd = self.v.prSpd.value()

        # 원본의 방향 반전 유지
        cnt_l = -cnt_l
        cnt_r = -cnt_r
        cnt_l, cnt_r = self.apply_hw_invert(cnt_l, cnt_r)
        self.m.queue(self.m.worker.cmd_write_pos_and_start, cnt_l, cnt_r, spd, spd)

    def send_torque(self):
        if not self.check_run():
            return
        tl, tr = self.get_synced_values(self.v.tL, self.v.tR)

        # 원본의 방향 반전 유지
        tl = -tl
        tr = -tr
        tl, tr = self.apply_hw_invert(tl, tr)
        self.m.queue(self.m.worker.cmd_write_torque, tl, tr)

    # -----------------------
    # Joystick
    # -----------------------
    def update_joystick_settings(self):
        self.v.joystick.set_max_rpm(self.v.joyLimit.value())

    def on_joystick_move(self, l_rpm, r_rpm):
        if self.v.tabs.currentIndex() == 2 and self.v.btnRun.isChecked():
            l_rpm = -l_rpm
            r_rpm = -r_rpm
            final_l, final_r = self.apply_hw_invert(l_rpm, r_rpm)
            self.m.queue(self.m.worker.cmd_write_vel, final_l, final_r)

    # -----------------------
    # Repeat: Velocity
    # -----------------------
    def start_vel_repeat(self):
        if not self.check_run():
            return
        self.vel_repeat_count = 0
        self.vel_repeat_max = self.v.vRepeatCount.value()
        self.vel_repeat_forward = True

        self.v.btnVelRepeatStart.setEnabled(False)
        self.v.btnVelRepeatStop.setEnabled(True)
        self.v.lblVelRepeatStatus.setText(f"Running: 0/{self.vel_repeat_max}")
        self.v.lblVelRepeatStatus.setStyleSheet("color: green; font-weight: bold;")
        self.vel_repeat_step()

    def stop_vel_repeat(self):
        self.v.vel_repeat_timer.stop()
        self.m.queue(self.m.worker.cmd_write_vel, 0, 0)
        self.v.btnVelRepeatStart.setEnabled(True)
        self.v.btnVelRepeatStop.setEnabled(False)
        self.v.lblVelRepeatStatus.setText("Stopped")
        self.v.lblVelRepeatStatus.setStyleSheet("color: red; font-weight: bold;")

    def vel_repeat_step(self):
        if self.vel_repeat_count >= self.vel_repeat_max:
            self.stop_vel_repeat()
            self.v.lblVelRepeatStatus.setText("Completed")
            self.v.lblVelRepeatStatus.setStyleSheet("color: blue; font-weight: bold;")
            return

        vl, vr = self.v.vL.value(), self.v.vR.value()
        if not self.vel_repeat_forward:
            vl, vr = -vl, -vr

        vl = -vl
        vr = -vr
        vl, vr = self.apply_hw_invert(vl, vr)
        self.m.queue(self.m.worker.cmd_write_vel, vl, vr)

        if not self.vel_repeat_forward:
            self.vel_repeat_count += 1
            self.v.lblVelRepeatStatus.setText(f"Running: {self.vel_repeat_count}/{self.vel_repeat_max}")

        self.vel_repeat_forward = not self.vel_repeat_forward
        self.v.vel_repeat_timer.start(int(self.v.vRepeatTime.value() * 1000))

    # -----------------------
    # Repeat: Position(상대)
    # -----------------------
    def start_pos_repeat(self):
        if not self.check_run():
            return
        self.pos_repeat_count = 0
        self.pos_repeat_max = self.v.prRepeatCount.value()
        self.pos_repeat_forward = True

        self.v.btnPosRepeatStart.setEnabled(False)
        self.v.btnPosRepeatStop.setEnabled(True)
        self.v.lblPosRepeatStatus.setText(f"Running: 0/{self.pos_repeat_max}")
        self.v.lblPosRepeatStatus.setStyleSheet("color: green; font-weight: bold;")
        self.pos_repeat_step()

    def stop_pos_repeat(self):
        self.v.pos_repeat_timer.stop()
        self.v.btnPosRepeatStart.setEnabled(True)
        self.v.btnPosRepeatStop.setEnabled(False)
        self.v.lblPosRepeatStatus.setText("Stopped")
        self.v.lblPosRepeatStatus.setStyleSheet("color: red; font-weight: bold;")

    def pos_repeat_step(self):
        if self.pos_repeat_count >= self.pos_repeat_max:
            self.stop_pos_repeat()
            self.v.lblPosRepeatStatus.setText("Completed")
            self.v.lblPosRepeatStatus.setStyleSheet("color: blue; font-weight: bold;")
            return

        val_l, val_r = self.v.prPosL.value(), self.v.prPosR.value()
        if not self.pos_repeat_forward:
            val_l, val_r = -val_l, -val_r

        cnt_l, cnt_r = self.convert_input(val_l, val_r, self.v.prUseMM.isChecked(), self.v.prScale.value())
        spd = self.v.prSpd.value()

        cnt_l = -cnt_l
        cnt_r = -cnt_r
        cnt_l, cnt_r = self.apply_hw_invert(cnt_l, cnt_r)
        self.m.queue(self.m.worker.cmd_write_pos_and_start, cnt_l, cnt_r, spd, spd)

        if not self.pos_repeat_forward:
            self.pos_repeat_count += 1
            self.v.lblPosRepeatStatus.setText(f"Running: {self.pos_repeat_count}/{self.pos_repeat_max}")

        self.pos_repeat_forward = not self.pos_repeat_forward

        # Position 이동 시간 추정
        max_cnt = max(abs(cnt_l), abs(cnt_r))
        if spd > 0:
            estimated_time = (max_cnt / 1000.0) / spd * 60.0 + 1.0
        else:
            estimated_time = 3.0
        self.v.pos_repeat_timer.start(int(estimated_time * 1000))

    # -----------------------
    # Callbacks from Model
    # -----------------------
    def _on_status(self, msg):
        self.v.lblStatus.setText(msg)

    def _on_error(self, msg):
        self.v.lblStatus.setText(f"❌ Error: {msg}")

    def _on_feedback(self, data):
        # Feedback labels
        if 'vl' in data and 'vr' in data:
            self.v.lblFbVel.setText(f"Vel: {data['vl']:.1f} / {data['vr']:.1f} rpm")
        if 'tl' in data and 'tr' in data:
            self.v.lblFbTq.setText(f"Tq: {data['tl']:.1f} / {data['tr']:.1f} A")
        if 'pl' in data and 'pr' in data:
            self.v.lblFbPos.setText(f"Pos: {data['pl']} / {data['pr']}")
