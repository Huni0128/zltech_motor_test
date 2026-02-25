# controller.py
import atexit
import csv
from datetime import datetime
from pathlib import Path
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
        self.pos_repeat_target_l = 0  # 목표 위치 추적
        self.pos_repeat_target_r = 0
        self.pos_repeat_checking = False  # 위치 도달 확인 중
        
        # CSV 로깅
        self.csv_file = None
        self.csv_writer = None
        self.log_folder = Path("logs")
        self.log_folder.mkdir(exist_ok=True)

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
        self.v.btnVelGo.clicked.connect(self.send_velocity)
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
        # pos_repeat_timer는 동적으로 연결/해제 (encoder 기반)

        # rotate
        self.v.btnRotateGo.clicked.connect(self.send_rotation)
        self.v.rotateAngle.lineEdit().returnPressed.connect(self.send_rotation)

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
            self.v.lblStatus.setText("Press RUN first!")
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
            self.v.lblStatus.setText("Velocity Mode Activated")
        elif index == 1:  # Relative Pos
            self.init_relative_mode()
            self.v.lblStatus.setText("Relative Position Mode Activated")
        elif index == 2:  # Rotate
            self.init_velocity_mode()
            self.v.lblStatus.setText("Rotate Mode Activated (Velocity)")
        elif index == 3:  # Joystick
            self.init_velocity_mode()
            self.v.lblStatus.setText("Joystick Mode Activated (Velocity)")

    # -----------------------
    # Connection / Run
    # -----------------------
    def toggle_connect(self):
        if not self.m.is_connected:
            port = self.v.portEdit.text()
            baud = int(self.v.baudCombo.currentText())
            self.v.logOutput.appendPlainText(f"[USER] Connecting to {port} @ {baud}...")
            self.m.connect_serial(port, baud, 1)
            self.v.btnConn.setText("Disconnect")
            # 연결 후 현재 탭에 맞는 모드로 초기화 (약간의 지연 후)
            QtCore.QTimer.singleShot(500, lambda: self.on_tab_changed(self.v.tabs.currentIndex()))
        else:
            self.v.logOutput.appendPlainText("[USER] Disconnecting...")
            self.m.disconnect_serial()
            self.v.btnConn.setText("Connect")

    def on_run_clicked(self):
        self.v.logOutput.appendPlainText(f"[USER] RUN button: {'ON' if self.v.btnRun.isChecked() else 'OFF'}")
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
        self.v.lblStatus.setText(f"Sending velocity: L={vl}, R={vr}")
        self.v.logOutput.appendPlainText(f"[USER] Send Velocity: L={vl}, R={vr}")
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

    def send_rotation(self):
        """제자리 회전 (relative position 사용)"""
        if not self.check_run():
            return
        
        angle = self.v.rotateAngle.value()  # deg
        speed = self.v.rotateSpeed.value()  # rpm
        
        if angle == 0:
            self.m.sig_status.emit("Rotation angle is zero")
            return
        
        # 회전 거리 계산: arc_length = |angle| * π/180 * wheelbase / 2
        import math
        wheelbase = self.v.rotateWheelbase.value()  # mm
        arc_length = abs(angle) * math.pi / 180.0 * wheelbase / 2.0  # mm
        
        # pulse로 변환
        scale = self.v.prScale.value()  # mm당 1000 pulse 기준
        pulse_distance = int(arc_length * 1000.0 / scale)
        
        # 양수 각도 = 시계방향 = 왼쪽 뒤로(-), 오른쪽 앞으로(+)
        # 음수 각도 = 반시계방향 = 왼쪽 앞으로(+), 오른쪽 뒤로(-)
        if angle > 0:
            cnt_l = -pulse_distance
            cnt_r = pulse_distance
        else:
            cnt_l = pulse_distance
            cnt_r = -pulse_distance
        
        # 하드웨어 반전 적용
        cnt_l, cnt_r = self.apply_hw_invert(cnt_l, cnt_r)
        
        # Relative Position 모드로 전환 후 이동
        self.m.queue(self.m.worker.cmd_set_mode_pos, False, self.v.prAcc.value(), self.v.prAcc.value())
        self.m.queue(self.m.worker.cmd_write_pos_and_start, cnt_l, cnt_r, speed, speed)
        
        self.m.sig_status.emit(f"Rotating {angle}° at {speed} rpm (L:{cnt_l}, R:{cnt_r} pulses)")

    # -----------------------
    # CSV 로깅
    # -----------------------
    def start_csv_logging(self, mode_name):
        """CSV 로깅 시작"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = self.log_folder / f"{mode_name}_repeat_{timestamp}.csv"
        self.csv_file = open(filename, 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        # 헤더 작성
        self.csv_writer.writerow([
            'Timestamp', 'Event', 'RepeatCount',
            'PosL_cnt', 'PosR_cnt', 'PosL_mm', 'PosR_mm',
            'VelL_rpm', 'VelR_rpm', 'TorqueL_A', 'TorqueR_A'
        ])
        self.csv_file.flush()
        self.v.lblStatus.setText(f"Logging to: {filename.name}")
    
    def log_csv_data(self, event_type, repeat_count=0):
        """CSV에 데이터 기록"""
        if self.csv_writer is None:
            return
        
        fb = self.m.fb
        scale = self.v.prScale.value()
        pos_l_mm = fb.get('pl', 0) * scale / 1000.0
        pos_r_mm = fb.get('pr', 0) * scale / 1000.0
        
        self.csv_writer.writerow([
            datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
            event_type,
            repeat_count,
            fb.get('pl', 0),
            fb.get('pr', 0),
            f"{pos_l_mm:.2f}",
            f"{pos_r_mm:.2f}",
            f"{fb.get('vl', 0):.1f}",
            f"{fb.get('vr', 0):.1f}",
            f"{fb.get('tl', 0):.1f}",
            f"{fb.get('tr', 0):.1f}"
        ])
        self.csv_file.flush()
    
    def stop_csv_logging(self):
        """CSV 로깅 종료"""
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None

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
        
        # CSV 로깅 시작
        self.start_csv_logging("velocity")
        self.log_csv_data("START", 0)
        
        self.vel_repeat_step()

    def stop_vel_repeat(self):
        self.v.vel_repeat_timer.stop()
        self.m.queue(self.m.worker.cmd_write_vel, 0, 0)
        self.v.btnVelRepeatStart.setEnabled(True)
        self.v.btnVelRepeatStop.setEnabled(False)
        self.v.lblVelRepeatStatus.setText("Stopped")
        self.v.lblVelRepeatStatus.setStyleSheet("color: red; font-weight: bold;")
        
        # CSV 로깅 종료
        self.log_csv_data("STOPPED", self.vel_repeat_count)
        self.stop_csv_logging()

    def vel_repeat_step(self):
        if self.vel_repeat_count >= self.vel_repeat_max:
            self.stop_vel_repeat()
            self.v.lblVelRepeatStatus.setText("Completed")
            self.v.lblVelRepeatStatus.setStyleSheet("color: blue; font-weight: bold;")
            self.log_csv_data("COMPLETED", self.vel_repeat_count)
            self.stop_csv_logging()
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
        
        # CSV 로깅 시작
        self.start_csv_logging("position")
        self.log_csv_data("START", 0)
        
        self.pos_repeat_step()

    def stop_pos_repeat(self):
        self.v.pos_repeat_timer.stop()
        try:
            self.v.pos_repeat_timer.timeout.disconnect()
        except:
            pass
        self.pos_repeat_checking = False
        self.v.btnPosRepeatStart.setEnabled(True)
        self.v.btnPosRepeatStop.setEnabled(False)
        self.v.lblPosRepeatStatus.setText("Stopped")
        self.v.lblPosRepeatStatus.setStyleSheet("color: red; font-weight: bold;")
        
        # CSV 로깅 종료
        self.log_csv_data("STOPPED", self.pos_repeat_count)
        self.stop_csv_logging()

    def pos_repeat_step(self):
        # 이전 체크 상태 초기화 (타임아웃으로 인한 재진입 대응)
        self.pos_repeat_checking = False
        
        if self.pos_repeat_count >= self.pos_repeat_max:
            self.v.lblPosRepeatStatus.setText("Completed")
            self.v.lblPosRepeatStatus.setStyleSheet("color: blue; font-weight: bold;")
            self.log_csv_data("COMPLETED", self.pos_repeat_count)
            self.stop_pos_repeat()
            return

        val_l, val_r = self.v.prPosL.value(), self.v.prPosR.value()
        if not self.pos_repeat_forward:
            val_l, val_r = -val_l, -val_r

        cnt_l, cnt_r = self.convert_input(val_l, val_r, self.v.prUseMM.isChecked(), self.v.prScale.value())
        spd = self.v.prSpd.value()

        cnt_l = -cnt_l
        cnt_r = -cnt_r
        cnt_l, cnt_r = self.apply_hw_invert(cnt_l, cnt_r)
        
        # 목표 위치 저장 (절대 위치 계산)
        current_pos_l = self.m.fb.get('pl', 0)
        current_pos_r = self.m.fb.get('pr', 0)
        self.pos_repeat_target_l = current_pos_l + cnt_l
        self.pos_repeat_target_r = current_pos_r + cnt_r
        
        # 위치 도달 확인 활성화 (이제 encoder로만 확인)
        self.pos_repeat_checking = True
        
        # 명령 전송 시점 로깅
        direction = "FORWARD" if self.pos_repeat_forward else "BACKWARD"
        self.log_csv_data(f"MOVE_{direction}", self.pos_repeat_count)
        
        self.m.queue(self.m.worker.cmd_write_pos_and_start, cnt_l, cnt_r, spd, spd)

        self.pos_repeat_forward = not self.pos_repeat_forward
        
        # 타이머는 안전장치로만 사용 (10초 타임아웃)
        self.v.pos_repeat_timer.stop()
        try:
            self.v.pos_repeat_timer.timeout.disconnect()
        except:
            pass
        self.v.pos_repeat_timer.timeout.connect(self._pos_repeat_timeout)
        self.v.pos_repeat_timer.start(10000)  # 10초 타임아웃

    def _pos_repeat_timeout(self):
        """Position repeat 타임아웃 (안전장치)"""
        if self.pos_repeat_checking:
            self.log_csv_data("TIMEOUT_ERROR", self.pos_repeat_count)
            self.v.lblStatus.setText("⚠️ Position timeout! Check connection.")
            self.stop_pos_repeat()

    # -----------------------
    # Callbacks from Model
    # -----------------------
    def _on_status(self, msg):
        self.v.lblStatus.setText(msg)
        self.v.logOutput.appendPlainText(f"[STATUS] {msg}")

    def _on_error(self, msg):
        self.v.lblStatus.setText(f"Error: {msg}")
        self.v.logOutput.appendPlainText(f"[ERROR] {msg}")

    def _on_feedback(self, data):
        # Feedback labels
        if 'vl' in data and 'vr' in data:
            self.v.lblFbVel.setText(f"Vel: {data['vl']:.1f} / {data['vr']:.1f} rpm")
        if 'tl' in data and 'tr' in data:
            self.v.lblFbTq.setText(f"Tq: {data['tl']:.1f} / {data['tr']:.1f} A")
        if 'pl' in data and 'pr' in data:
            self.v.lblFbPos.setText(f"Pos: {data['pl']} / {data['pr']} cnt")
            
            # mm로 환산하여 표시
            scale = self.v.prScale.value()  # 1000펄스당 mm
            mm_l = data['pl'] * scale / 1000.0
            mm_r = data['pr'] * scale / 1000.0
            self.v.lblFbPosMM.setText(f"({mm_l:.1f} / {mm_r:.1f} mm)")
            
            # Position Repeat 모드에서 위치 도달 확인
            if self.pos_repeat_checking:
                # ±2mm 오차로 tolerance 계산
                scale = self.v.prScale.value()  # 1000펄스당 mm
                tolerance_mm = 2.0  # ±2mm
                tolerance_pulse = int(tolerance_mm * 1000.0 / scale)
                
                error_l = abs(data['pl'] - self.pos_repeat_target_l)
                error_r = abs(data['pr'] - self.pos_repeat_target_r)
                
                if error_l <= tolerance_pulse and error_r <= tolerance_pulse:
                    # 목표 위치 정확히 도달!
                    self.pos_repeat_checking = False
                    self.v.pos_repeat_timer.stop()  # 타임아웃 타이머 정지
                    
                    # 카운트 업데이트 (후진 완료 시에만)
                    if self.pos_repeat_forward:  # 방금 후진 완료
                        self.pos_repeat_count += 1
                        self.v.lblPosRepeatStatus.setText(f"Running: {self.pos_repeat_count}/{self.pos_repeat_max}")
                    
                    # 다음 스텝 실행 (짧은 딜레이)
                    QtCore.QTimer.singleShot(50, self.pos_repeat_step)
