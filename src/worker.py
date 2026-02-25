# worker.py
from PyQt5 import QtCore
from pymodbus.client import ModbusSerialClient

from .regs import REG, SerialCfg


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
        self.feedback_counter = 0  # 피드백 읽기 주기 제어

    def connect_serial(self, port, baud, slave):
        self.cfg.port = port
        self.cfg.baudrate = baud
        self.cfg.slave = slave
        self.start()

    def disconnect_serial(self):
        self.running = False
        self.wait()
        if self.client:
            self.client.close()
        self.is_connected = False
        self.sig_status.emit("Disconnected")

    def queue_command(self, func, *args, **kwargs):
        self.mutex.lock()
        self.cmd_queue.append((func, args, kwargs))
        self.sig_status.emit(f"Command queued: {func.__name__}")
        self.mutex.unlock()

    def run(self):
        self.running = True
        self.sig_status.emit(f"Connecting to {self.cfg.port} @ {self.cfg.baudrate} baud...")
        try:
            self.client = ModbusSerialClient(
                port=self.cfg.port,
                baudrate=self.cfg.baudrate,
                parity='N',
                stopbits=1,
                bytesize=8,
                timeout=0.05  # 타임아웃 단축
            )
            if self.client.connect():
                # Set slave ID for all subsequent calls
                self.client.slave_id = self.cfg.slave
                self.is_connected = True
                self.sig_status.emit(f"Connected to {self.cfg.port}")

                # Safety watchdog
                self._write(REG.COM_TIMEOUT, 500)
                self.sig_status.emit("Safety Watchdog Enabled (500ms)")
            else:
                self.sig_status.emit("Connection Failed")
                self.running = False
                return
        except Exception as e:
            self.sig_error.emit(str(e))
            self.running = False
            return

        while self.running:
            # 명령이 있으면 즉시 처리
            has_commands = self._drain_commands()
            
            # 피드백은 5회에 1번만 읽기 (50ms 주기)
            if not has_commands:
                self.feedback_counter += 1
                if self.feedback_counter >= 5:
                    self.feedback_counter = 0
                    try:
                        if self.is_connected:
                            self._read_status()
                    except Exception:
                        pass
            
            # 명령이 있으면 sleep 없이 즉시 다음 루프, 없으면 10ms 대기
            if not has_commands:
                self.msleep(10)

    def _drain_commands(self):
        has_commands = False
        self.mutex.lock()
        while self.cmd_queue:
            has_commands = True
            func, args, kwargs = self.cmd_queue.pop(0)
            self.mutex.unlock()
            try:
                func(*args, **kwargs)
            except Exception as e:
                self.sig_error.emit(f"CMD Error: {e}")
            self.mutex.lock()
        self.mutex.unlock()
        return has_commands

    def _write(self, addr, val):
        self.client.write_register(addr, int(val) & 0xFFFF)

    def _write_multi(self, addr, vals):
        self.client.write_registers(
            addr,
            [int(v) & 0xFFFF for v in vals]
        )

    def _s16(self, val):
        return val - 0x10000 if val & 0x8000 else val

    def _s32(self, high, low):
        val = (high << 16) | low
        return val - 0x100000000 if val & 0x80000000 else val

    def _read_status(self):
        fb = {}

        try:
            rr = self.client.read_holding_registers(REG.ACT_VEL_L, 2)
            if not rr.isError():
                fb['vl'] = self._s16(rr.registers[0]) / 10.0
                fb['vr'] = self._s16(rr.registers[1]) / 10.0

            rr = self.client.read_holding_registers(REG.APOS_H_L, 4)
            if not rr.isError():
                fb['pl'] = self._s32(rr.registers[0], rr.registers[1])
                fb['pr'] = self._s32(rr.registers[2], rr.registers[3])

            rr = self.client.read_holding_registers(REG.ATORQUE_L, 2)
            if not rr.isError():
                fb['tl'] = self._s16(rr.registers[0]) / 10.0
                fb['tr'] = self._s16(rr.registers[1]) / 10.0

            self.sig_feedback.emit(fb)
        except Exception:
            pass

    # --------------------
    # Commands
    # --------------------
    def cmd_enable(self, enable: bool):
        self.sig_status.emit(f"Enable command: {enable}")
        self._write(REG.CONTROL_WORD, 0x08 if enable else 0x07)
        self.sig_status.emit(f"Enable {'ON' if enable else 'OFF'} done")

    def cmd_clear_fault(self):
        self.sig_status.emit("Clearing fault...")
        self._write(REG.CONTROL_WORD, 0x06)
        self.sig_status.emit("Fault cleared")

    def _safe_change_mode(self, mode, sync_val=1):
        self._write(REG.CONTROL_WORD, 0x07); self.msleep(50)
        self._write(REG.CONTROL_WORD, 0x00); self.msleep(100)
        self._write(REG.CONTROL_MODE, mode)
        self._write(REG.SYNC_ASYNC, sync_val)
        self.msleep(100)
        self._write(REG.CONTROL_WORD, 0x08)

    def cmd_set_mode_vel(self, acc, dec):
        self.sig_status.emit(f"Setting velocity mode (acc={acc}, dec={dec})")
        self._safe_change_mode(3, 1)
        self._write(REG.ACC_L, acc); self._write(REG.ACC_R, acc)
        self._write(REG.DEC_L, dec); self._write(REG.DEC_R, dec)
        self.sig_status.emit("Velocity mode ready")

    def cmd_write_vel(self, vl, vr):
        self.sig_status.emit(f"Writing velocity: L={vl}, R={vr}")
        self._write(REG.TARGET_VEL_L, int(vl))
        self._write(REG.TARGET_VEL_R, int(vr))

    def cmd_set_mode_pos(self, absolute_mode, acc, dec):
        self._safe_change_mode(2 if absolute_mode else 1, 1)
        self._write(REG.ACC_L, acc); self._write(REG.ACC_R, acc)
        self._write(REG.DEC_L, dec); self._write(REG.DEC_R, dec)

    def cmd_write_pos_and_start(self, pl, pr, speed_l, speed_r):
        self._write(REG.TSPD_L, int(speed_l))
        self._write(REG.TSPD_R, int(speed_r))

        def split(v):
            v = int(v)
            if v >= 0:
                return ((v >> 16) & 0xFFFF, v & 0xFFFF)
            v = v + (1 << 32)
            return ((v >> 16) & 0xFFFF, v & 0xFFFF)

        l_hi, l_lo = split(pl)
        r_hi, r_lo = split(pr)
        self._write_multi(REG.TPOS_H_L, [l_hi, l_lo, r_hi, r_lo])
        self.msleep(50)
        self._write(REG.CONTROL_WORD, 0x10)

    def cmd_clear_feedback_pos(self):
        self._write(REG.CLEAR_FEEDBACK_POS, 3)

    def cmd_set_zero(self):
        self._write(REG.SET_ZERO, 3)

    def cmd_set_mode_torque(self, slope):
        self._write(REG.CONTROL_WORD, 0x07); self.msleep(50)
        self._write(REG.TTORQUE_L, 0); self._write(REG.TTORQUE_R, 0)
        self._safe_change_mode(4, 1)
        self._write(REG.TSLOPE_L, slope); self._write(REG.TSLOPE_R, slope)

    def cmd_write_torque(self, tl, tr):
        self._write(REG.TTORQUE_L, int(tl))
        self._write(REG.TTORQUE_R, int(tr))
