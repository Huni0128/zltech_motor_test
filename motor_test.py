import sys
import time
from dataclasses import dataclass
from typing import Tuple

from PyQt5 import QtCore, QtWidgets
from pymodbus.client import ModbusSerialClient

# -----------------------
# ZLAC8030D Modbus Map (per RS485 manual)
# -----------------------
class REG:
    CONTROL_MODE = 0x200D           # 1:Pos(Rel), 2:Pos(Abs), 3:Vel, 4:Torque
    CONTROL_WORD = 0x200E           # 0x05 estop, 0x06 clear, 0x07 stop, 0x08 enable, 0x10/0x11/0x12 start
    SYNC_ASYNC  = 0x200F            # 0: sync, 1: async (position mode only)
    EEPROM_STORE = 0x2010           # 1 -> store RW regs to EEPROM

    # Common accel/dec fields (used in Velocity/Position modes)
    ACC_L = 0x2080
    ACC_R = 0x2081
    DEC_L = 0x2082
    DEC_R = 0x2083

    # -------- Velocity mode --------
    TARGET_VEL_L = 0x2088           # I16, rpm
    TARGET_VEL_R = 0x2089
    ACT_VEL_L = 0x20AB              # I16, 0.1 rpm
    ACT_VEL_R = 0x20AC

    # -------- Position mode --------
    TPOS_H_L = 0x208A               # I16 high, low (Left)
    TPOS_L_L = 0x208B
    TPOS_H_R = 0x208C               # (Right)
    TPOS_L_R = 0x208D
    TSPD_L = 0x208E                 # U16 1~1000 rpm (Left)
    TSPD_R = 0x208F                 # U16 1~1000 rpm (Right)
    APOS_H_L = 0x20A7               # I16 feedback position high/low (Left)
    APOS_L_L = 0x20A8
    APOS_H_R = 0x20A9               # (Right)
    APOS_L_R = 0x20AA

    # -------- Torque mode --------
    TSLOPE_L = 0x2086               # U16 torque slope mA/s
    TSLOPE_R = 0x2087
    TTORQUE_L = 0x2090              # I16 target torque mA (-30000~30000)
    TTORQUE_R = 0x2091
    ATORQUE_L = 0x20AD              # I16 actual 0.1A (-300~300)
    ATORQUE_R = 0x20AE


def i16(value: int) -> int:
    value = max(-32768, min(32767, int(value)))
    return value & 0xFFFF


def u16(value: int) -> int:
    value = max(0, min(65535, int(value)))
    return value & 0xFFFF


@dataclass
class SerialCfg:
    port: str = "COM7"
    baudrate: int = 115200
    slave: int = 1
    parity: str = "N"
    stopbits: int = 1


class ZLAC8030D:
    def __init__(self, cfg: SerialCfg):
        self.cfg = cfg
        self.client = ModbusSerialClient(
            port=cfg.port, baudrate=cfg.baudrate, bytesize=8,
            parity=cfg.parity, stopbits=cfg.stopbits, timeout=0.2, method="rtu"
        )
        self.slave = cfg.slave

    # --- transport ---
    def connect(self) -> bool:
        return self.client.connect()

    def close(self):
        self.client.close()

    # --- low-level RW ---
    def r(self, addr: int, n: int = 1):
        rr = self.client.read_holding_registers(addr, n, slave=self.slave)
        if rr.isError():
            raise RuntimeError(str(rr))
        return rr.registers if n > 1 else rr.registers[0]

    def w(self, addr: int, val: int):
        wr = self.client.write_register(addr, val & 0xFFFF, slave=self.slave)
        if wr.isError():
            raise RuntimeError(str(wr))

    def w_multi(self, addr: int, vals):
        wr = self.client.write_registers(addr, [v & 0xFFFF for v in vals], slave=self.slave)
        if wr.isError():
            raise RuntimeError(str(wr))

    # --- helpers ---
    def clear_fault(self):
        self.w(REG.CONTROL_WORD, 0x06)

    def stop(self):
        self.w(REG.CONTROL_WORD, 0x07)

    def estop(self):
        self.w(REG.CONTROL_WORD, 0x05)

    def enable(self):
        self.w(REG.CONTROL_WORD, 0x08)

    def set_mode_velocity(self, acc_ms=500, dec_ms=500):
        self.w(REG.CONTROL_MODE, 3)
        self.w(REG.ACC_L, u16(acc_ms)); self.w(REG.ACC_R, u16(acc_ms))
        self.w(REG.DEC_L, u16(dec_ms)); self.w(REG.DEC_R, u16(dec_ms))

    def set_mode_torque(self, slope_ma_per_s=500):
        self.w(REG.CONTROL_MODE, 4)
        self.w(REG.TSLOPE_L, u16(slope_ma_per_s))
        self.w(REG.TSLOPE_R, u16(slope_ma_per_s))

    def set_mode_position(self, relative=True, acc_ms=500, dec_ms=500, sync=False):
        self.w(REG.SYNC_ASYNC, 0 if sync else 1)
        self.w(REG.CONTROL_MODE, 1 if relative else 2)
        self.w(REG.ACC_L, u16(acc_ms)); self.w(REG.ACC_R, u16(acc_ms))
        self.w(REG.DEC_L, u16(dec_ms)); self.w(REG.DEC_R, u16(dec_ms))

    # ---- Velocity ----
    def set_target_velocity(self, left_rpm: int, right_rpm: int, sync=True):
        l = max(-3000, min(3000, int(left_rpm)))
        r = max(-3000, min(3000, int(right_rpm)))
        if sync:
            self.w_multi(REG.TARGET_VEL_L, [i16(l), i16(r)])
        else:
            self.w(REG.TARGET_VEL_L, i16(l)); self.w(REG.TARGET_VEL_R, i16(r))

    def read_actual_velocity(self) -> Tuple[float, float]:
        def s16(u):
            return u - 0x10000 if u & 0x8000 else u
        vl = s16(self.r(REG.ACT_VEL_L)) / 10.0
        vr = s16(self.r(REG.ACT_VEL_R)) / 10.0
        return vl, vr

    # ---- Torque ----
    def set_target_torque(self, left_ma: int, right_ma: int, sync=True):
        l = max(-30000, min(30000, int(left_ma)))
        r = max(-30000, min(30000, int(right_ma)))
        if sync:
            self.w_multi(REG.TTORQUE_L, [i16(l), i16(r)])
        else:
            self.w(REG.TTORQUE_L, i16(l)); self.w(REG.TTORQUE_R, i16(r))

    def read_actual_torque(self) -> Tuple[float, float]:
        def s16(u):
            return u - 0x10000 if u & 0x8000 else u
        tl = s16(self.r(REG.ATORQUE_L)) / 10.0  # 0.1A
        tr = s16(self.r(REG.ATORQUE_R)) / 10.0
        return tl, tr

    # ---- Position ----
    def set_target_position_pulses(self, left_counts: int, right_counts: int, sync=True):
        # counts are 32-bit signed; driver expects split into two I16 (high, low)
        def split32(val: int):
            # clamp to signed 31-bit usable range per manual
            val = max(-(1<<31)+1, min((1<<31)-1, int(val)))
            if val < 0:
                val = (1<<32) + val
            hi = (val >> 16) & 0xFFFF
            lo = val & 0xFFFF
            return hi, lo
        lhi, llo = split32(left_counts)
        rhi, rlo = split32(right_counts)
        if sync:
            # left hi/lo then right hi/lo (contiguous addresses 0x208A~0x208D)
            self.w_multi(REG.TPOS_H_L, [lhi, llo, rhi, rlo])
        else:
            self.w_multi(REG.TPOS_H_L, [lhi, llo])
            self.w_multi(REG.TPOS_H_R, [rhi, rlo])

    def set_position_target_speed(self, left_rpm: int, right_rpm: int):
        self.w(REG.TSPD_L, u16(left_rpm))
        self.w(REG.TSPD_R, u16(right_rpm))

    def start_position(self, which: str = "sync"):
        if which == "left":
            self.w(REG.CONTROL_WORD, 0x11)
        elif which == "right":
            self.w(REG.CONTROL_WORD, 0x12)
        else:
            self.w(REG.CONTROL_WORD, 0x10)  # synchronous start

    def read_actual_position(self) -> Tuple[int, int]:
        def s32_from_hi_lo(hi, lo):
            v = ((hi & 0xFFFF) << 16) | (lo & 0xFFFF)
            if v & 0x80000000:
                v -= 0x100000000
            return v
        lhi = self.r(REG.APOS_H_L)
        llo = self.r(REG.APOS_L_L)
        rhi = self.r(REG.APOS_H_R)
        rlo = self.r(REG.APOS_L_R)
        return s32_from_hi_lo(lhi, llo), s32_from_hi_lo(rhi, rlo)


# -----------------------
# GUI
# -----------------------
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ZLAC8030D PyQt Controller")
        self.resize(960, 620)

        self.cfg = SerialCfg()
        self.dev: ZLAC8030D | None = None

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)

        # Connection panel
        conn = QtWidgets.QHBoxLayout()
        layout.addLayout(conn)
        self.portEdit = QtWidgets.QLineEdit(self.cfg.port); conn.addWidget(QtWidgets.QLabel("Port")); conn.addWidget(self.portEdit)
        self.baudCombo = QtWidgets.QComboBox(); self.baudCombo.addItems(["128000","115200","57600","38400","19200","9600"]); self.baudCombo.setCurrentText(str(self.cfg.baudrate)); conn.addWidget(QtWidgets.QLabel("Baud")); conn.addWidget(self.baudCombo)
        self.slaveSpin = QtWidgets.QSpinBox(); self.slaveSpin.setRange(1,127); self.slaveSpin.setValue(self.cfg.slave); conn.addWidget(QtWidgets.QLabel("Slave")); conn.addWidget(self.slaveSpin)
        self.btnConnect = QtWidgets.QPushButton("Connect"); conn.addWidget(self.btnConnect)
        self.btnDisconnect = QtWidgets.QPushButton("Disconnect"); conn.addWidget(self.btnDisconnect)
        self.lblStatus = QtWidgets.QLabel("disconnected"); conn.addWidget(self.lblStatus)

        # Control buttons
        btns = QtWidgets.QHBoxLayout(); layout.addLayout(btns)
        self.btnEnable = QtWidgets.QPushButton("Enable")
        self.btnStop = QtWidgets.QPushButton("Stop")
        self.btnClear = QtWidgets.QPushButton("Clear Fault")
        self.btnEStop = QtWidgets.QPushButton("E-STOP")
        btns.addWidget(self.btnEnable); btns.addWidget(self.btnStop); btns.addWidget(self.btnClear); btns.addWidget(self.btnEStop)

        # Tabs
        tabs = QtWidgets.QTabWidget(); layout.addWidget(tabs, 1)
        tabs.addTab(self._build_velocity_tab(), "Velocity")
        tabs.addTab(self._build_torque_tab(), "Torque")
        tabs.addTab(self._build_position_tab(), "Position")

        # signals
        self.btnConnect.clicked.connect(self.on_connect)
        self.btnDisconnect.clicked.connect(self.on_disconnect)
        self.btnEnable.clicked.connect(lambda: self._safe(lambda: self.dev.enable()))
        self.btnStop.clicked.connect(lambda: self._safe(lambda: self.dev.stop()))
        self.btnClear.clicked.connect(lambda: self._safe(lambda: self.dev.clear_fault()))
        self.btnEStop.clicked.connect(lambda: self._safe(lambda: self.dev.estop()))

        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(200)
        self.timer.timeout.connect(self.on_poll)

    # -------- tabs --------
    def _build_velocity_tab(self):
        w = QtWidgets.QWidget(); L = QtWidgets.QGridLayout(w)
        self.velAcc = QtWidgets.QSpinBox(); self.velAcc.setRange(0,32767); self.velAcc.setValue(500)
        self.velDec = QtWidgets.QSpinBox(); self.velDec.setRange(0,32767); self.velDec.setValue(500)
        self.velL = QtWidgets.QSpinBox(); self.velL.setRange(-3000,3000);
        self.velR = QtWidgets.QSpinBox(); self.velR.setRange(-3000,3000)
        self.velSync = QtWidgets.QCheckBox("Sync write L/R"); self.velSync.setChecked(True)
        self.btnVelInit = QtWidgets.QPushButton("Init Velocity Mode")
        self.btnVelWrite = QtWidgets.QPushButton("Write Target RPM")
        self.lblVelFb = QtWidgets.QLabel("fb: (0,0) rpm")

        L.addWidget(QtWidgets.QLabel("Acc/Dec (ms)"),0,0)
        L.addWidget(self.velAcc,0,1); L.addWidget(self.velDec,0,2)
        L.addWidget(QtWidgets.QLabel("L/R target rpm"),1,0)
        L.addWidget(self.velL,1,1); L.addWidget(self.velR,1,2)
        L.addWidget(self.velSync,2,0)
        L.addWidget(self.btnVelInit,2,1); L.addWidget(self.btnVelWrite,2,2)
        L.addWidget(self.lblVelFb,3,0,1,3)

        self.btnVelInit.clicked.connect(self.on_vel_init)
        self.btnVelWrite.clicked.connect(self.on_vel_write)
        return w

    def _build_torque_tab(self):
        w = QtWidgets.QWidget(); L = QtWidgets.QGridLayout(w)
        self.tqSlope = QtWidgets.QSpinBox(); self.tqSlope.setRange(0, 60000); self.tqSlope.setValue(500)
        self.tqL = QtWidgets.QSpinBox(); self.tqL.setRange(-30000,30000)
        self.tqR = QtWidgets.QSpinBox(); self.tqR.setRange(-30000,30000)
        self.tqSync = QtWidgets.QCheckBox("Sync write L/R"); self.tqSync.setChecked(True)
        self.btnTqInit = QtWidgets.QPushButton("Init Torque Mode")
        self.btnTqWrite = QtWidgets.QPushButton("Write Target Torque (mA)")
        self.lblTqFb = QtWidgets.QLabel("fb: (0,0) A")

        L.addWidget(QtWidgets.QLabel("Slope (mA/s)"),0,0); L.addWidget(self.tqSlope,0,1)
        L.addWidget(QtWidgets.QLabel("L/R target torque (mA)"),1,0)
        L.addWidget(self.tqL,1,1); L.addWidget(self.tqR,1,2)
        L.addWidget(self.tqSync,2,0)
        L.addWidget(self.btnTqInit,2,1); L.addWidget(self.btnTqWrite,2,2)
        L.addWidget(self.lblTqFb,3,0,1,3)

        self.btnTqInit.clicked.connect(self.on_tq_init)
        self.btnTqWrite.clicked.connect(self.on_tq_write)
        return w

    def _build_position_tab(self):
        w = QtWidgets.QWidget(); L = QtWidgets.QGridLayout(w)
        # --- unit conversion inputs ---
        self.mmPerRev = QtWidgets.QDoubleSpinBox(); self.mmPerRev.setDecimals(4); self.mmPerRev.setRange(0.0001, 100000); self.mmPerRev.setValue(100.0)
        self.pulsesPerRev = QtWidgets.QSpinBox(); self.pulsesPerRev.setRange(1, 2000000); self.pulsesPerRev.setValue(4096)  # encoder_lines*quad*gear_ratio
        self.gearRatio = QtWidgets.QDoubleSpinBox(); self.gearRatio.setDecimals(4); self.gearRatio.setRange(0.0001, 10000); self.gearRatio.setValue(1.0)

        # position mode controls
        self.posRelative = QtWidgets.QCheckBox("Relative mode (unchecked = Absolute)"); self.posRelative.setChecked(True)
        self.posSync = QtWidgets.QCheckBox("Synchronous start"); self.posSync.setChecked(True)
        self.posAcc = QtWidgets.QSpinBox(); self.posAcc.setRange(0,32767); self.posAcc.setValue(500)
        self.posDec = QtWidgets.QSpinBox(); self.posDec.setRange(0,32767); self.posDec.setValue(500)
        self.posSpeedL = QtWidgets.QSpinBox(); self.posSpeedL.setRange(1,1000); self.posSpeedL.setValue(120)
        self.posSpeedR = QtWidgets.QSpinBox(); self.posSpeedR.setRange(1,1000); self.posSpeedR.setValue(120)

        # target inputs in both counts and mm
        self.posCountsL = QtWidgets.QSpinBox(); self.posCountsL.setRange(-2147483647, 2147483647)
        self.posCountsR = QtWidgets.QSpinBox(); self.posCountsR.setRange(-2147483647, 2147483647)
        self.posMmL = QtWidgets.QDoubleSpinBox(); self.posMmL.setDecimals(3); self.posMmL.setRange(-1e9, 1e9)
        self.posMmR = QtWidgets.QDoubleSpinBox(); self.posMmR.setDecimals(3); self.posMmR.setRange(-1e9, 1e9)

        self.btnPosInit = QtWidgets.QPushButton("Init Position Mode")
        self.btnPosWriteCounts = QtWidgets.QPushButton("Write Target (counts)")
        self.btnPosWriteMm = QtWidgets.QPushButton("Write Target (mm)")
        self.btnPosStartSync = QtWidgets.QPushButton("Start (Sync/Left/Right)")
        self.lblPosFb = QtWidgets.QLabel("fb: (0,0) counts | (0.0, 0.0) mm")

        # layout
        row = 0
        L.addWidget(QtWidgets.QLabel("Mechanics"), row,0); row+=1
        L.addWidget(QtWidgets.QLabel("Travel per rev (mm/rev)"), row,0); L.addWidget(self.mmPerRev, row,1)
        L.addWidget(QtWidgets.QLabel("Effective pulses per rev"), row,2); L.addWidget(self.pulsesPerRev, row,3); row+=1
        L.addWidget(QtWidgets.QLabel("Gear ratio (output/motor)"), row,0); L.addWidget(self.gearRatio, row,1); row+=1

        L.addWidget(self.posRelative, row,0,1,2); L.addWidget(self.posSync, row,2,1,2); row+=1
        L.addWidget(QtWidgets.QLabel("Acc/Dec (ms)"), row,0)
        L.addWidget(self.posAcc, row,1); L.addWidget(self.posDec, row,2); row+=1
        L.addWidget(QtWidgets.QLabel("Target speed L/R (rpm)"), row,0)
        L.addWidget(self.posSpeedL, row,1); L.addWidget(self.posSpeedR, row,2); row+=1
        L.addWidget(QtWidgets.QLabel("Target counts L/R"), row,0)
        L.addWidget(self.posCountsL, row,1); L.addWidget(self.posCountsR, row,2); row+=1
        L.addWidget(QtWidgets.QLabel("Target mm L/R"), row,0)
        L.addWidget(self.posMmL, row,1); L.addWidget(self.posMmR, row,2); row+=1
        L.addWidget(self.btnPosInit, row,0); L.addWidget(self.btnPosWriteCounts, row,1); L.addWidget(self.btnPosWriteMm, row,2); L.addWidget(self.btnPosStartSync, row,3); row+=1
        L.addWidget(self.lblPosFb, row,0,1,4)

        self.btnPosInit.clicked.connect(self.on_pos_init)
        self.btnPosWriteCounts.clicked.connect(self.on_pos_write)
        self.btnPosWriteMm.clicked.connect(self.on_pos_write_mm)
        self.btnPosStartSync.clicked.connect(self.on_pos_start)
        return w

    # -------- actions --------
    def on_connect(self):
        try:
            self.cfg = SerialCfg(port=self.portEdit.text(), baudrate=int(self.baudCombo.currentText()), slave=self.slaveSpin.value())
            self.dev = ZLAC8030D(self.cfg)
            if self.dev.connect():
                self.lblStatus.setText("connected")
                self.timer.start()
            else:
                self.lblStatus.setText("connect failed")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Connect error", str(e))

    def on_disconnect(self):
        self.timer.stop()
        if self.dev:
            self.dev.close()
        self.lblStatus.setText("disconnected")

    def on_vel_init(self):
        self._safe(lambda: (self.dev.set_mode_velocity(self.velAcc.value(), self.velDec.value()), self.dev.enable()))

    def on_vel_write(self):
        self._safe(lambda: self.dev.set_target_velocity(self.velL.value(), self.velR.value(), sync=self.velSync.isChecked()))

    def on_tq_init(self):
        self._safe(lambda: (self.dev.set_mode_torque(self.tqSlope.value()), self.dev.enable()))

    def on_tq_write(self):
        self._safe(lambda: self.dev.set_target_torque(self.tqL.value(), self.tqR.value(), sync=self.tqSync.isChecked()))

    def on_pos_init(self):
        def init():
            self.dev.set_mode_position(relative=self.posRelative.isChecked(), acc_ms=self.posAcc.value(), dec_ms=self.posDec.value(), sync=self.posSync.isChecked())
            self.dev.set_position_target_speed(self.posSpeedL.value(), self.posSpeedR.value())
            self.dev.enable()
        self._safe(init)

    def on_pos_write(self):
        self._safe(lambda: self.dev.set_target_position_pulses(self.posCountsL.value(), self.posCountsR.value(), sync=True))

    def on_pos_write_mm(self):
        # counts = mm * (pulses_per_rev * gear_ratio) / (mm_per_rev)
        mm_rev = max(1e-9, float(self.mmPerRev.value()))
        ppr = int(self.pulsesPerRev.value())
        gr = float(self.gearRatio.value())
        scale = (ppr * gr) / mm_rev
        left_counts = int(round(self.posMmL.value() * scale))
        right_counts = int(round(self.posMmR.value() * scale))
        self.posCountsL.setValue(left_counts)
        self.posCountsR.setValue(right_counts)
        self._safe(lambda: self.dev.set_target_position_pulses(left_counts, right_counts, sync=True))

    def on_pos_start(self):
        which = "sync" if self.posSync.isChecked() else "left"
        # If async and you want right, toggle which accordingly in UI; here we use sync/left for simplicity
        self._safe(lambda: self.dev.start_position(which))

    def on_poll(self):
        if not self.dev:
            return
        try:
            # velocity feedback
            try:
                vl, vr = self.dev.read_actual_velocity()
                self.lblVelFb.setText(f"fb: ({vl:.1f}, {vr:.1f}) rpm")
            except Exception:
                pass
            # torque feedback
            try:
                tl, tr = self.dev.read_actual_torque()
                self.lblTqFb.setText(f"fb: ({tl:.1f}, {tr:.1f}) A")
            except Exception:
                pass
            # position feedback (also show mm)
            try:
                pl, pr = self.dev.read_actual_position()
                mm_rev = max(1e-9, float(self.mmPerRev.value()))
                ppr = int(self.pulsesPerRev.value())
                gr = float(self.gearRatio.value())
                scale_mm = mm_rev / (ppr * gr)
                ml, mr = pl * scale_mm, pr * scale_mm
                self.lblPosFb.setText(f"fb: ({pl}, {pr}) counts | ({ml:.3f}, {mr:.3f}) mm")
            except Exception:
                pass
        except Exception as e:
            self.lblStatus.setText(f"poll err: {e}")

    def _safe(self, fn):
        if not self.dev:
            QtWidgets.QMessageBox.warning(self, "Warning", "Not connected")
            return
        try:
            fn()
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", str(e))


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow(); w.show()
    sys.exit(app.exec_())
