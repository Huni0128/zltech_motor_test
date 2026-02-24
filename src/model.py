# model.py
from PyQt5 import QtCore
from .worker import MotorWorker


class MotorModel(QtCore.QObject):
    sig_status = QtCore.pyqtSignal(str)
    sig_error = QtCore.pyqtSignal(str)
    sig_feedback = QtCore.pyqtSignal(dict)  # 원본 fb dict 그대로

    def __init__(self):
        super().__init__()
        self.worker = MotorWorker()

        self.worker.sig_status.connect(self.sig_status)
        self.worker.sig_error.connect(self.sig_error)
        self.worker.sig_feedback.connect(self._on_feedback)

        # 최신 상태(원하면 더 확장 가능)
        self.fb = {'vl': 0.0, 'vr': 0.0, 'pl': 0, 'pr': 0, 'tl': 0.0, 'tr': 0.0}

    @property
    def is_connected(self):
        return self.worker.is_connected

    def connect_serial(self, port, baud, slave=1):
        self.worker.connect_serial(port, baud, slave)

    def disconnect_serial(self):
        self.worker.disconnect_serial()

    def queue(self, func, *args, **kwargs):
        self.worker.queue_command(func, *args, **kwargs)

    def emergency_stop(self):
        if self.worker.is_connected:
            try:
                self.worker.cmd_enable(False)
            except Exception:
                pass

    def _on_feedback(self, fb):
        self.fb.update(fb)
        self.sig_feedback.emit(self.fb)
