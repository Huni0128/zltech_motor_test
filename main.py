#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ZLAC8030D Motor Test Application
Main Entry Point
"""
import sys
from PyQt5 import QtWidgets

from src.views import MainWindow
from src.model import MotorModel
from src.controller import MainController


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    
    # MVC 구조 생성
    model = MotorModel()
    view = MainWindow()
    controller = MainController(view, model)
    
    view.show()
    sys.exit(app.exec_())
