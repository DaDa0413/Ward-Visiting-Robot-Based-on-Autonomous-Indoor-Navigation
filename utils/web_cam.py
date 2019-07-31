import cv2
import numpy as np
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import time
class web_camera(QThread):

    refresh = pyqtSignal(np.ndarray)

    def __init__(self, parent=None):
        QThread.__init__(self, parent=parent)
        self.cap = cv2.VideoCapture(0)
        # '\033[95m' : color of header
        print('\033[95m' + '[INFO] Camera started')
        self.running = True
    
    def run(self):
        while self.running:
            ret, frame = self.cap.read()
            self.image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.refresh.emit(self.image)
    
    def stop(self):
        self.running = False
        time.sleep(1)
        print('\033[0m' + '[INFO] Terminating Camera')
        self.cap.release()
