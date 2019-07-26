#!/usr/bin/python
from __future__ import print_function
import sys

from GUI import Ui_MainWindow

from PyQt4.QtCore import *
from PyQt4.QtGui import *

import cv2
if "../" not in sys.path:
    sys.path.append("../")

import pyrealsense2 as rs
import numpy as np

import time

import socket

import speech_recognition as sr


LOCATIONS = {
    "daniel":["Daniel_Lin", "0.00 -5.00 0.00"],
    "max":["Max_Huang", "0.00 0.40 0.00"],
    "robin":["Robin_Lin", "0.00 -3.33 0.00"]
}

ADDR = ("192.168.31.64", 6666)

class Voice(QThread):

    def __init__(self, parent=None):
        QThread.__init__(self, parent=parent)
        self.rec = False
        self.r = sr.Recognizer()

        self.keywords_EN = [
            ("find robin", 1),
            ("find daniel", 1),
            ("find max", 1),
            ("find professor", 1)
        ]

        print("[VOICE] Voice Thread Started")

        self.running = True

        self.ui_parent = parent

    def run(self):
        while self.running == True:
            if self.rec == True:
                with sr.Microphone() as source:
                    print("[INFO] Say something!")
                    audio = self.r.listen(source,phrase_time_limit=2)

                print("[INFO] Recognizing...")
                try:
                    text = self.r.recognize_sphinx(audio,language="en-US", keyword_entries=self.keywords_EN)
                    print('[RES]' + text)

                    if "daniel" in text:
                        self.toggle_rec()
                        msg = "POS " + LOCATIONS['daniel'][1] + " " + LOCATIONS['daniel'][0] 
                        self.ui_parent.tcp_send(msg)

                    elif "max" in text:
                        self.toggle_rec()
                        msg = "POS " + LOCATIONS['max'][1] + " " + LOCATIONS['max'][0]
                        self.ui_parent.tcp_send(msg)

                    elif "robin" in text:
                        self.toggle_rec()
                        msg = "POS " + LOCATIONS['robin'][1] + " " + LOCATIONS['robin'][0]
                        self.ui_parent.tcp_send(msg)

                except sr.UnknownValueError:
                    # print("Sphinx could not understand audio")
                    pass
                except sr.RequestError as e:
                    # print("Sphinx error; {0}".format(e))
                    pass
                except sr.WaitTimeoutError as e:
                    # print("timeout!")
                    pass
            else:
                time.sleep(1)
    
    def toggle_rec(self):
        if self.rec:
            self.rec = False
        else:
            self.rec = True

    def stop(self):
        self.running = False


class MainWindow(QMainWindow, Ui_MainWindow):
    
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self.onBindingUI()

        self.clientsock = socket.socket()
        self.connect()

        self.voice_th = Voice(self)
        self.voice_th.start()

    def connect(self):
        try:
            self.clientsock.connect(ADDR)
        except Exception as e:
            print(e)

    def onBindingUI(self):
        self.btn_send_pos.clicked.connect(self.on_btn_send_pos_Clicked)
        self.btn_stop.clicked.connect(self.on_btn_stop_Clicked)
        self.btn_voice_rec.clicked.connect(self.on_btn_voice_rec_Clicked)

    

    def tcp_send(self, msg):
        try:
            self.clientsock.send(msg)
            print("[SEND]", msg)
        except Exception as e:
            print(e)

        return


    def on_btn_send_pos_Clicked(self):
        print("[DEBUG] SEND POS BTN Clicked")

        x = float(self.pos_x.text())
        y = float(self.pos_y.text())
        heading = float(self.pos_h.text())

        msg = "POS %.2f %.2f %.2f" % (x, y, heading)
        self.tcp_send(msg)
        return

    def on_btn_stop_Clicked(self):
        print("[DEBUG] STOP BTN Clicked")
        msg = "STOP"
        self.tcp_send(msg)
        return
        
    def on_btn_voice_rec_Clicked(self):
        print("[DEBUG] VOICE REC BTN Clicked")
        self.voice_th.toggle_rec()

        return

    def closeEvent(self, event):
        print("[EXIT] Close socket")
        self.clientsock.close()
        self.voice_th.stop()
        print("[EXIT] Closing Voice Thread")
        time.sleep(3)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
