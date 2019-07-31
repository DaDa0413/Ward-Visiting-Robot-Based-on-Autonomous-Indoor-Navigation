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

ADDR = ("192.168.31.64", 6668)

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
        self.parent = parent
        self.go_home = False

    def run(self):
        while self.running == True:
            if self.go_home == True:
                self.go_home = False
                msg = "HOME"
                self.parent.tcp_send(msg)
                self.wait_receive()
                
            if self.rec == True:
                with sr.Microphone() as source:
                    self.r.adjust_for_ambient_noise(source)
                    self.parent.print_label("Say something!")
                    try:
                        audio = self.r.listen(source,phrase_time_limit=2, timeout=2)
                        self.parent.print_label("Recognizing...")
                        try:
                            text = self.r.recognize_sphinx(audio,language="en-US", keyword_entries=self.keywords_EN)
                            
                            if "daniel" in text:
                                self.parent.print_label("Finding Daniel...")
                                self.toggle_rec()
                                msg = "POS " + LOCATIONS['daniel'][1] + " " + LOCATIONS['daniel'][0] 
                                self.parent.tcp_send(msg)
                                self.wait_receive()

                            elif "max" in text:
                                self.parent.print_label("Finding Max...")
                                self.toggle_rec()
                                msg = "POS " + LOCATIONS['max'][1] + " " + LOCATIONS['max'][0]
                                self.parent.tcp_send(msg)
                                self.wait_receive()

                            elif "robin" in text:
                                self.parent.print_label("Finding Robin...")
                                self.toggle_rec()
                                msg = "POS " + LOCATIONS['robin'][1] + " " + LOCATIONS['robin'][0]
                                self.parent.tcp_send(msg)
                                self.wait_receive()

                        except sr.UnknownValueError:
                            self.parent.print_label("Try Again!")
                            time.sleep(1)
                        except sr.RequestError:
                            # print("Sphinx error; {0}".format(e))
                            pass
                    except sr.WaitTimeoutError:
                        self.parent.print_label("Try Again!")
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
        self.wait()

    def home(self):
        self.go_home = True
        self.parent.print_label("Going Home...")

    def wait_receive(self):
        data = self.parent.clientsock.recv(64)
        """
        0: success
        1: timeout
        2: no plan
        3: stopped
        """
        if data == "0":
            self.parent.print_label("Success!")
        elif data == "1":
            self.parent.print_label("Cannot find person!")
        elif data == "2":
            self.parent.print_label("No Plan!")
        elif data == "3":
            self.parent.print_label("Stop by master!")
        else:
            self.parent.print_label(str(data))
        


class MainWindow(QMainWindow, Ui_MainWindow):
    
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self.setGeometry(300,300, self.frameGeometry().width(), self.frameGeometry().height())
        self.onBindingUI()

        self.clientsock = socket.socket()
        self.connect()

        self.voice_th = Voice(parent=self)
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
        self.btn_home.clicked.connect(self.on_btn_home_Clicked)

    def tcp_send(self, msg):
        try:
            self.clientsock.send(msg)
            print("[SEND]", msg)
        except Exception as e:
            print(e)

        return

    def on_btn_home_Clicked(self):
        print("[DEBUG] HOME BTN Clicked")
        self.voice_th.home()
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
        # print("[DEBUG] VOICE REC BTN Clicked")
        # 1. show message
        self.print_label("Starting...")

        # 2. toggle thread
        self.voice_th.toggle_rec()

        return

    def closeEvent(self, event):
        print("[EXIT] Close socket")
        self.clientsock.close()
        self.voice_th.stop()
        print("[EXIT] Voice Thread Closed")
    
    def print_label(self, msg):
        self.message.setText(msg)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
