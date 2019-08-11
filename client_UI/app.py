#!/usr/bin/python
from __future__ import print_function
import os
import sys
from GUI import Ui_MainWindow
from PyQt4.QtCore import *
from PyQt4.QtGui import *

import cv2
if "../utils/" not in sys.path:
    sys.path.append("../utils/")

from recognize_face_imgs import recognize

import webbrowser
import pyrealsense2 as rs
import numpy as np

import roslib 
import rospy
import actionlib
import geometry_msgs
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
import tf.transformations
import time

from web_cam import web_camera

import socket
import re

ngrok_url = 'https://f7261ff4.ngrok.io/b'
# Font color
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class ros_node(QThread):

    rotate_done = pyqtSignal()
    take_photo = pyqtSignal()
    def __init__(self, parent=None):
        QThread.__init__(self, parent=parent)

        print(bcolors.HEADER + '[NODE] Starting ROS node...')
        rospy.init_node('app_client')

        self.parent = parent

        # Creates the SimpleActionClient
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

        # Rotate agv 
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.rot = Twist()
        self.rotate_rate = rospy.Rate(3)

        # send signal to other node
        self.rotate_done.connect(parent.on_btn_verify_click)
        self.take_photo.connect(parent.on_btn_takePhoto_click)
        # activate the moving action
        self.gogoagv = False
        # activate face recognizing 
        self.recognizing = True
        # lock (wait for verifying)
        self.done = False
        # only moving, no face recogition
        self.only_moving = False
    def run(self):
        while True:
            if self.gogoagv:
                if self.only_moving:
                    print(bcolors.ENDC + 'Only Moving')
                    self.activate_home()
                    self.only_moving = False
                else:
                    print(bcolors.ENDC + 'Moving and Verifying')
                    self.activate()
                self.gogoagv = False
            else:
                self.sleep(1)

    def activate(self):
        print(bcolors.ENDC + '[NODE] x=' + str(self.x) + ' y=' + str(self.y) + ' h=' + str(self.heading))

        # Waits until the action server has started up and started
        # listening for goals.
        print (bcolors.ENDC + '[NODE] Waiting for server...')
        self.move_base_client.wait_for_server()

        # Creates a goal message to be sent to the action server.
        pose = geometry_msgs.msg.Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        pose.position.z = 0.0
        if (self.heading != None) :
            q = tf.transformations.quaternion_from_euler(0, 0, self.heading)
            pose.orientation = geometry_msgs.msg.Quaternion(*q)
        goal = MoveBaseGoal()
        goal.target_pose.pose = pose
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()


        # Sends the goal to the action server.
        print(bcolors.ENDC + '[NODE] Sending goal to action server: %s' % goal)
        self.move_base_client.send_goal(goal)

        # Waits for the server to finish performing the action.
        print(bcolors.ENDC + '[NODE] Waiting for result...')
        self.move_base_client.wait_for_result()

        print(bcolors.OKBLUE + '[NODE] Result received. Action state is %s' % self.move_base_client.get_state())
        print(bcolors.OKBLUE + '[NODE] Goal status message is %s' % self.move_base_client.get_goal_status_text())
        # if AGV succeffuly went to the target
        if int(self.move_base_client.get_state()) == 3:
            self.take_photo.emit()
            self.recognizing = True
            time.sleep(1)
            print(bcolors.OKGREEN + "[NODE] Start rotate")
            rotate_counter = 0
            while self.recognizing:
                self.rotate(2)
                # to solve the racing condition with verify function
                self.done = False
                while self.done == False:
                    pass
                rotate_counter = rotate_counter + 1
                if rotate_counter >= 10:
                    self.recognizing = False
                    # send 1 (time out) back to nurse
                    self.parent.tcp_th.connect_socket.send('1')
        # Failed to find a valid plan
        elif int(self.move_base_client.get_state()) == 4:
            print(bcolors.FAIL + '[ERROR] AGV: No plan')
            # send 2 (timeout) back to nurse
            self.parent.tcp_th.connect_socket.send('2')
        else:
            print(bcolors.FAIL + '[ERROR] AGV: Action state:%s' % self.move_base_client.get_state())

    def activate_home(self):
        print(bcolors.ENDC + '[NODE] x=' + str(self.x) +
              ' y=' + str(self.y) + ' h=' + str(self.heading))

        # Waits until the action server has started up and started
        # listening for goals.
        print(bcolors.ENDC + '[NODE] Waiting for server...')
        self.move_base_client.wait_for_server()

        # Creates a goal message to be sent to the action server.
        pose = geometry_msgs.msg.Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        pose.position.z = 0.0
        if (self.heading != None):
            q = tf.transformations.quaternion_from_euler(0, 0, self.heading)
            pose.orientation = geometry_msgs.msg.Quaternion(*q)
        goal = MoveBaseGoal()
        goal.target_pose.pose = pose
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        # Sends the goal to the action server.
        print(bcolors.ENDC + '[NODE] Sending goal to action server: %s' % goal)
        self.move_base_client.send_goal(goal)

        # Waits for the server to finish performing the action.
        print(bcolors.ENDC + '[NODE] Waiting for result...')
        self.move_base_client.wait_for_result()

        print(bcolors.OKBLUE + '[NODE] Result received. Action state is %s' %
              self.move_base_client.get_state())
        print(bcolors.OKBLUE + '[NODE] Goal status message is %s' %
              self.move_base_client.get_goal_status_text())
        # if AGV succeffuly went to the target
        if int(self.move_base_client.get_state()) == 3:
            # send 0 (success) back to nurse
            self.parent.tcp_th.connect_socket.send('0')
        # Failed to find a valid plan
        elif int(self.move_base_client.get_state()) == 4:
            print(bcolors.FAIL + '[ERROR] AGV: No plan')
            # send 2 (timeout) back to nurse
            self.parent.tcp_th.connect_socket.send('2')
        else:
            print(bcolors.FAIL + '[ERROR] AGV: Action state:%s' %
                  self.move_base_client.get_state())

    def rotate(self, duration):
        
        self.rot.angular.z = 0.5

        start = time.time()
        while time.time() - start < duration:
            self.pub.publish(self.rot)
            self.rotate_rate.sleep()

        time.sleep(1)
        self.rotate_done.emit()

    def cancel(self):
        print(bcolors.WARNING + '[NODE] Cancelling Goal ...')
        self.move_base_client.cancel_goal()
        self.parent.tcp_th.connect_socket.send('3')

class TCP_server(QThread):

    reset_authority = pyqtSignal(bool)
    def __init__(self, parent=None):
        
        QThread.__init__(self, parent=parent)
        # get parent of the thread
        self.parent = parent

        self.reset_authority.connect(lambda p:parent.set_authority(p))
        hostname = '0.0.0.0' 
        port = 6667
        addr = (hostname,port)
        self.srv = socket.socket()
        self.srv.settimeout(None)
        # print('timeout:~~')
        # print(self.srv.gettimeout())
        self.srv.bind(addr)
        self.srv.listen(5)
        print(bcolors.HEADER + '[INFO] TCP Server established')
        # TCP thread is running
        self.running = True

        # regular expression format
        self.r = re.compile('POS .* .* .* .*')
        
    def run(self):
        self.connect_socket, self.client_addr = self.srv.accept()
        print(bcolors.HEADER + '[INFO] TCP connection set')
        print(bcolors.OKBLUE + 'Client:' + str(self.client_addr))
        while self.running:
            try:
                # block til recv msg
                recevent = str(self.connect_socket.recv(128))
                print(bcolors.ENDC + recevent)
                self.action(recevent)
                # self.connect_socket.send('Hi~Robin')
                if recevent == "":
                    break
            except Exception as e:
                print(bcolors.FAIL + "[ERROR] TCP:" + str(e))
                # break
        if self.running == False:
            self.connect_socket.close()
            print(bcolors.ENDC + '[INFO] TCP socket closed')
            print(bcolors.ENDC + '[INFO] TCP thread terminate')
        # if not terminated by quit btn, restart again
        else:
            self.connect_socket.close()
            print(bcolors.ENDC + '[INFO] TCP socket closed')
            print(bcolors.ENDC + '[INFO] Restart TCP thread')
            self.run()
    def action(self, recvMSG):
        # self.parent.set_authority(False)
        self.reset_authority.emit(False)
        if recvMSG == 'STOP':
            print(bcolors.WARNING + 'AGV is canceling goal')
            self.parent.ros_th.cancel()
        # I should draw distingush between the formats
        elif recvMSG == 'HOME':
            print(bcolors.OKGREEN + 'AGV is heading to Home')
            self.parent.ros_th.x = -0.2
            self.parent.ros_th.y = -0.1
            self.parent.ros_th.heading = 0.00
            self.parent.ros_th.gogoagv = True
            self.parent.ros_th.only_moving = True
            os.system("pkill chrome")
        elif self.r.match(recvMSG) is not None:
            print(bcolors.OKGREEN + 'AGV is heading to %s' % (recvMSG))
            pos = recvMSG.split(' ')
            self.parent.ros_th.x = float(pos[1])
            self.parent.ros_th.y = float(pos[2])
            self.parent.ros_th.heading = float(pos[3])
            self.parent.authorized_name = pos[4]
            self.parent.label_name.setText(pos[4])
            # activate agv
            self.parent.ros_th.gogoagv = True
        elif recvMSG == 'OPEN':
            self.parent.on_btn_openLink_click()
        else:
            print(bcolors.FAIL + '[ERROR] Unkowned message : %s' %(recvMSG))

class MainWindow(QMainWindow, Ui_MainWindow):
    
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self.onBindingUI()
        self.image = []
        self.set_authority(False)
        # Create thread for camera
        self.th = web_camera(self)
        self.th.start()
        self.camera_running = True
        # Create thread for TCP server
        self.tcp_th = TCP_server(self)
        # Create thread for ROS node
        self.ros_th = ros_node(self)  
        self.ros_th.start()
        self.tcp_th.start()

        # nurse picture showed while agv moving
        self.nurse_img = cv2.imread('nurse.jpg')
        self.nurse_img = cv2.cvtColor(self.nurse_img, cv2.COLOR_BGR2RGB)
        self.drawPicture(self.nurse_img)

        self.authorized_name = "<choose>"

    def drawPicture(self, img, cache=True):
        if cache:
            self.image = img
        height, width, bytesPerComponent = img.shape
        bytesPerLine = 3 * width
        QImg = QImage(img.data, width, height, bytesPerLine, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(QImg)
        scaledPix = pixmap.scaled(self.viewer.size(), Qt.KeepAspectRatio)
        self.viewer.setPixmap(scaledPix)

    def onBindingUI(self):
        # open camera
        # self.btn_takePhoto.clicked.connect(self.on_btn_takePhoto_click)
        # verify and authenticate
        # self.btn_verify.clicked.connect(self.on_btn_verify_click)
        # open link for video communication
        self.btn_openLink.clicked.connect(self.on_btn_openLink_click)
        # clear viewer image
        # self.btn_clear.clicked.connect(self.on_btn_clear_click)
        # capture viewer image
        # self.btn_ok.clicked.connect(self.on_btn_ok_click)
        self.btn_quit.clicked.connect(self.on_btn_quit_click)
        # go home button
        self.btn_home.clicked.connect(self.on_btn_home_click)
        # go to certain link
        self.btn_point.clicked.connect(self.on_btn_point_click)
        # cancel goal
        self.btn_cancel.clicked.connect(self.on_btn_cancel_click)

    def on_btn_takePhoto_click(self):
        print(bcolors.OKGREEN + '[INFO] Take Photo button pressed')
        self.th.refresh.connect(lambda p:self.drawPicture(p))
        
        
    def set_authority(self, authorized):
        if authorized:
            self.label_status.setText("Authorized")
            self.label_status.setStyleSheet("QLabel{color:rgb(0,255,0)}")
            self.btn_openLink.setEnabled(True)
        else:
            self.label_status.setText("Unauthorized")
            self.label_status.setStyleSheet("QLabel{color:rgb(255,0,0)}")
            self.btn_openLink.setEnabled(False)

    def verify(self, image):
        if len(image) == 0:
            print(bcolors.FAIL + '[ERRO] No image found!')
            self.set_authority(False)
            return

        image, names = recognize(image)
        
        if len(names) == 0:
            print(bcolors.WARNING + '[WARN] No person in the image')
            self.set_authority(False)

            return False
        elif len(names) > 1:
            print(bcolors.WARNING + '[WARN] Multiple people in the image')
            self.set_authority(False)

            return False
        else:
            text = self.authorized_name
            # qtext = QString(text)
            # self.comboBox.setCurrentIndex(qtext)
            # text = str(self.comboBox.currentText())
            if names[0] == 'Unknown':
                print(bcolors.WARNING + '[WARN] Unknown person')
                self.set_authority(False)
                self.drawPicture(image.copy(),cache=False)

                return False

            elif names[0] == text:
                print(bcolors.OKBLUE + '[INFO] Authorized User: '+text)
                self.set_authority(True)
                self.drawPicture(image.copy(),cache=False)

                return True
            else:
                print(bcolors.WARNING + '[WARN] Unauthorized User')
                self.set_authority(False)

                return False
    
    
    def on_btn_verify_click(self):
        print(bcolors.OKGREEN + '[INFO] verify button pressed')

        self.th.refresh.disconnect()

        ret = self.verify(self.image.copy())
        # verification failed, rotate again
        if ret == False:
            self.th.refresh.connect(lambda p:self.drawPicture(p))
        else:
            self.ros_th.recognizing = False
            # send success signal("0") back to nurse
            self.tcp_th.connect_socket.send('0')
            # after rotating, present the viewer with the nurese 
            self.drawPicture(self.nurse_img)
        
        self.ros_th.done = True

        
    def on_btn_openLink_click(self):
        print(bcolors.OKGREEN + '[INFO] Open Link button pressed')
        self.th.stop()
        self.camera_running = False
        # self.btn_takePhoto.setEnabled(False)
        webbrowser.open(ngrok_url)

    def on_btn_clear_click(self):
        self.viewer.clear()
        self.image = []

    def on_btn_ok_click(self):
        self.th.refresh.disconnect()

    def on_btn_quit_click(self):
        print(bcolors.OKGREEN + '[INFO] Quit Application')
        if self.camera_running:
            self.th.stop()
        self.tcp_th.running = False
        self.tcp_th.quit()
        self.close()

    def on_btn_home_click(self):
        self.tcp_th.action('HOME')

    def on_btn_point_click(self):
        MSG = 'POS ' + self.pos_x.text() + " " + self.pos_y.text() + " " + self.pos_h.text()
        print(bcolors.OKGREEN + "Givein AGV a point : %s" %(MSG))
        self.tcp_th.action(MSG)

    def on_btn_cancel_click(self):
        print(bcolors.OKGREEN + "AGV STOP")
        self.tcp_th.action('STOP')

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
