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

# URL to the app deployed on heroku
# It will create a P2P connection to 'A'
ngrok_url = 'https://agv-webrtc.herokuapp.com/b'
# TCP/IP socket address and port
addr = ('0.0.0.0', 6666)

# Font color displayed in terminal
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

# Create a node called "app_client"
# Send a Action to topic "/move_base"
# Tell it where the AGV will go
# And wait for the result
class ros_node(QThread):
    rotate_done_sn = pyqtSignal()
    take_photo_sn = pyqtSignal()
    def __init__(self, parent=None):
        QThread.__init__(self, parent=parent)
        self.parent = parent

        print(bcolors.HEADER + '[NODE] Starting ROS node...')
        rospy.init_node('app_client')

        # Creates the SimpleActionClient
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

        # Rotate publisher 
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Send signal to other node
        # take_photo_sn : open camera for face recognition
        # rotate_done_sn : After each rotation, start face recognition once
        self.take_photo_sn.connect(parent.on_btn_takePhoto_click)
        self.rotate_done_sn.connect(parent.on_btn_verify_click)

        # enable agv to move
        self.enable_agv = False
        # activate face recognition
        self.enable_recognition = True
        # Mutex lock (wait for verification)
        self.verification_done = False
        # only moving, no face recogition
        self.moving_home = False
    def run(self):
        while True:
            if self.enable_agv:
                if self.moving_home:
                    print(bcolors.ENDC + 'Moving Home(No Verification')
                    self.activate_home()
                    self.moving_home = False
                else:
                    print(bcolors.ENDC + 'Moving to Find Someone')
                    self.activate()
                self.enable_agv = False
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
            # Open camera and try to find target person
            self.take_photo_sn.emit()
            self.enable_recognition = True
            # Wait until camera opened
            time.sleep(1)
            print(bcolors.OKGREEN + "[NODE] Start rotate")
            rotate_counter = 0
            while self.enable_recognition:
                self.rotate(2)
                # To solve the racing condition with verify function
                self.verification_done = False
                while self.verification_done == False:
                    pass
                # No rotate more than 10 times
                rotate_counter = rotate_counter + 1
                if rotate_counter >= 10:
                    self.enable_recognition = False
                    # send 1 (time out) back to nurse
                    self.parent.tcp_th.connect_socket.send('1')
        # Failed to find a valid plan
        elif int(self.move_base_client.get_state()) == 4:
            print(bcolors.FAIL + '[ERROR] AGV: No plan')
            # send 2 (timeout) back to nurse
            self.parent.tcp_th.connect_socket.send('2')
        else:
            pass
            
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

        print(bcolors.OKBLUE + '[NODE] Result received. Action state is %s' % self.move_base_client.get_state())
        print(bcolors.OKBLUE + '[NODE] Goal status message is %s' % self.move_base_client.get_goal_status_text())
        # if AGV succeffuly went to the target
        if int(self.move_base_client.get_state()) == 3:
            # send 4 (success) back to nurse
            # but ask nurse not to open video communication
            self.parent.tcp_th.connect_socket.send('4')
        # Failed to find a valid plan
        elif int(self.move_base_client.get_state()) == 4:
            print(bcolors.FAIL + '[ERROR] AGV: No plan')
            # send 2 (timeout) back to nurse
            self.parent.tcp_th.connect_socket.send('2')
        else:
            pass

    # Rotate a specific interval
    # And then start face recognition
    def rotate(self, duration):
        rot = Twist()
        rotate_rate = rospy.Rate(3)
        rot.angular.z = 0.5

        start = time.time()
        while time.time() - start < duration:
            self.pub.publish(rot)
            rotate_rate.sleep()

        time.sleep(1)
        self.rotate_done_sn.emit()

    def cancel(self):
        print(bcolors.WARNING + '[NODE] Cancelling Goal ...')
        self.move_base_client.cancel_goal()
        # send 3 (cancel) back to nurse
        self.parent.tcp_th.connect_socket.send('3')

class TCP_server(QThread):

    reset_authority_sn = pyqtSignal(bool)
    set_webcam_sn = pyqtSignal()
    set_pic_sn = pyqtSignal(np.ndarray)
    def __init__(self, parent=None):
        
        QThread.__init__(self, parent=parent)
        # get parent of the thread
        self.parent = parent

        # Send signal to other node
        # reset_authority_sn : set authentication presented on UI to be false
        # set_webcam_sn : create a webcam thread and start it(set it running)
        # set_pic_sn : present the viewer in UI to be nurse.jpg
        self.reset_authority_sn.connect(lambda p:parent.set_authority(p))
        self.set_webcam_sn.connect(self.parent.set_webcam)
        self.set_pic_sn.connect(lambda p:self.parent.drawPicture(p))
        self.srv = socket.socket()
        self.srv.settimeout(None)
        self.srv.bind(addr)
        self.srv.listen(5)
        print(bcolors.HEADER + '[INFO] TCP Server established' + bcolors.ENDC)
        # TCP thread is running
        # It is always true til quit buttom being press
        self.running = True

        # regular expression format
        # Message from nurse has to be "POS FLOAT FLOAT FLOAT NAME"
        self.r = re.compile('POS .* .* .* .*')
        
    def run(self):
        self.connect_socket, self.client_addr = self.srv.accept()
        print(bcolors.HEADER + '[INFO] TCP connection set' + bcolors.ENDC)
        print(bcolors.OKBLUE + 'Client:' + str(self.client_addr) + bcolors.ENDC)
        while self.running:
            try:
                # it will be blocked til recv msg
                recevent = str(self.connect_socket.recv(128))
                print(bcolors.ENDC + recevent)
                self.action(recevent)
                if recevent == "":
                    break
            except Exception as e:
                print(bcolors.FAIL + "[ERROR] TCP:" + str(e) + bcolors.ENDC)
        if self.running == False:
            self.connect_socket.close()
            print(bcolors.ENDC + '[INFO] TCP socket closed')
            print(bcolors.ENDC + '[INFO] TCP thread terminate')
        # if not terminated by quit btn, restart again
        else:
            self.connect_socket.close()
            print(bcolors.ENDC + '[INFO] TCP socket closed')
            print(bcolors.ENDC + '[INFO] Restart TCP thread' + bcolors.ENDC)
            self.run()
    def action(self, recvMSG):
        if recvMSG == 'STOP':
            print(bcolors.WARNING + 'AGV is canceling goal' + bcolors.ENDC)
            self.parent.ros_th.cancel()
        elif recvMSG == 'HOME':
            self.reset_authority_sn.emit(False)
            self.set_pic_sn.emit(self.parent.nurse_img)
            print(bcolors.OKGREEN + 'AGV is heading to Home' + bcolors.ENDC)
            # Home coordinate
            self.parent.ros_th.x = -0.2
            self.parent.ros_th.y = -0.1
            self.parent.ros_th.heading = 0.00
            self.parent.ros_th.enable_agv = True
            self.parent.ros_th.moving_home = True
            self.parent.label_name.setText("Name")
            os.system("pkill chrome")
        elif self.r.match(recvMSG) is not None:
            self.reset_authority_sn.emit(False)
            self.set_webcam_sn.emit()
            print(bcolors.OKGREEN + 'AGV is heading to %s' % (recvMSG))
            pos = recvMSG.split(' ')
            self.parent.ros_th.x = float(pos[1])
            self.parent.ros_th.y = float(pos[2])
            self.parent.ros_th.heading = float(pos[3])
            self.parent.authorized_name = pos[4]
            self.parent.label_name.setText(pos[4])
            # activate agv
            self.parent.ros_th.enable_agv = True
            # open video communication
        elif recvMSG == 'OPEN':
            time.sleep(5)
            self.parent.label_name.setText("Name")
            self.parent.on_btn_openLink_click()
        else:
            print(bcolors.FAIL + '[ERROR] Unkowned message : %s' %(recvMSG) + bcolors.ENDC)

class MainWindow(QMainWindow, Ui_MainWindow):
    
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self.onBindingUI()
        self.image = []
        self.set_authority(False)
        # camera is opened upon arrival
        self.camera_running = False
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
        self.btn_quit.clicked.connect(self.on_btn_quit_click)
        
    def on_btn_takePhoto_click(self):
        print(bcolors.OKGREEN + '[INFO] Take Photo button pressed' + bcolors.ENDC)
        self.webcam_th.refresh_sn.connect(lambda p:self.drawPicture(p))
        
        
    def set_authority(self, authorized):
        if authorized:
            self.label_status.setText("Authorized")
            self.label_status.setStyleSheet("QLabel{color:rgb(0,255,0)}")
            # self.btn_openLink.setEnabled(True)
        else:
            self.label_status.setText("Unauthorized")
            self.label_status.setStyleSheet("QLabel{color:rgb(255,0,0)}")
            # self.btn_openLink.setEnabled(False)

    # Verify the image sent
    # If the person in it is Daniel, Robin or Max,
    # Return True
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
            # text is which person we are finding
            text = self.authorized_name
            if names[0] == 'Unknown':
                print(bcolors.WARNING + '[WARN] Unknown person' + bcolors.ENDC)
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

        # To send just one picture,
        # We have to disconnect the camera
        # And reserve the camera for video communication
        self.webcam_th.refresh_sn.disconnect()

        ret = self.verify(self.image.copy())
        # Verification failed, rotate again
        # And trying to find once
        if ret == False:
            self.webcam_th.refresh_sn.connect(lambda p:self.drawPicture(p))
        else:
            self.ros_th.enable_recognition = False
            # send success signal("0") back to nurse
            self.tcp_th.connect_socket.send('0')
            self.webcam_th.stop()
        # Tell ros_th "Verification is done, you can continue to next step"        
        self.ros_th.verification_done = True
        
    def on_btn_openLink_click(self):
        print(bcolors.OKGREEN + '[INFO] Open Link button pressed')
        self.webcam_th.stop()
        self.camera_running = False
        # self.btn_takePhoto.setEnabled(False)
        webbrowser.open(ngrok_url)

    def on_btn_quit_click(self):
        print(bcolors.OKGREEN + '[INFO] Quit Application')
        if self.camera_running:
            self.webcam_th.stop()
        self.tcp_th.running = False
        self.tcp_th.quit()
        self.close()

    def on_btn_cancel_click(self):
        print(bcolors.OKGREEN + "AGV STOP")
        self.tcp_th.action('STOP')

    # Create a webcam thread
    # Make it to be running
    def set_webcam(self):
        self.webcam_th = web_camera(self)
        self.webcam_th.start()
        self.camera_running = True

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
