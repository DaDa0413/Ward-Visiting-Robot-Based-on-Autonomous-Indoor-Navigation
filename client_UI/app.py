#!/usr/bin/python
from __future__ import print_function
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
class ros_node(QThread):

    rotate_done = pyqtSignal()

    def __init__(self, parent=None):
        QThread.__init__(self, parent=parent)

        print('[NODE] Starting ROS node...')
        rospy.init_node('app_client')

        self.parent = parent

        # Creates the SimpleActionClient
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

        # Rotate agv 
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.rot = Twist()
        self.rotate_rate = rospy.Rate(3)

        self.rotate_done.connect(parent.on_btn_verify_click)
        self.gogoagv = False
        self.recognizing = True
        self.done = False

    def run(self):
        while True:
            if self.gogoagv:
                self.activate()
                self.gogoagv = False
            else:
                self.sleep(1)

    def activate(self):

        print('[NODE] x=' + str(self.x) + ' y=' + str(self.y) + ' h=' + str(self.heading))
        # self.x = x
        # self.y = y
        # self.heading = heading

        # Waits until the action server has started up and started
        # listening for goals.
        print ('[NODE] Waiting for server...')
        self.move_base_client.wait_for_server()

        # Creates a goal to send to the action server.
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
        print('[NODE] Sending goal to action server: %s' % goal)
        self.move_base_client.send_goal(goal)

        canceltime = None

        if canceltime != None:
            print('[NODE] Letting action server work for 3 seconds but then cancelling...')
            time.sleep(canceltime)
            print('[NODE] Cancelling current goal...')
            self.move_base_client.cancel_goal()
        else:
            # Waits for the server to finish performing the action.
            print('[NODE] Waiting for result...')
            self.move_base_client.wait_for_result()

        print('[NODE] Result received. Action state is %s' % self.move_base_client.get_state())
        print('[NODE] Goal status message is %s' % self.move_base_client.get_goal_status_text())
        
        # if succeffuly go to the target
        if int(self.move_base_client.get_state()) == 3:
            while self.recognizing:
                self.rotate(2)
                self.done = False
                while self.done == False:
                    pass
        # Failed to find a valid plan
        elif int(self.move_base_client.get_state()) == 4:
            print('[ERROR] No plan')
        #return move_base_client.get_result() 

    def rotate(self, duration):
        print("[NODE] Start rotate")
        
        self.rot.angular.z = 0.5

        start = time.time()
        while time.time() - start < duration:
            self.pub.publish(self.rot)
            self.rotate_rate.sleep()

        time.sleep(1)
        self.rotate_done.emit()

    def cancel(self):
        print('[NODE] Cancelling Goal ...')
        self.move_base_client.cancel_goal()

class TCP_server(QThread):

    def __init__(self, parent=None):
        
        QThread.__init__(self, parent=parent)
        # get parent of the thread
        self.parent = parent

        hostname = '0.0.0.0' 
        port = 6671
        addr = (hostname,port)
        self.srv = socket.socket() 
        self.srv.bind(addr)
        self.srv.listen(5)
        print('[INFO] TCP Server established')
        self.running = True

    def run(self):
        self.connect_socket, self.client_addr = self.srv.accept()
        print('[INFO] TCP connection set')
        while self.running:
            print('Client:' + str(self.client_addr))
            try:
                recevent = str(self.connect_socket.recv(1024))
                print(recevent)
                self.action(recevent)
                if recevent == "":
                    break
            except Exception as e:
                print(e)
                break
        print('[INFO] TCP thread terminate')
        self.connect_socket.close()
        print('[INFO] TCP socket closed')
    def action(self, recvMSG):
        if recvMSG == 'STOP':
            print('AGV is canceling goal')
            self.parent.ros_th.cancel()
        else:
            print('AGV is heading to %s' % (recvMSG))
            pos = recvMSG.split(' ')
            self.parent.ros_th.x = float(pos[1])
            self.parent.ros_th.y = float(pos[2])
            self.parent.ros_th.heading = float(pos[3])
            self.parent.ros_th.gogoagv = True

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
        self.btn_takePhoto.clicked.connect(self.on_btn_takePhoto_click)
        # verify and authenticate
        self.btn_verify.clicked.connect(self.on_btn_verify_click)
        # open link for video communication
        self.btn_openLink.clicked.connect(self.on_btn_openLink_click)
        # clear viewer image
        self.btn_clear.clicked.connect(self.on_btn_clear_click)
        # capture viewer image
        self.btn_ok.clicked.connect(self.on_btn_ok_click)
        self.btn_quit.clicked.connect(self.on_btn_quit_click)
        # go home button
        self.btn_home.clicked.connect(self.on_btn_home_click)
        # go to certain link
        self.btn_point.clicked.connect(self.on_btn_point_click)
        # cancel goal
        self.btn_cancel.clicked.connect(self.on_btn_cancel_click)

    def on_btn_takePhoto_click(self):
        print('[INFO] Take Photo button pressed')
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
            print('[ERRO] No image found!')
            self.set_authority(False)
            return

        image, names = recognize(image)
        
        if len(names) == 0:
            print('[WARN] No person in the image')
            self.set_authority(False)

            return False
        elif len(names) > 1:
            print('[WARN] Multiple people in the image')
            self.set_authority(False)

            return False
        else:
            text = str(self.comboBox.currentText())
            if names[0] == 'Unknown':
                print('[WARN] Unknown person')
                self.set_authority(False)
                self.drawPicture(image.copy(),cache=False)

                return False

            elif names[0] == text:
                print('[INFO] Authorized User: '+text)
                self.set_authority(True)
                self.drawPicture(image.copy(),cache=False)

                return True
            else:
                print('[WARN] Unauthorized User')
                self.set_authority(False)

                return False
    
    
    def on_btn_verify_click(self):
        print('[INFO] verify button pressed')

        self.th.refresh.disconnect()

        ret = self.verify(self.image.copy())
        if ret == False:
            # rotate again
            self.th.refresh.connect(lambda p:self.drawPicture(p))
        else:
            self.ros_th.recognizing = False
        
        self.ros_th.done = True
            
        
    def on_btn_openLink_click(self):
        print('[INFO] Open Link button pressed')
        self.th.stop()
        self.camera_running = False
        self.btn_takePhoto.setEnabled(False)
        webbrowser.open("https://d2dc5bd0.ngrok.io/A")
        webbrowser.open("https://d2dc5bd0.ngrok.io/B")
	

    def on_btn_clear_click(self):
        self.viewer.clear()
        self.image = []

    def on_btn_ok_click(self):
        self.th.refresh.disconnect()

    def on_btn_quit_click(self):
        print('[INFO] Quit Application')
        if self.camera_running:
            self.th.stop()
        self.tcp_th.running = False
        self.close()

    def on_btn_home_click(self):
        # todo
        # self.th2 = ros_node(6.25,0.463,0,self)
        # self.th2.start()
        # self.r_th.start()
        pass
    
    def on_btn_point_click(self):
        self.x = float(self.pos_x.text())
        self.y = float(self.pos_y.text())
        self.heading = float(self.pos_h.text())
        
        self.ros_th.gogoagv = True

    def on_btn_cancel_click(self):
        if self.th2:
            self.th2.cancel()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
