# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'GUI.ui'
#
# Created by: PyQt4 UI code generator 4.11.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(405, 191)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.btn_voice_rec = QtGui.QPushButton(self.centralwidget)
        self.btn_voice_rec.setGeometry(QtCore.QRect(10, 70, 151, 41))
        self.btn_voice_rec.setObjectName(_fromUtf8("btn_voice_rec"))
        self.btn_send_pos = QtGui.QPushButton(self.centralwidget)
        self.btn_send_pos.setGeometry(QtCore.QRect(280, 10, 111, 41))
        self.btn_send_pos.setObjectName(_fromUtf8("btn_send_pos"))
        self.pos_x = QtGui.QLineEdit(self.centralwidget)
        self.pos_x.setGeometry(QtCore.QRect(10, 10, 71, 41))
        self.pos_x.setObjectName(_fromUtf8("pos_x"))
        self.pos_y = QtGui.QLineEdit(self.centralwidget)
        self.pos_y.setGeometry(QtCore.QRect(100, 10, 71, 41))
        self.pos_y.setObjectName(_fromUtf8("pos_y"))
        self.pos_h = QtGui.QLineEdit(self.centralwidget)
        self.pos_h.setGeometry(QtCore.QRect(190, 10, 71, 41))
        self.pos_h.setObjectName(_fromUtf8("pos_h"))
        self.btn_stop = QtGui.QPushButton(self.centralwidget)
        self.btn_stop.setGeometry(QtCore.QRect(280, 70, 111, 41))
        self.btn_stop.setObjectName(_fromUtf8("btn_stop"))
        self.message = QtGui.QLabel(self.centralwidget)
        self.message.setGeometry(QtCore.QRect(20, 120, 251, 61))
        font = QtGui.QFont()
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.message.setFont(font)
        self.message.setObjectName(_fromUtf8("message"))
        self.btn_home = QtGui.QPushButton(self.centralwidget)
        self.btn_home.setGeometry(QtCore.QRect(280, 130, 111, 41))
        self.btn_home.setObjectName(_fromUtf8("btn_home"))
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "AGV", None))
        self.btn_voice_rec.setText(_translate("MainWindow", "Voice Recognition", None))
        self.btn_send_pos.setText(_translate("MainWindow", "Send Position", None))
        self.pos_x.setText(_translate("MainWindow", "0", None))
        self.pos_y.setText(_translate("MainWindow", "0", None))
        self.pos_h.setText(_translate("MainWindow", "0", None))
        self.btn_stop.setText(_translate("MainWindow", "STOP", None))
        self.message.setText(_translate("MainWindow", "No Target", None))
        self.btn_home.setText(_translate("MainWindow", "Home", None))

