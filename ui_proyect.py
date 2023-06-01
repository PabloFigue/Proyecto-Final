# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'designerSoovGp.ui'
##
## Created by: Qt User Interface Compiler version 5.15.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import *
from PySide6.QtGui import *
from PySide6.QtWidgets import *



class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(465, 417)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.pushButton = QPushButton(self.centralwidget)
        self.pushButton.setObjectName(u"pushButton")
        self.pushButton.setGeometry(QRect(190, 60, 121, 31))
        self.pushButton_2 = QPushButton(self.centralwidget)
        self.pushButton_2.setObjectName(u"pushButton_2")
        self.pushButton_2.setGeometry(QRect(190, 100, 121, 31))
        self.pushButton_3 = QPushButton(self.centralwidget)
        self.pushButton_3.setObjectName(u"pushButton_3")
        self.pushButton_3.setGeometry(QRect(190, 140, 121, 31))
        self.pushButton_4 = QPushButton(self.centralwidget)
        self.pushButton_4.setObjectName(u"pushButton_4")
        self.pushButton_4.setGeometry(QRect(190, 180, 121, 31))
        self.textBrowser = QTextBrowser(self.centralwidget)
        self.textBrowser.setObjectName(u"textBrowser")
        self.textBrowser.setGeometry(QRect(40, 230, 381, 131))
        self.dial = QDial(self.centralwidget)
        self.dial.setObjectName(u"dial")
        self.dial.setGeometry(QRect(40, 50, 131, 131))
        self.pushButton_5 = QPushButton(self.centralwidget)
        self.pushButton_5.setObjectName(u"pushButton_5")
        self.pushButton_5.setGeometry(QRect(50, 10, 104, 31))
        self.checkBox = QCheckBox(self.centralwidget)
        self.checkBox.setObjectName(u"checkBox")
        self.checkBox.setGeometry(QRect(40, 180, 131, 27))
        self.pushButton_6 = QPushButton(self.centralwidget)
        self.pushButton_6.setObjectName(u"pushButton_6")
        self.pushButton_6.setGeometry(QRect(360, 100, 41, 31))
        self.pushButton_7 = QPushButton(self.centralwidget)
        self.pushButton_7.setObjectName(u"pushButton_7")
        self.pushButton_7.setGeometry(QRect(360, 140, 41, 31))
        self.label = QLabel(self.centralwidget)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(340, 60, 91, 31))
        self.comboBox = QComboBox(self.centralwidget)
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.setObjectName(u"comboBox")
        self.comboBox.setGeometry(QRect(190, 10, 88, 28))
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 465, 22))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.pushButton.setText(QCoreApplication.translate("MainWindow", u"M. Brazo 1", None))
        self.pushButton_2.setText(QCoreApplication.translate("MainWindow", u"Motor Garra", None))
        self.pushButton_3.setText(QCoreApplication.translate("MainWindow", u"Motor Base", None))
        self.pushButton_4.setText(QCoreApplication.translate("MainWindow", u"M. Brazo 2", None))
        self.pushButton_5.setText(QCoreApplication.translate("MainWindow", u"ON/OFF", None))
        self.checkBox.setText(QCoreApplication.translate("MainWindow", u"Modo Manual", None))
        self.pushButton_6.setText(QCoreApplication.translate("MainWindow", u"0", None))
        self.pushButton_7.setText(QCoreApplication.translate("MainWindow", u"1", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"Posiciones", None))
        self.comboBox.setItemText(0, QCoreApplication.translate("MainWindow", u"COM1", None))
        self.comboBox.setItemText(1, QCoreApplication.translate("MainWindow", u"COM2", None))
        self.comboBox.setItemText(2, QCoreApplication.translate("MainWindow", u"COM3", None))
        self.comboBox.setItemText(3, QCoreApplication.translate("MainWindow", u"COM4", None))
        self.comboBox.setItemText(4, QCoreApplication.translate("MainWindow", u"COM5", None))
        self.comboBox.setItemText(5, QCoreApplication.translate("MainWindow", u"COM6", None))
        self.comboBox.setItemText(6, QCoreApplication.translate("MainWindow", u"COM7", None))
        self.comboBox.setItemText(7, QCoreApplication.translate("MainWindow", u"COM8", None))
        self.comboBox.setItemText(8, QCoreApplication.translate("MainWindow", u"COM9", None))
        self.comboBox.setItemText(9, QCoreApplication.translate("MainWindow", u"COM10", None))

    # retranslateUi

