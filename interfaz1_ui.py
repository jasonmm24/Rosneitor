# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'interfaz1.ui'
##
## Created by: Qt User Interface Compiler version 6.10.1
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QFrame, QGroupBox, QHBoxLayout,
    QLabel, QMainWindow, QPushButton, QSizePolicy,
    QSlider, QSpacerItem, QVBoxLayout, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(900, 650)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.horizontalLayout = QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.lbl_video = QLabel(self.centralwidget)
        self.lbl_video.setObjectName(u"lbl_video")
        self.lbl_video.setMinimumSize(QSize(640, 480))
        self.lbl_video.setFrameShape(QFrame.Box)
        self.lbl_video.setAlignment(Qt.AlignCenter)

        self.horizontalLayout.addWidget(self.lbl_video)

        self.verticalLayout = QVBoxLayout()
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.label_info = QLabel(self.centralwidget)
        self.label_info.setObjectName(u"label_info")
        font = QFont()
        font.setPointSize(12)
        font.setBold(True)
        self.label_info.setFont(font)
        self.label_info.setAlignment(Qt.AlignCenter)

        self.verticalLayout.addWidget(self.label_info)

        self.lbl_info = QLabel(self.centralwidget)
        self.lbl_info.setObjectName(u"lbl_info")
        self.lbl_info.setAlignment(Qt.AlignCenter)

        self.verticalLayout.addWidget(self.lbl_info)

        self.verticalSpacer_2 = QSpacerItem(20, 20, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout.addItem(self.verticalSpacer_2)

        self.btn_auto = QPushButton(self.centralwidget)
        self.btn_auto.setObjectName(u"btn_auto")
        self.btn_auto.setMinimumSize(QSize(0, 50))
        font1 = QFont()
        font1.setBold(True)
        self.btn_auto.setFont(font1)
        self.btn_auto.setCheckable(True)

        self.verticalLayout.addWidget(self.btn_auto)

        self.btn_stop = QPushButton(self.centralwidget)
        self.btn_stop.setObjectName(u"btn_stop")
        self.btn_stop.setMinimumSize(QSize(0, 50))
        self.btn_stop.setStyleSheet(u"background-color: rgb(255, 85, 0); color: white;")

        self.verticalLayout.addWidget(self.btn_stop)

        self.verticalSpacer = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout.addItem(self.verticalSpacer)

        self.groupBox = QGroupBox(self.centralwidget)
        self.groupBox.setObjectName(u"groupBox")
        self.verticalLayout_2 = QVBoxLayout(self.groupBox)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.label_2 = QLabel(self.groupBox)
        self.label_2.setObjectName(u"label_2")

        self.verticalLayout_2.addWidget(self.label_2)

        self.sl_h_min = QSlider(self.groupBox)
        self.sl_h_min.setObjectName(u"sl_h_min")
        self.sl_h_min.setMaximum(179)
        self.sl_h_min.setOrientation(Qt.Horizontal)

        self.verticalLayout_2.addWidget(self.sl_h_min)

        self.sl_h_max = QSlider(self.groupBox)
        self.sl_h_max.setObjectName(u"sl_h_max")
        self.sl_h_max.setMaximum(179)
        self.sl_h_max.setValue(179)
        self.sl_h_max.setOrientation(Qt.Horizontal)

        self.verticalLayout_2.addWidget(self.sl_h_max)

        self.label_3 = QLabel(self.groupBox)
        self.label_3.setObjectName(u"label_3")

        self.verticalLayout_2.addWidget(self.label_3)

        self.sl_s_min = QSlider(self.groupBox)
        self.sl_s_min.setObjectName(u"sl_s_min")
        self.sl_s_min.setMaximum(255)
        self.sl_s_min.setOrientation(Qt.Horizontal)

        self.verticalLayout_2.addWidget(self.sl_s_min)

        self.sl_s_max = QSlider(self.groupBox)
        self.sl_s_max.setObjectName(u"sl_s_max")
        self.sl_s_max.setMaximum(255)
        self.sl_s_max.setValue(255)
        self.sl_s_max.setOrientation(Qt.Horizontal)

        self.verticalLayout_2.addWidget(self.sl_s_max)

        self.label_4 = QLabel(self.groupBox)
        self.label_4.setObjectName(u"label_4")

        self.verticalLayout_2.addWidget(self.label_4)

        self.sl_v_min = QSlider(self.groupBox)
        self.sl_v_min.setObjectName(u"sl_v_min")
        self.sl_v_min.setMaximum(255)
        self.sl_v_min.setOrientation(Qt.Horizontal)

        self.verticalLayout_2.addWidget(self.sl_v_min)

        self.sl_v_max = QSlider(self.groupBox)
        self.sl_v_max.setObjectName(u"sl_v_max")
        self.sl_v_max.setMaximum(255)
        self.sl_v_max.setValue(255)
        self.sl_v_max.setOrientation(Qt.Horizontal)

        self.verticalLayout_2.addWidget(self.sl_v_max)


        self.verticalLayout.addWidget(self.groupBox)


        self.horizontalLayout.addLayout(self.verticalLayout)

        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"Programa 1: Entrenador de Datos", None))
        self.lbl_video.setText(QCoreApplication.translate("MainWindow", u"C\u00c1MARA OFF", None))
        self.label_info.setText(QCoreApplication.translate("MainWindow", u"ESTADO: ESPERANDO", None))
        self.lbl_info.setText(QCoreApplication.translate("MainWindow", u"Presiona 'P' para Pausar/Reacomodar", None))
        self.btn_auto.setText(QCoreApplication.translate("MainWindow", u"ACTIVAR AUTO-RECOLECCI\u00d3N", None))
        self.btn_stop.setText(QCoreApplication.translate("MainWindow", u"PAUSA / REACOMODAR (P)", None))
        self.groupBox.setTitle(QCoreApplication.translate("MainWindow", u"Calibraci\u00f3n HSV (Pelota)", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"H Min / Max", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"S Min / Max", None))
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"V Min / Max", None))
    # retranslateUi

