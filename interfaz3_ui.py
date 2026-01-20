# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'interfaz3.ui'
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
from PySide6.QtWidgets import (QApplication, QGroupBox, QHBoxLayout, QLabel,
    QMainWindow, QPushButton, QSizePolicy, QSlider,
    QSpacerItem, QVBoxLayout, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(950, 600)
        MainWindow.setStyleSheet(u"background-color: #2c3e50; color: white;")
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.horizontalLayout = QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.lbl_video = QLabel(self.centralwidget)
        self.lbl_video.setObjectName(u"lbl_video")
        self.lbl_video.setMinimumSize(QSize(640, 480))
        self.lbl_video.setStyleSheet(u"background-color: black; border: 2px solid #34495e;")
        self.lbl_video.setAlignment(Qt.AlignCenter)

        self.horizontalLayout.addWidget(self.lbl_video)

        self.verticalLayout = QVBoxLayout()
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.groupBox_status = QGroupBox(self.centralwidget)
        self.groupBox_status.setObjectName(u"groupBox_status")
        self.verticalLayout_2 = QVBoxLayout(self.groupBox_status)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.lbl_estado = QLabel(self.groupBox_status)
        self.lbl_estado.setObjectName(u"lbl_estado")
        font = QFont()
        font.setPointSize(16)
        font.setBold(True)
        self.lbl_estado.setFont(font)
        self.lbl_estado.setStyleSheet(u"color: #f39c12;")
        self.lbl_estado.setAlignment(Qt.AlignCenter)

        self.verticalLayout_2.addWidget(self.lbl_estado)

        self.lbl_comando = QLabel(self.groupBox_status)
        self.lbl_comando.setObjectName(u"lbl_comando")
        font1 = QFont()
        font1.setPointSize(12)
        self.lbl_comando.setFont(font1)
        self.lbl_comando.setAlignment(Qt.AlignCenter)

        self.verticalLayout_2.addWidget(self.lbl_comando)


        self.verticalLayout.addWidget(self.groupBox_status)

        self.btn_start_ia = QPushButton(self.centralwidget)
        self.btn_start_ia.setObjectName(u"btn_start_ia")
        self.btn_start_ia.setMinimumSize(QSize(0, 60))
        font2 = QFont()
        font2.setPointSize(14)
        font2.setBold(True)
        self.btn_start_ia.setFont(font2)
        self.btn_start_ia.setStyleSheet(u"background-color: #27ae60; border-radius: 5px;")
        self.btn_start_ia.setCheckable(True)

        self.verticalLayout.addWidget(self.btn_start_ia)

        self.btn_estop = QPushButton(self.centralwidget)
        self.btn_estop.setObjectName(u"btn_estop")
        self.btn_estop.setMinimumSize(QSize(0, 60))
        self.btn_estop.setFont(font2)
        self.btn_estop.setStyleSheet(u"background-color: #c0392b; border-radius: 5px;")

        self.verticalLayout.addWidget(self.btn_estop)

        self.verticalSpacer = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout.addItem(self.verticalSpacer)

        self.groupBox_calib = QGroupBox(self.centralwidget)
        self.groupBox_calib.setObjectName(u"groupBox_calib")
        self.groupBox_calib.setStyleSheet(u"QGroupBox { border: 1px solid gray; border-radius: 3px; margin-top: 0.5em; } QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px 0 3px; }")
        self.verticalLayout_3 = QVBoxLayout(self.groupBox_calib)
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.label_3 = QLabel(self.groupBox_calib)
        self.label_3.setObjectName(u"label_3")

        self.verticalLayout_3.addWidget(self.label_3)

        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.sl_h_min = QSlider(self.groupBox_calib)
        self.sl_h_min.setObjectName(u"sl_h_min")
        self.sl_h_min.setMaximum(179)
        self.sl_h_min.setOrientation(Qt.Horizontal)

        self.horizontalLayout_2.addWidget(self.sl_h_min)

        self.sl_h_max = QSlider(self.groupBox_calib)
        self.sl_h_max.setObjectName(u"sl_h_max")
        self.sl_h_max.setMaximum(179)
        self.sl_h_max.setValue(179)
        self.sl_h_max.setOrientation(Qt.Horizontal)

        self.horizontalLayout_2.addWidget(self.sl_h_max)


        self.verticalLayout_3.addLayout(self.horizontalLayout_2)

        self.label_4 = QLabel(self.groupBox_calib)
        self.label_4.setObjectName(u"label_4")

        self.verticalLayout_3.addWidget(self.label_4)

        self.horizontalLayout_3 = QHBoxLayout()
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.sl_s_min = QSlider(self.groupBox_calib)
        self.sl_s_min.setObjectName(u"sl_s_min")
        self.sl_s_min.setMaximum(255)
        self.sl_s_min.setOrientation(Qt.Horizontal)

        self.horizontalLayout_3.addWidget(self.sl_s_min)

        self.sl_s_max = QSlider(self.groupBox_calib)
        self.sl_s_max.setObjectName(u"sl_s_max")
        self.sl_s_max.setMaximum(255)
        self.sl_s_max.setValue(255)
        self.sl_s_max.setOrientation(Qt.Horizontal)

        self.horizontalLayout_3.addWidget(self.sl_s_max)


        self.verticalLayout_3.addLayout(self.horizontalLayout_3)

        self.label_5 = QLabel(self.groupBox_calib)
        self.label_5.setObjectName(u"label_5")

        self.verticalLayout_3.addWidget(self.label_5)

        self.horizontalLayout_4 = QHBoxLayout()
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.sl_v_min = QSlider(self.groupBox_calib)
        self.sl_v_min.setObjectName(u"sl_v_min")
        self.sl_v_min.setMaximum(255)
        self.sl_v_min.setOrientation(Qt.Horizontal)

        self.horizontalLayout_4.addWidget(self.sl_v_min)

        self.sl_v_max = QSlider(self.groupBox_calib)
        self.sl_v_max.setObjectName(u"sl_v_max")
        self.sl_v_max.setMaximum(255)
        self.sl_v_max.setValue(255)
        self.sl_v_max.setOrientation(Qt.Horizontal)

        self.horizontalLayout_4.addWidget(self.sl_v_max)


        self.verticalLayout_3.addLayout(self.horizontalLayout_4)


        self.verticalLayout.addWidget(self.groupBox_calib)


        self.horizontalLayout.addLayout(self.verticalLayout)

        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"Programa 3: JUGADOR COMPETITIVO (IA)", None))
        self.lbl_video.setText(QCoreApplication.translate("MainWindow", u"NO SIGNAL", None))
        self.groupBox_status.setTitle(QCoreApplication.translate("MainWindow", u"Estado del Partido", None))
        self.lbl_estado.setText(QCoreApplication.translate("MainWindow", u"MODO: MANUAL", None))
        self.lbl_comando.setText(QCoreApplication.translate("MainWindow", u"Acci\u00f3n: Esperando...", None))
        self.btn_start_ia.setText(QCoreApplication.translate("MainWindow", u"INICIAR IA (AUTO)", None))
        self.btn_estop.setText(QCoreApplication.translate("MainWindow", u"PARO DE EMERGENCIA", None))
        self.groupBox_calib.setTitle(QCoreApplication.translate("MainWindow", u"Calibraci\u00f3n R\u00e1pida (Luz)", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"H - Tono", None))
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"S - Saturaci\u00f3n", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", u"V - Brillo", None))
    # retranslateUi

