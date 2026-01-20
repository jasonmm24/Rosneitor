# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'interfaz2.ui'
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
    QMainWindow, QPlainTextEdit, QProgressBar, QPushButton,
    QSizePolicy, QSpacerItem, QVBoxLayout, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(600, 500)
        MainWindow.setStyleSheet(u"background-color: #f0f0f0;")
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.verticalLayout = QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.groupBox_datos = QGroupBox(self.centralwidget)
        self.groupBox_datos.setObjectName(u"groupBox_datos")
        font = QFont()
        font.setBold(True)
        self.groupBox_datos.setFont(font)
        self.horizontalLayout = QHBoxLayout(self.groupBox_datos)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.lbl_datos_info = QLabel(self.groupBox_datos)
        self.lbl_datos_info.setObjectName(u"lbl_datos_info")

        self.horizontalLayout.addWidget(self.lbl_datos_info)

        self.horizontalSpacer = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout.addItem(self.horizontalSpacer)

        self.lbl_contador = QLabel(self.groupBox_datos)
        self.lbl_contador.setObjectName(u"lbl_contador")
        font1 = QFont()
        font1.setPointSize(10)
        font1.setBold(True)
        self.lbl_contador.setFont(font1)
        self.lbl_contador.setStyleSheet(u"color: blue;")

        self.horizontalLayout.addWidget(self.lbl_contador)

        self.btn_cargar = QPushButton(self.groupBox_datos)
        self.btn_cargar.setObjectName(u"btn_cargar")

        self.horizontalLayout.addWidget(self.btn_cargar)


        self.verticalLayout.addWidget(self.groupBox_datos)

        self.groupBox_train = QGroupBox(self.centralwidget)
        self.groupBox_train.setObjectName(u"groupBox_train")
        self.groupBox_train.setFont(font)
        self.verticalLayout_2 = QVBoxLayout(self.groupBox_train)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.btn_entrenar = QPushButton(self.groupBox_train)
        self.btn_entrenar.setObjectName(u"btn_entrenar")
        self.btn_entrenar.setMinimumSize(QSize(0, 50))
        font2 = QFont()
        font2.setPointSize(12)
        font2.setBold(True)
        self.btn_entrenar.setFont(font2)
        self.btn_entrenar.setStyleSheet(u"background-color: #2ecc71; color: white; border-radius: 5px;")

        self.verticalLayout_2.addWidget(self.btn_entrenar)

        self.progressBar = QProgressBar(self.groupBox_train)
        self.progressBar.setObjectName(u"progressBar")
        self.progressBar.setValue(0)
        self.progressBar.setAlignment(Qt.AlignCenter)

        self.verticalLayout_2.addWidget(self.progressBar)


        self.verticalLayout.addWidget(self.groupBox_train)

        self.groupBox_log = QGroupBox(self.centralwidget)
        self.groupBox_log.setObjectName(u"groupBox_log")
        self.verticalLayout_3 = QVBoxLayout(self.groupBox_log)
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.txt_log = QPlainTextEdit(self.groupBox_log)
        self.txt_log.setObjectName(u"txt_log")
        self.txt_log.setStyleSheet(u"background-color: white; font-family: Courier;")
        self.txt_log.setReadOnly(True)

        self.verticalLayout_3.addWidget(self.txt_log)


        self.verticalLayout.addWidget(self.groupBox_log)

        self.groupBox_res = QGroupBox(self.centralwidget)
        self.groupBox_res.setObjectName(u"groupBox_res")
        self.groupBox_res.setFont(font)
        self.horizontalLayout_2 = QHBoxLayout(self.groupBox_res)
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.label_3 = QLabel(self.groupBox_res)
        self.label_3.setObjectName(u"label_3")

        self.horizontalLayout_2.addWidget(self.label_3)

        self.lbl_accuracy = QLabel(self.groupBox_res)
        self.lbl_accuracy.setObjectName(u"lbl_accuracy")
        font3 = QFont()
        font3.setPointSize(16)
        font3.setBold(True)
        self.lbl_accuracy.setFont(font3)
        self.lbl_accuracy.setStyleSheet(u"color: #e74c3c;")
        self.lbl_accuracy.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_2.addWidget(self.lbl_accuracy)


        self.verticalLayout.addWidget(self.groupBox_res)

        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"Programa 2: Entrenador de IA", None))
        self.groupBox_datos.setTitle(QCoreApplication.translate("MainWindow", u"1. Conjunto de Datos (Dataset)", None))
        self.lbl_datos_info.setText(QCoreApplication.translate("MainWindow", u"Archivo: datos_entrenamiento.csv", None))
        self.lbl_contador.setText(QCoreApplication.translate("MainWindow", u"Registros: 0", None))
        self.btn_cargar.setText(QCoreApplication.translate("MainWindow", u"Recargar CSV", None))
        self.groupBox_train.setTitle(QCoreApplication.translate("MainWindow", u"2. Entrenamiento Neuronal", None))
        self.btn_entrenar.setText(QCoreApplication.translate("MainWindow", u"GENERAR CEREBRO (Entrenar)", None))
        self.groupBox_log.setTitle(QCoreApplication.translate("MainWindow", u"Registro de Eventos (Log)", None))
        self.groupBox_res.setTitle(QCoreApplication.translate("MainWindow", u"Resultados", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"Precisi\u00f3n del Modelo:", None))
        self.lbl_accuracy.setText(QCoreApplication.translate("MainWindow", u"--- %", None))
    # retranslateUi

