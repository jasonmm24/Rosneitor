# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'interfaz.ui'
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
from PySide6.QtWidgets import (QApplication, QComboBox, QDoubleSpinBox, QFrame,
    QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QListWidget, QListWidgetItem, QMainWindow, QPushButton,
    QSizePolicy, QSlider, QSpacerItem, QStackedWidget,
    QTextEdit, QVBoxLayout, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(750, 600)
        MainWindow.setStyleSheet(u"background-color: #2c3e50; color: white;")
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.horizontalLayout = QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setSpacing(0)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.frame_menu = QFrame(self.centralwidget)
        self.frame_menu.setObjectName(u"frame_menu")
        self.frame_menu.setMinimumSize(QSize(200, 0))
        self.frame_menu.setMaximumSize(QSize(200, 16777215))
        self.frame_menu.setStyleSheet(u"background-color: #1a252f;")
        self.verticalLayout_menu = QVBoxLayout(self.frame_menu)
        self.verticalLayout_menu.setSpacing(15)
        self.verticalLayout_menu.setObjectName(u"verticalLayout_menu")
        self.label_titulo = QLabel(self.frame_menu)
        self.label_titulo.setObjectName(u"label_titulo")
        font = QFont()
        font.setFamilies([u"Arial"])
        font.setPointSize(14)
        font.setBold(True)
        self.label_titulo.setFont(font)
        self.label_titulo.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout_menu.addWidget(self.label_titulo)

        self.line = QFrame(self.frame_menu)
        self.line.setObjectName(u"line")
        self.line.setFrameShape(QFrame.Shape.HLine)
        self.line.setFrameShadow(QFrame.Shadow.Sunken)

        self.verticalLayout_menu.addWidget(self.line)

        self.lbl_estado = QLabel(self.frame_menu)
        self.lbl_estado.setObjectName(u"lbl_estado")
        self.lbl_estado.setStyleSheet(u"color: #e74c3c; font-weight: bold;")
        self.lbl_estado.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout_menu.addWidget(self.lbl_estado)

        self.btn_conectar = QPushButton(self.frame_menu)
        self.btn_conectar.setObjectName(u"btn_conectar")
        self.btn_conectar.setMinimumSize(QSize(0, 40))
        self.btn_conectar.setStyleSheet(u"QPushButton { background-color: #27ae60; border: none; border-radius: 5px; font-weight: bold; } QPushButton:hover { background-color: #2ecc71; }")

        self.verticalLayout_menu.addWidget(self.btn_conectar)

        self.grp_vel = QGroupBox(self.frame_menu)
        self.grp_vel.setObjectName(u"grp_vel")
        self.vl_speed = QVBoxLayout(self.grp_vel)
        self.vl_speed.setObjectName(u"vl_speed")
        self.lbl_valor_vel = QLabel(self.grp_vel)
        self.lbl_valor_vel.setObjectName(u"lbl_valor_vel")
        self.lbl_valor_vel.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.vl_speed.addWidget(self.lbl_valor_vel)

        self.slider_velocidad = QSlider(self.grp_vel)
        self.slider_velocidad.setObjectName(u"slider_velocidad")
        self.slider_velocidad.setMaximum(255)
        self.slider_velocidad.setValue(180)
        self.slider_velocidad.setOrientation(Qt.Orientation.Horizontal)

        self.vl_speed.addWidget(self.slider_velocidad)


        self.verticalLayout_menu.addWidget(self.grp_vel)

        self.verticalSpacer = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_menu.addItem(self.verticalSpacer)

        self.btn_nav_manual = QPushButton(self.frame_menu)
        self.btn_nav_manual.setObjectName(u"btn_nav_manual")
        self.btn_nav_manual.setMinimumSize(QSize(0, 45))
        self.btn_nav_manual.setStyleSheet(u"QPushButton { background-color: #34495e; border-left: 5px solid #3498db; text-align: left; padding-left: 10px; font-weight: bold; } QPushButton:hover { background-color: #2c3e50; }")

        self.verticalLayout_menu.addWidget(self.btn_nav_manual)

        self.btn_nav_seq = QPushButton(self.frame_menu)
        self.btn_nav_seq.setObjectName(u"btn_nav_seq")
        self.btn_nav_seq.setMinimumSize(QSize(0, 45))
        self.btn_nav_seq.setStyleSheet(u"QPushButton { background-color: #34495e; border-left: 5px solid #9b59b6; text-align: left; padding-left: 10px; font-weight: bold; } QPushButton:hover { background-color: #2c3e50; }")

        self.verticalLayout_menu.addWidget(self.btn_nav_seq)

        self.btn_nav_prog = QPushButton(self.frame_menu)
        self.btn_nav_prog.setObjectName(u"btn_nav_prog")
        self.btn_nav_prog.setMinimumSize(QSize(0, 45))
        self.btn_nav_prog.setStyleSheet(u"QPushButton { background-color: #34495e; border-left: 5px solid #f1c40f; text-align: left; padding-left: 10px; font-weight: bold; } QPushButton:hover { background-color: #2c3e50; }")

        self.verticalLayout_menu.addWidget(self.btn_nav_prog)

        self.verticalSpacer_2 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_menu.addItem(self.verticalSpacer_2)

        self.btn_exit = QPushButton(self.frame_menu)
        self.btn_exit.setObjectName(u"btn_exit")
        self.btn_exit.setMinimumSize(QSize(0, 40))
        self.btn_exit.setStyleSheet(u"QPushButton { background-color: #c0392b; border-radius: 5px; font-weight: bold; } QPushButton:hover { background-color: #e74c3c; }")

        self.verticalLayout_menu.addWidget(self.btn_exit)


        self.horizontalLayout.addWidget(self.frame_menu)

        self.layout_content = QVBoxLayout()
        self.layout_content.setObjectName(u"layout_content")
        self.layout_content.setContentsMargins(10, 10, 10, 10)
        self.stackedWidget = QStackedWidget(self.centralwidget)
        self.stackedWidget.setObjectName(u"stackedWidget")
        self.page_manual = QWidget()
        self.page_manual.setObjectName(u"page_manual")
        self.vl_manual = QVBoxLayout(self.page_manual)
        self.vl_manual.setObjectName(u"vl_manual")
        self.lbl_p1 = QLabel(self.page_manual)
        self.lbl_p1.setObjectName(u"lbl_p1")
        font1 = QFont()
        font1.setPointSize(12)
        font1.setBold(True)
        self.lbl_p1.setFont(font1)
        self.lbl_p1.setStyleSheet(u"color: #3498db;")
        self.lbl_p1.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.vl_manual.addWidget(self.lbl_p1)

        self.sp1 = QSpacerItem(0, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.vl_manual.addItem(self.sp1)

        self.grid_manual = QGridLayout()
        self.grid_manual.setObjectName(u"grid_manual")
        self.btn_fwd = QPushButton(self.page_manual)
        self.btn_fwd.setObjectName(u"btn_fwd")
        self.btn_fwd.setMinimumSize(QSize(80, 60))
        self.btn_fwd.setStyleSheet(u"background-color: #34495e; font-size: 20px; border-radius: 8px;")

        self.grid_manual.addWidget(self.btn_fwd, 0, 1, 1, 1)

        self.btn_left = QPushButton(self.page_manual)
        self.btn_left.setObjectName(u"btn_left")
        self.btn_left.setMinimumSize(QSize(80, 60))
        self.btn_left.setStyleSheet(u"background-color: #34495e; font-size: 20px; border-radius: 8px;")

        self.grid_manual.addWidget(self.btn_left, 1, 0, 1, 1)

        self.btn_stop = QPushButton(self.page_manual)
        self.btn_stop.setObjectName(u"btn_stop")
        self.btn_stop.setMinimumSize(QSize(80, 60))
        self.btn_stop.setStyleSheet(u"background-color: #e74c3c; font-weight: bold; border-radius: 8px;")

        self.grid_manual.addWidget(self.btn_stop, 1, 1, 1, 1)

        self.btn_right = QPushButton(self.page_manual)
        self.btn_right.setObjectName(u"btn_right")
        self.btn_right.setMinimumSize(QSize(80, 60))
        self.btn_right.setStyleSheet(u"background-color: #34495e; font-size: 20px; border-radius: 8px;")

        self.grid_manual.addWidget(self.btn_right, 1, 2, 1, 1)

        self.btn_bwd = QPushButton(self.page_manual)
        self.btn_bwd.setObjectName(u"btn_bwd")
        self.btn_bwd.setMinimumSize(QSize(80, 60))
        self.btn_bwd.setStyleSheet(u"background-color: #34495e; font-size: 20px; border-radius: 8px;")

        self.grid_manual.addWidget(self.btn_bwd, 2, 1, 1, 1)


        self.vl_manual.addLayout(self.grid_manual)

        self.sp2 = QSpacerItem(0, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.vl_manual.addItem(self.sp2)

        self.stackedWidget.addWidget(self.page_manual)
        self.page_seq = QWidget()
        self.page_seq.setObjectName(u"page_seq")
        self.vl_seq = QVBoxLayout(self.page_seq)
        self.vl_seq.setObjectName(u"vl_seq")
        self.lbl_p2 = QLabel(self.page_seq)
        self.lbl_p2.setObjectName(u"lbl_p2")
        self.lbl_p2.setFont(font1)
        self.lbl_p2.setStyleSheet(u"color: #9b59b6;")
        self.lbl_p2.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.vl_seq.addWidget(self.lbl_p2)

        self.sp3 = QSpacerItem(0, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.vl_seq.addItem(self.sp3)

        self.btn_sq = QPushButton(self.page_seq)
        self.btn_sq.setObjectName(u"btn_sq")
        self.btn_sq.setMinimumSize(QSize(0, 50))
        self.btn_sq.setStyleSheet(u"background-color: #8e44ad; font-weight: bold; border-radius: 5px;")

        self.vl_seq.addWidget(self.btn_sq)

        self.btn_zz = QPushButton(self.page_seq)
        self.btn_zz.setObjectName(u"btn_zz")
        self.btn_zz.setMinimumSize(QSize(0, 50))
        self.btn_zz.setStyleSheet(u"background-color: #8e44ad; font-weight: bold; border-radius: 5px;")

        self.vl_seq.addWidget(self.btn_zz)

        self.btn_inf = QPushButton(self.page_seq)
        self.btn_inf.setObjectName(u"btn_inf")
        self.btn_inf.setMinimumSize(QSize(0, 50))
        self.btn_inf.setStyleSheet(u"background-color: #8e44ad; font-weight: bold; border-radius: 5px;")

        self.vl_seq.addWidget(self.btn_inf)

        self.hl_seq_ctrl = QHBoxLayout()
        self.hl_seq_ctrl.setObjectName(u"hl_seq_ctrl")
        self.btn_pause_seq = QPushButton(self.page_seq)
        self.btn_pause_seq.setObjectName(u"btn_pause_seq")
        self.btn_pause_seq.setMinimumSize(QSize(0, 40))

        self.hl_seq_ctrl.addWidget(self.btn_pause_seq)

        self.btn_stop_seq = QPushButton(self.page_seq)
        self.btn_stop_seq.setObjectName(u"btn_stop_seq")
        self.btn_stop_seq.setMinimumSize(QSize(0, 40))

        self.hl_seq_ctrl.addWidget(self.btn_stop_seq)


        self.vl_seq.addLayout(self.hl_seq_ctrl)

        self.sp4 = QSpacerItem(0, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.vl_seq.addItem(self.sp4)

        self.stackedWidget.addWidget(self.page_seq)
        self.page_prog = QWidget()
        self.page_prog.setObjectName(u"page_prog")
        self.vl_prog = QVBoxLayout(self.page_prog)
        self.vl_prog.setObjectName(u"vl_prog")
        self.lbl_p3 = QLabel(self.page_prog)
        self.lbl_p3.setObjectName(u"lbl_p3")
        self.lbl_p3.setFont(font1)
        self.lbl_p3.setStyleSheet(u"color: #f1c40f;")
        self.lbl_p3.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.vl_prog.addWidget(self.lbl_p3)

        self.hl_inputs = QHBoxLayout()
        self.hl_inputs.setObjectName(u"hl_inputs")
        self.combo_acciones = QComboBox(self.page_prog)
        self.combo_acciones.addItem("")
        self.combo_acciones.addItem("")
        self.combo_acciones.addItem("")
        self.combo_acciones.addItem("")
        self.combo_acciones.addItem("")
        self.combo_acciones.setObjectName(u"combo_acciones")
        self.combo_acciones.setStyleSheet(u"background-color: #34495e; padding: 5px;")

        self.hl_inputs.addWidget(self.combo_acciones)

        self.spin_tiempo = QDoubleSpinBox(self.page_prog)
        self.spin_tiempo.setObjectName(u"spin_tiempo")
        self.spin_tiempo.setStyleSheet(u"background-color: #34495e;")
        self.spin_tiempo.setMinimum(0.100000000000000)
        self.spin_tiempo.setMaximum(10.000000000000000)
        self.spin_tiempo.setValue(1.000000000000000)

        self.hl_inputs.addWidget(self.spin_tiempo)

        self.btn_agregar = QPushButton(self.page_prog)
        self.btn_agregar.setObjectName(u"btn_agregar")
        self.btn_agregar.setStyleSheet(u"background-color: #2980b9; font-weight: bold;")

        self.hl_inputs.addWidget(self.btn_agregar)


        self.vl_prog.addLayout(self.hl_inputs)

        self.list_secuencia = QListWidget(self.page_prog)
        self.list_secuencia.setObjectName(u"list_secuencia")
        self.list_secuencia.setStyleSheet(u"background-color: #1a252f; color: #f39c12;")

        self.vl_prog.addWidget(self.list_secuencia)

        self.hl_btns_prog = QHBoxLayout()
        self.hl_btns_prog.setObjectName(u"hl_btns_prog")
        self.btn_ejecutar_custom = QPushButton(self.page_prog)
        self.btn_ejecutar_custom.setObjectName(u"btn_ejecutar_custom")
        self.btn_ejecutar_custom.setMinimumSize(QSize(0, 40))
        self.btn_ejecutar_custom.setStyleSheet(u"background-color: #27ae60; font-weight: bold;")

        self.hl_btns_prog.addWidget(self.btn_ejecutar_custom)

        self.btn_pause_custom = QPushButton(self.page_prog)
        self.btn_pause_custom.setObjectName(u"btn_pause_custom")
        self.btn_pause_custom.setMinimumSize(QSize(0, 40))

        self.hl_btns_prog.addWidget(self.btn_pause_custom)

        self.btn_stop_custom = QPushButton(self.page_prog)
        self.btn_stop_custom.setObjectName(u"btn_stop_custom")
        self.btn_stop_custom.setMinimumSize(QSize(0, 40))

        self.hl_btns_prog.addWidget(self.btn_stop_custom)

        self.btn_limpiar = QPushButton(self.page_prog)
        self.btn_limpiar.setObjectName(u"btn_limpiar")
        self.btn_limpiar.setMinimumSize(QSize(0, 40))
        self.btn_limpiar.setStyleSheet(u"background-color: #7f8c8d; font-weight: bold;")

        self.hl_btns_prog.addWidget(self.btn_limpiar)


        self.vl_prog.addLayout(self.hl_btns_prog)

        self.stackedWidget.addWidget(self.page_prog)

        self.layout_content.addWidget(self.stackedWidget)

        self.txt_log = QTextEdit(self.centralwidget)
        self.txt_log.setObjectName(u"txt_log")
        self.txt_log.setMaximumSize(QSize(16777215, 100))
        self.txt_log.setStyleSheet(u"background-color: black; color: #00ff00; font-family: Consolas;")
        self.txt_log.setReadOnly(True)

        self.layout_content.addWidget(self.txt_log)


        self.horizontalLayout.addLayout(self.layout_content)

        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)

        self.stackedWidget.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"ROS 2 Pro Control", None))
        self.label_titulo.setText(QCoreApplication.translate("MainWindow", u"ROS 2 PANEL", None))
        self.lbl_estado.setText(QCoreApplication.translate("MainWindow", u"DESCONECTADO", None))
        self.btn_conectar.setText(QCoreApplication.translate("MainWindow", u"CONECTAR", None))
        self.grp_vel.setTitle(QCoreApplication.translate("MainWindow", u"Velocidad Global", None))
        self.lbl_valor_vel.setText(QCoreApplication.translate("MainWindow", u"PWM: 180", None))
        self.btn_nav_manual.setText(QCoreApplication.translate("MainWindow", u"\U0001f3ae  Manual (Push)", None))
        self.btn_nav_seq.setText(QCoreApplication.translate("MainWindow", u"\U0001f504  Secuencias", None))
        self.btn_nav_prog.setText(QCoreApplication.translate("MainWindow", u"\U0001f6e0  Programador", None))
        self.btn_exit.setText(QCoreApplication.translate("MainWindow", u"SALIR", None))
        self.lbl_p1.setText(QCoreApplication.translate("MainWindow", u"MODO CONTROL MANUAL (Mantener presionado)", None))
        self.btn_fwd.setText(QCoreApplication.translate("MainWindow", u"\u25b2", None))
        self.btn_left.setText(QCoreApplication.translate("MainWindow", u"\u25c4", None))
        self.btn_stop.setText(QCoreApplication.translate("MainWindow", u"STOP", None))
        self.btn_right.setText(QCoreApplication.translate("MainWindow", u"\u25ba", None))
        self.btn_bwd.setText(QCoreApplication.translate("MainWindow", u"\u25bc", None))
        self.lbl_p2.setText(QCoreApplication.translate("MainWindow", u"SECUENCIAS PREDEFINIDAS", None))
        self.btn_sq.setText(QCoreApplication.translate("MainWindow", u"Ejecutar CUADRADO", None))
        self.btn_zz.setText(QCoreApplication.translate("MainWindow", u"Ejecutar ZIG-ZAG", None))
        self.btn_inf.setText(QCoreApplication.translate("MainWindow", u"Ejecutar OCHO", None))
        self.btn_pause_seq.setStyleSheet(QCoreApplication.translate("MainWindow", u"background-color: #f39c12; font-weight: bold;", None))
        self.btn_pause_seq.setText(QCoreApplication.translate("MainWindow", u"\u23f8 Pausar/Reanudar", None))
        self.btn_stop_seq.setStyleSheet(QCoreApplication.translate("MainWindow", u"background-color: #c0392b; font-weight: bold;", None))
        self.btn_stop_seq.setText(QCoreApplication.translate("MainWindow", u"\u23f9 Interrumpir", None))
        self.lbl_p3.setText(QCoreApplication.translate("MainWindow", u"PROGRAMADOR PERSONALIZADO", None))
        self.combo_acciones.setItemText(0, QCoreApplication.translate("MainWindow", u"Adelante", None))
        self.combo_acciones.setItemText(1, QCoreApplication.translate("MainWindow", u"Atr\u00e1s", None))
        self.combo_acciones.setItemText(2, QCoreApplication.translate("MainWindow", u"Izquierda", None))
        self.combo_acciones.setItemText(3, QCoreApplication.translate("MainWindow", u"Derecha", None))
        self.combo_acciones.setItemText(4, QCoreApplication.translate("MainWindow", u"Pausa (Stop)", None))

        self.spin_tiempo.setSuffix(QCoreApplication.translate("MainWindow", u" s", None))
        self.btn_agregar.setText(QCoreApplication.translate("MainWindow", u"+ Agregar", None))
        self.btn_ejecutar_custom.setText(QCoreApplication.translate("MainWindow", u"\u25b6 Ejecutar Lista", None))
        self.btn_pause_custom.setStyleSheet(QCoreApplication.translate("MainWindow", u"background-color: #f39c12; font-weight: bold;", None))
        self.btn_pause_custom.setText(QCoreApplication.translate("MainWindow", u"\u23f8 Pausa", None))
        self.btn_stop_custom.setStyleSheet(QCoreApplication.translate("MainWindow", u"background-color: #c0392b; font-weight: bold;", None))
        self.btn_stop_custom.setText(QCoreApplication.translate("MainWindow", u"\u23f9 Stop", None))
        self.btn_limpiar.setText(QCoreApplication.translate("MainWindow", u"\U0001f5d1 Borrar", None))
    # retranslateUi

