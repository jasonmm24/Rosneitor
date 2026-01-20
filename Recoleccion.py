"""
PROGRAMA 1: ENTRENADOR Y RECOLECTOR DE DATOS
--------------------------------------------
Sistema de visión artificial y control para la recolección de datos
de entrenamiento (Data Logging) en un robot Mini Sumo diferencial.

Características:
- Detección de marcadores ArUco (ID 0: Robot, ID 1: Meta).
- Detección de objetos por color (Pelota).
- Algoritmo matemático de navegación (Auto-Recolección).
- Interfaz gráfica para control manual y supervisión.
"""

import sys
import csv
import os
import math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtGui import QImage, QPixmap
from interfaz1_ui import Ui_MainWindow

class RobotController(Node):
    """
    Nodo de ROS 2 encargado de la comunicación con el microcontrolador (ESP32).
    Publica tópicos de comando de movimiento y velocidad.
    """
    def __init__(self):
        super().__init__('data_trainer_node')
        self.publisher_cmd = self.create_publisher(Int32, '/car_cmd', 10)
        self.publisher_speed = self.create_publisher(Int32, '/car_speed', 10)
        self.last_cmd = 5

    def enviar_comando(self, cmd):
        """Publica un comando de dirección (1:FWD, 2:BWD, 3:LEFT, 4:RIGHT, 5:STOP)."""
        msg = Int32()
        msg.data = int(cmd)
        self.publisher_cmd.publish(msg)
        self.last_cmd = cmd

    def cambiar_velocidad(self, pwm):
        """Publica el valor de velocidad PWM (0-255)."""
        msg = Int32()
        msg.data = int(pwm)
        self.publisher_speed.publish(msg)

class VisionThread(QThread):
    """
    Hilo de procesamiento de visión y lógica de control.
    Maneja la cámara, detección de objetos y escritura del dataset CSV.
    """
    image_signal = Signal(np.ndarray)
    data_signal = Signal(str, str)

    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self._run_flag = True
        self.paused = True
        self.modo_auto = False
        
        self.csv_file = 'datos_entrenamiento.csv'
        self._init_csv()

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.hsv_params = [0, 179, 0, 255, 0, 255]
        self.meta_pos = (640, 240) 

    def _init_csv(self):
        """Inicializa el archivo CSV con encabezados si no existe."""
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, 'w', newline='') as f:
                csv.writer(f).writerow(["AnguloError", "Distancia", "Comando"])

    def update_hsv(self, val):
        """Actualiza los rangos HSV desde la interfaz gráfica."""
        self.hsv_params = val

    def set_paused(self, estado):
        """Establece el estado de pausa y detiene el robot si es necesario."""
        self.paused = estado
        if estado:
            self.ros_node.enviar_comando(5)

    def guardar_dato(self, error, dist, cmd):
        """Registra una muestra de entrenamiento en el archivo CSV."""
        if not self.paused and cmd in [1, 2, 3, 4]:
            try:
                with open(self.csv_file, 'a', newline='') as f:
                    csv.writer(f).writerow([error, dist, cmd])
                return True
            except Exception:
                pass
        return False

    def calcular_logica_matematica(self, robot_pos, robot_angle, ball_pos):
        """
        Calcula la acción óptima basada en geometría vectorial.
        Retorna: Comando sugerido, Modo de operación, Error Angular, Distancia y Punto Objetivo.
        """
        dx_meta = self.meta_pos[0] - ball_pos[0]
        dy_meta = self.meta_pos[1] - ball_pos[1]
        angle_meta = math.degrees(math.atan2(dy_meta, dx_meta))
        
        dist_offset = 120
        rads = math.radians(angle_meta)
        target_x = ball_pos[0] - math.cos(rads) * dist_offset
        target_y = ball_pos[1] - math.sin(rads) * dist_offset
        
        dist_to_target = math.sqrt((target_x - robot_pos[0])**2 + (target_y - robot_pos[1])**2)
        
        if dist_to_target > 50:
            objetivo = (target_x, target_y)
            modo = "ACOMODANDO"
        else:
            objetivo = self.meta_pos
            modo = "EMPUJANDO"

        dx = objetivo[0] - robot_pos[0]
        dy = objetivo[1] - robot_pos[1]
        target_angle = math.degrees(math.atan2(dy, dx))
        
        error_angle = target_angle - robot_angle
        while error_angle > 180: error_angle -= 360
        while error_angle < -180: error_angle += 360
        
        distancia = math.sqrt(dx**2 + dy**2)
        
        cmd = 4 if error_angle > 0 else 3 if abs(error_angle) > 15 else 1
            
        return cmd, modo, error_angle, distancia, (int(target_x), int(target_y))

    def run(self):
        """Bucle principal de visión."""
        ip_celular = "http://10.87.17.107:8080/video"
        cap = cv2.VideoCapture(ip_celular)
        # cap = cv2.VideoCapture(0) 
        
        while self._run_flag:
            ret, frame = cap.read()
            if ret:
                frame = cv2.resize(frame, (640, 480))
                robot_pos, robot_angle = None, 0
                ball_pos = None
                
                # Detección ArUco
                corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)
                if ids is not None:
                    for i, m_id in enumerate(ids):
                        if m_id[0] == 0: # ID 0: Robot
                            c = corners[i][0]
                            cx, cy = int((c[0][0]+c[2][0])/2), int((c[0][1]+c[2][1])/2)
                            fx, fy = (c[0][0]+c[1][0])/2, (c[0][1]+c[1][1])/2
                            robot_pos = (cx, cy)
                            robot_angle = math.degrees(math.atan2(fy - cy, fx - cx))
                            cv2.arrowedLine(frame, (cx, cy), (int(fx), int(fy)), (255, 0, 0), 3)
                        elif m_id[0] == 1: # ID 1: Meta Dinámica
                            c = corners[i][0]
                            mx, my = int((c[0][0]+c[2][0])/2), int((c[0][1]+c[2][1])/2)
                            self.meta_pos = (mx, my)
                            cv2.circle(frame, (mx, my), 10, (0, 255, 0), 2)

                # Detección Pelota (HSV)
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                lower = np.array([self.hsv_params[0], self.hsv_params[2], self.hsv_params[4]])
                upper = np.array([self.hsv_params[1], self.hsv_params[3], self.hsv_params[5]])
                mask = cv2.inRange(hsv, lower, upper)
                mask = cv2.erode(mask, None, iterations=2)
                mask = cv2.dilate(mask, None, iterations=2)
                
                cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if cnts:
                    c = max(cnts, key=cv2.contourArea)
                    ((bx, by), r) = cv2.minEnclosingCircle(c)
                    if r > 5:
                        ball_pos = (int(bx), int(by))
                        cv2.circle(frame, ball_pos, int(r), (0, 255, 255), 2)

                # Visualización Meta
                cv2.circle(frame, self.meta_pos, 5, (0, 255, 0), -1)

                # Lógica de Control
                cmd = 5
                estado_txt = "PAUSA"
                sub_txt = "ESPERANDO..."

                if not self.paused and robot_pos and ball_pos:
                    cmd_math, modo, err, dist, target_pt = self.calcular_logica_matematica(robot_pos, robot_angle, ball_pos)
                    cv2.circle(frame, target_pt, 5, (0, 0, 255), -1)

                    if self.modo_auto:
                        cmd = cmd_math
                        self.guardar_dato(err, dist, cmd)
                        estado_txt = f"AUTO: {modo}"
                    else:
                        cmd = self.ros_node.last_cmd
                        if self.guardar_dato(err, dist, cmd):
                            estado_txt = "GRABANDO MANUAL"
                        else:
                            estado_txt = "MANUAL"
                    
                    self.ros_node.enviar_comando(cmd)
                    sub_txt = f"CMD: {cmd} | Err: {int(err)}"
                
                elif not self.paused and self.modo_auto:
                    estado_txt = "BUSCANDO..."
                    self.ros_node.enviar_comando(5)

                self.image_signal.emit(frame)
                self.data_signal.emit(estado_txt, sub_txt)
        
        cap.release()

    def stop(self):
        self._run_flag = False
        self.wait()

class MainWindow(QMainWindow):
    """
    Controlador de la interfaz gráfica principal.
    """
    def __init__(self, ros_node):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.ros_node = ros_node
        
        self.sliders = [
            self.ui.sl_h_min, self.ui.sl_h_max, 
            self.ui.sl_s_min, self.ui.sl_s_max, 
            self.ui.sl_v_min, self.ui.sl_v_max
        ]
        for s in self.sliders: s.valueChanged.connect(self.update_vision)

        self.ui.btn_auto.clicked.connect(self.toggle_auto)
        self.ui.btn_stop.clicked.connect(self.toggle_pause)

        self.thread = VisionThread(self.ros_node)
        self.thread.image_signal.connect(self.update_image)
        self.thread.data_signal.connect(self.update_labels)
        self.thread.start()

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_P:
            self.toggle_pause()
            return
        
        if self.thread.paused or self.thread.modo_auto:
            return

        k = event.key()
        cmd = 5
        if k == Qt.Key_W: cmd = 1
        elif k == Qt.Key_S: cmd = 2
        elif k == Qt.Key_A: cmd = 3
        elif k == Qt.Key_D: cmd = 4
        
        if cmd != 5:
            self.ros_node.enviar_comando(cmd)

    def keyReleaseEvent(self, event):
        if not self.thread.modo_auto and not self.thread.paused and not event.isAutoRepeat():
             self.ros_node.enviar_comando(5)

    def toggle_auto(self, checked):
        self.thread.modo_auto = checked
        self.ui.btn_auto.setText("MODO AUTO (ACTIVO)" if checked else "ACTIVAR AUTO-RECOLECCIÓN")

    def toggle_pause(self):
        nuevo_estado = not self.thread.paused
        self.thread.set_paused(nuevo_estado)
        txt = "REANUDAR (P)" if nuevo_estado else "PAUSAR (P)"
        self.ui.btn_stop.setText(txt)

    def update_vision(self):
        vals = [s.value() for s in self.sliders]
        self.thread.update_hsv(vals)

    def update_labels(self, main_txt, sub_txt):
        self.ui.label_info.setText(main_txt)
        self.ui.lbl_info.setText(sub_txt)

    def update_image(self, cv_img):
        h, w, ch = cv_img.shape
        qt_img = QImage(cv_img.data, w, h, ch*w, QImage.Format_BGR888)
        self.ui.lbl_video.setPixmap(QPixmap.fromImage(qt_img))
    
    def closeEvent(self, e):
        self.thread.stop()
        e.accept()

if __name__ == "__main__":
    rclpy.init()
    app = QApplication(sys.argv)
    win = MainWindow(RobotController())
    win.show()
    sys.exit(app.exec())