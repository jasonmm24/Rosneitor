"""
PROGRAMA 3: SISTEMA DE COMPETENCIA (IA)
---------------------------------------
Controlador final para despliegue en competencia.
Utiliza el modelo entrenado para tomar decisiones autónomas basadas
en la posición relativa del robot, la pelota y la meta dinámica.
"""

import sys
import os
import math
import joblib
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtGui import QImage, QPixmap
from interfaz3_ui import Ui_MainWindow

class RobotController(Node):
    """
    Nodo ROS 2 para control de actuadores en tiempo real.
    """
    def __init__(self):
        super().__init__('competition_node')
        self.publisher_cmd = self.create_publisher(Int32, '/car_cmd', 10)
        self.publisher_speed = self.create_publisher(Int32, '/car_speed', 10)

    def enviar_comando(self, cmd):
        msg = Int32()
        msg.data = int(cmd)
        self.publisher_cmd.publish(msg)

class VisionThread(QThread):
    """
    Hilo de inferencia y visión. Carga el modelo ML y ejecuta predicciones
    sobre el stream de video en tiempo real.
    """
    image_signal = Signal(np.ndarray)
    status_signal = Signal(str, str)

    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self._run_flag = True
        self.ai_active = False
        self.model = None
        
        if os.path.exists('cerebro_robot.pkl'):
            self.model = joblib.load('cerebro_robot.pkl')
            
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.hsv_params = [0, 179, 0, 255, 0, 255]
        self.meta_pos = (640, 240) # Meta por defecto

    def update_hsv(self, val):
        self.hsv_params = val

    def get_geometry_features(self, robot_pos, robot_angle, ball_pos):
        """
        Extrae las características geométricas (Features) necesarias para el modelo ML.
        Replica la lógica matemática del Programa 1 para consistencia.
        """
        dx_meta = self.meta_pos[0] - ball_pos[0]
        dy_meta = self.meta_pos[1] - ball_pos[1]
        angle_meta = math.degrees(math.atan2(dy_meta, dx_meta))
        
        dist_offset = 120
        rads = math.radians(angle_meta)
        target_x = ball_pos[0] - math.cos(rads) * dist_offset
        target_y = ball_pos[1] - math.sin(rads) * dist_offset
        
        dist_to_target = math.sqrt((target_x - robot_pos[0])**2 + (target_y - robot_pos[1])**2)
        objetivo = (target_x, target_y) if dist_to_target > 50 else self.meta_pos

        dx = objetivo[0] - robot_pos[0]
        dy = objetivo[1] - robot_pos[1]
        target_angle = math.degrees(math.atan2(dy, dx))
        
        error_angle = target_angle - robot_angle
        while error_angle > 180: error_angle -= 360
        while error_angle < -180: error_angle += 360
        
        return error_angle, math.sqrt(dx**2 + dy**2), (int(target_x), int(target_y))

    def run(self):
        """Bucle principal de inferencia."""
        ip_celular = "http://10.87.17.107:8080/video"
        cap = cv2.VideoCapture(ip_celular)
        # cap = cv2.VideoCapture(0) 
        
        while self._run_flag:
            ret, frame = cap.read()
            if ret:
                frame = cv2.resize(frame, (640, 480))
                robot_pos, robot_angle = None, 0
                ball_pos = None

                # 1. Detección ArUco (Robot ID 0, Meta ID 1)
                corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)
                if ids is not None:
                    for i, m_id in enumerate(ids):
                        if m_id[0] == 0:
                            c = corners[i][0]
                            cx, cy = int((c[0][0]+c[2][0])/2), int((c[0][1]+c[2][1])/2)
                            fx, fy = (c[0][0]+c[1][0])/2, (c[0][1]+c[1][1])/2
                            robot_pos = (cx, cy)
                            robot_angle = math.degrees(math.atan2(fy - cy, fx - cx))
                            cv2.arrowedLine(frame, (cx, cy), (int(fx), int(fy)), (255, 0, 0), 3)
                        elif m_id[0] == 1:
                            c = corners[i][0]
                            self.meta_pos = (int((c[0][0]+c[2][0])/2), int((c[0][1]+c[2][1])/2))
                
                # Visualización Meta
                cv2.circle(frame, self.meta_pos, 10, (0, 255, 0), 2)

                # 2. Detección Pelota
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

                # 3. Lógica IA
                cmd = 5
                main = "MODO: MANUAL"
                sub = "Control Humano..."

                if self.ai_active:
                    main = "MODO: IA AUTO"
                    if self.model and robot_pos and ball_pos:
                        err, dist, t_pt = self.get_geometry_features(robot_pos, robot_angle, ball_pos)
                        cv2.circle(frame, t_pt, 5, (0, 255, 255), -1)
                        
                        # Inferencia
                        cmd = int(self.model.predict([[err, dist]])[0])
                        self.ros_node.enviar_comando(cmd)
                        sub = f"IA Acción: {cmd}"
                    else:
                        self.ros_node.enviar_comando(5)
                        sub = "Buscando objetivos..." if self.model else "Error: Sin Modelo"
                
                self.image_signal.emit(frame)
                self.status_signal.emit(main, sub)
        
        cap.release()

    def stop(self):
        self._run_flag = False
        self.wait()

class MainWindow(QMainWindow):
    """
    Panel de control de competencia.
    """
    def __init__(self, ros_node):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.ros_node = ros_node
        
        self.ui.btn_start_ia.clicked.connect(self.toggle_ia)
        self.ui.btn_estop.clicked.connect(self.estop)
        
        self.sliders = [
            self.ui.sl_h_min, self.ui.sl_h_max, 
            self.ui.sl_s_min, self.ui.sl_s_max, 
            self.ui.sl_v_min, self.ui.sl_v_max
        ]
        for s in self.sliders: s.valueChanged.connect(self.update_vision)
        
        self.thread = VisionThread(self.ros_node)
        self.thread.image_signal.connect(self.update_image)
        self.thread.status_signal.connect(self.update_status)
        self.thread.start()

    def toggle_ia(self, c):
        self.thread.ai_active = c
        self.ui.btn_start_ia.setText("IA ACTIVA" if c else "INICIAR IA")
        self.ui.btn_start_ia.setStyleSheet("background-color: #f39c12;" if c else "background-color: #27ae60;")

    def estop(self):
        self.toggle_ia(False)
        self.ui.btn_start_ia.setChecked(False)
        self.ros_node.enviar_comando(5)

    def update_vision(self):
        vals = [s.value() for s in self.sliders]
        self.thread.update_hsv(vals)

    def update_status(self, m, s):
        self.ui.lbl_estado.setText(m)
        self.ui.lbl_comando.setText(s)

    def update_image(self, cv_img):
        h, w, c = cv_img.shape
        self.ui.lbl_video.setPixmap(QPixmap.fromImage(QImage(cv_img.data, w, h, c*w, QImage.Format_BGR888)))

    def keyPressEvent(self, event):
        if not self.thread.ai_active:
            k = event.key()
            cmd = 5
            if k == Qt.Key_W: cmd = 1
            elif k == Qt.Key_S: cmd = 2
            elif k == Qt.Key_A: cmd = 3
            elif k == Qt.Key_D: cmd = 4
            elif k == Qt.Key_Space: self.estop()
            if cmd != 5: self.ros_node.enviar_comando(cmd)

    def keyReleaseEvent(self, event):
        if not self.thread.ai_active and not event.isAutoRepeat():
            self.ros_node.enviar_comando(5)

    def closeEvent(self, e):
        self.thread.stop()
        self.ros_node.destroy_node()
        e.accept()

if __name__ == "__main__":
    rclpy.init()
    app = QApplication(sys.argv)
    win = MainWindow(RobotController())
    win.show()
    sys.exit(app.exec())