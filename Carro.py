"""
======================================================================================
 PROYECTO: ROS 2 CONTROL FINAL
 AUTOR:    Jonathan Jason Medina Martinez
 FECHA:    Diciembre 2025

 DESCRIPCIÓN:
 Interfaz gráfica (GUI) desarrollada en PySide6 para controlar el robot móvil.
 Actúa como nodo publicador en ROS 2. Soporta:
 1. Control Manual (Teclado/Botones).
 2. Secuencias Predefinidas (Figuras).
 3. Programación secuencial por pasos personalizada.
======================================================================================
"""

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtCore import QTimer, Qt, QTime
from interfaz_ui import Ui_MainWindow

class RosNode(Node):
    """
    Clase que encapsula la funcionalidad de ROS 2.
    Se encarga únicamente de publicar comandos en el tópico '/car_cmd'.
    """
    def __init__(self):
        super().__init__('pc_gui_node')
        
        # Configuración QoS: Reliable es crítico para asegurar que los comandos
        # llegan al ESP32 sobre WiFi (UDP) sin pérdidas.
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        
        self.publisher_ = self.create_publisher(Int32, '/car_cmd', qos)

    def publicar(self, dato):
        """Publica un entero (Comando de movimiento o Velocidad PWM)."""
        msg = Int32()
        msg.data = int(dato)
        self.publisher_.publish(msg)

class MainWindow(QMainWindow):
    """
    Lógica principal de la UI. Gestiona la máquina de estados de conexión,
    los temporizadores de secuencias y la integración del loop de ROS con Qt.
    """
    def __init__(self):
        super().__init__()
        
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        
        # --- Variables de Estado ---
        self.conectado = False
        self.ros_node = None
        self.ui.stackedWidget.setCurrentIndex(0) # Iniciar en pág. manual

        # --- Timer para Loop de ROS ---
        # Permite ejecutar rclpy.spin_once() periódicamente sin bloquear la GUI
        self.timer_ros = QTimer()
        self.timer_ros.timeout.connect(self.spin_ros)

        # --- Variables de Control de Secuencias ---
        self.secuencia_activa = False
        self.en_pausa = False
        self.pasos_restantes = []     # Cola de ejecución [(comando, ms), ...]
        self.lista_programada = []    # Lista creada por el usuario en la UI
        
        self.timer_secuencia = QTimer()
        self.timer_secuencia.setSingleShot(True)
        self.timer_secuencia.timeout.connect(self.finalizar_paso_actual)
        
        # Variables para cálculo preciso de pausa/reanudación
        self.tiempo_inicio_paso = QTime()
        self.duracion_paso_actual = 0
        self.comando_actual_secuencia = 5
        self.tiempo_restante_exacto = 0

        # --- Conexiones de Señales (Eventos UI) ---
        self._conectar_eventos()

    def _conectar_eventos(self):
        """Agrupa todas las conexiones de botones y sliders."""
        # Sistema
        self.ui.btn_conectar.clicked.connect(self.toggle_conexion)
        self.ui.slider_velocidad.valueChanged.connect(self.cambiar_velocidad)
        self.ui.btn_exit.clicked.connect(self.close)

        # Navegación
        self.ui.btn_nav_manual.clicked.connect(lambda: self.cambiar_pagina(0))
        self.ui.btn_nav_seq.clicked.connect(lambda: self.cambiar_pagina(1))
        self.ui.btn_nav_prog.clicked.connect(lambda: self.cambiar_pagina(2))

        # Control Manual
        self.ui.btn_fwd.pressed.connect(lambda: self.manual_start(1))
        self.ui.btn_bwd.pressed.connect(lambda: self.manual_start(2))
        self.ui.btn_left.pressed.connect(lambda: self.manual_start(3))
        self.ui.btn_right.pressed.connect(lambda: self.manual_start(4))
        
        self.ui.btn_fwd.released.connect(self.manual_stop)
        self.ui.btn_bwd.released.connect(self.manual_stop)
        self.ui.btn_left.released.connect(self.manual_stop)
        self.ui.btn_right.released.connect(self.manual_stop)
        self.ui.btn_stop.clicked.connect(self.manual_stop)

        # Secuencias Predefinidas
        self.ui.btn_sq.clicked.connect(self.iniciar_cuadrado)
        self.ui.btn_zz.clicked.connect(self.iniciar_zigzag)
        self.ui.btn_inf.clicked.connect(self.iniciar_ocho)
        self.ui.btn_pause_seq.clicked.connect(self.toggle_pausa)
        self.ui.btn_stop_seq.clicked.connect(self.abortar_secuencia)

        # Programación Personalizada
        self.ui.btn_agregar.clicked.connect(self.agregar_paso_lista)
        self.ui.btn_limpiar.clicked.connect(self.limpiar_lista)
        self.ui.btn_ejecutar_custom.clicked.connect(self.iniciar_custom)
        self.ui.btn_pause_custom.clicked.connect(self.toggle_pausa)
        self.ui.btn_stop_custom.clicked.connect(self.abortar_secuencia)

    def cambiar_pagina(self, index):
        """Cambia la vista del StackedWidget y seguridad: detiene robot."""
        self.ui.stackedWidget.setCurrentIndex(index)
        self.abortar_secuencia()

    # =========================================================================
    # LÓGICA DE CONEXIÓN ROS 2
    # =========================================================================

    def toggle_conexion(self):
        """Maneja el inicio y cierre del contexto ROS 2 y el Nodo."""
        if not self.conectado:
            try:
                if not rclpy.ok(): rclpy.init(args=None)
                self.ros_node = RosNode()
                self.timer_ros.start(20) # Loop ROS a 50Hz aprox
                
                self.conectado = True
                self.ui.lbl_estado.setText("ONLINE")
                self.ui.lbl_estado.setStyleSheet("color: #2ecc71; font-weight: bold;")
                self.ui.btn_conectar.setText("DESCONECTAR")
                self.ui.btn_conectar.setStyleSheet("background-color: #c0392b;")
                self.log("Sistema: CONECTADO")
            except Exception as e:
                self.log(f"Error: {e}")
        else:
            self.abortar_secuencia()
            self.timer_ros.stop()
            if self.ros_node: self.ros_node.destroy_node()
            if rclpy.ok(): rclpy.shutdown()
            
            self.conectado = False
            self.ui.lbl_estado.setText("OFFLINE")
            self.ui.lbl_estado.setStyleSheet("color: #e74c3c; font-weight: bold;")
            self.ui.btn_conectar.setText("CONECTAR")
            self.ui.btn_conectar.setStyleSheet("background-color: #27ae60;")
            self.log("Sistema: DESCONECTADO")

    def spin_ros(self):
        """Ejecuta tareas pendientes de ROS sin bloquear el hilo principal."""
        if self.conectado and rclpy.ok():
            rclpy.spin_once(self.ros_node, timeout_sec=0)

    # =========================================================================
    # MOTOR DE SECUENCIAS (CORE LOGIC)
    # =========================================================================

    def cargar_secuencia(self, nombre, pasos):
        """
        Inicializa una nueva secuencia.
        :param nombre: Etiqueta para el log.
        :param pasos: Lista de tuplas (comando, duracion_ms).
        """
        if not self.conectado:
            self.log("Error: No conectado a ROS.")
            return
        
        self.abortar_secuencia() # Limpieza previa
        self.log(f"--- INICIO SECUENCIA: {nombre} ---")
        
        self.secuencia_activa = True
        self.en_pausa = False
        self.pasos_restantes = pasos
        
        self.ejecutar_siguiente_paso()

    def ejecutar_siguiente_paso(self):
        """Extrae el siguiente comando de la cola y configura el temporizador."""
        if not self.secuencia_activa: return

        if not self.pasos_restantes:
            self.log("--- SECUENCIA FINALIZADA CON ÉXITO ---")
            self.enviar_comando(5) # Stop Final
            self.secuencia_activa = False
            return

        cmd, duracion = self.pasos_restantes.pop(0)
        
        self.comando_actual_secuencia = cmd
        self.duracion_paso_actual = duracion 
        
        self.ros_node.publicar(cmd)
        
        self.tiempo_inicio_paso = QTime.currentTime()
        self.timer_secuencia.start(duracion)
        
        m = {1:"ADELANTE", 2:"ATRAS", 3:"IZQUIERDA", 4:"DERECHA", 5:"ESPERAR"}
        self.log(f"EJECUTANDO >> {m.get(cmd)} durante {duracion/1000.0}s")

    def finalizar_paso_actual(self):
        """Callback del timer cuando termina el tiempo de un paso."""
        self.ejecutar_siguiente_paso()

    def toggle_pausa(self):
        """
        Gestiona la pausa inteligente.
        Calcula el tiempo transcurrido para reanudar exactamente donde se quedó.
        """
        if not self.secuencia_activa: return

        if not self.en_pausa:
            # --- PAUSAR ---
            self.en_pausa = True
            self.timer_secuencia.stop()
            
            # Calcular cuánto tiempo faltaba
            ahora = QTime.currentTime()
            transcurrido = self.tiempo_inicio_paso.msecsTo(ahora)
            restante = self.duracion_paso_actual - transcurrido
            
            if restante < 0: restante = 0
            self.tiempo_restante_exacto = restante
            
            self.ros_node.publicar(5) # Detener físicamente el robot
            
            self.log(f"|| PAUSA || Paso interrumpido. Faltaban {restante}ms")
            self.ui.btn_pause_seq.setText("▶ Reanudar")
            self.ui.btn_pause_custom.setText("▶ Reanudar")
            
        else:
            # --- REANUDAR ---
            self.en_pausa = False
            self.log(f"▶ REANUDANDO por {self.tiempo_restante_exacto}ms...")
            
            # Reenviar el comando que estaba activo
            self.ros_node.publicar(self.comando_actual_secuencia)
            
            # Configurar timer solo con el tiempo que faltaba
            self.duracion_paso_actual = self.tiempo_restante_exacto 
            self.tiempo_inicio_paso = QTime.currentTime()
            self.timer_secuencia.start(self.tiempo_restante_exacto)

            self.ui.btn_pause_seq.setText("⏸ Pausar")
            self.ui.btn_pause_custom.setText("⏸ Pausar")

    def abortar_secuencia(self):
        """Detiene la secuencia inmediatamente y limpia colas."""
        if self.secuencia_activa or self.en_pausa:
            self.secuencia_activa = False
            self.en_pausa = False
            self.timer_secuencia.stop()
            self.pasos_restantes = []
            
            if self.conectado: 
                self.enviar_comando(5)
                
            self.log("!!! SECUENCIA ABORTADA !!!")
            self.ui.btn_pause_seq.setText("⏸ Pausar/Reanudar")
            self.ui.btn_pause_custom.setText("⏸ Pausar/Reanudar")

    # =========================================================================
    # DEFINICIÓN DE RUTINAS
    # =========================================================================

    def iniciar_cuadrado(self):
        # (Acción, ms) -> Mueve, para, gira, para...
        p = [(1,1000), (5,200), (4,600), (5,200)] * 4
        self.cargar_secuencia("CUADRADO", p)

    def iniciar_zigzag(self):
        p = [(1,1000), (5,200), (3,600), (5,200), (1,1000), (5,200), (4,600), (5,200), (1,1000)]
        self.cargar_secuencia("ZIGZAG", p)

    def iniciar_ocho(self):
        p = [(1,500), (4,2500), (1,500), (3,2500)]
        self.cargar_secuencia("OCHO", p)

    def agregar_paso_lista(self):
        """Agrega un paso a la lista personalizada desde la UI."""
        txt = self.ui.combo_acciones.currentText()
        sec = self.ui.spin_tiempo.value()
        ms = int(sec * 1000)
        
        m = {"Adelante":1, "Atrás":2, "Izquierda":3, "Derecha":4, "Pausa (Stop)":5}
        cmd = m.get(txt, 5)
        
        self.lista_programada.append((cmd, ms))
        self.ui.list_secuencia.addItem(f"{len(self.lista_programada)}. {txt} ({sec}s)")

    def limpiar_lista(self):
        self.lista_programada = []
        self.ui.list_secuencia.clear()

    def iniciar_custom(self):
        if self.lista_programada:
            self.cargar_secuencia("PERSONALIZADA", list(self.lista_programada))

    # =========================================================================
    # CONTROL MANUAL Y EVENTOS
    # =========================================================================

    def manual_start(self, valor):
        self.abortar_secuencia() # Seguridad: Prioridad al manual
        self.enviar_comando(valor)

    def manual_stop(self):
        if self.conectado:
            self.enviar_comando(5)

    def enviar_comando(self, valor, tipo="CMD"):
        if not self.conectado: return
        self.ros_node.publicar(valor)
        
        if tipo == "VEL": 
            self.log(f"PWM Ajustado: {valor}")
        else:
            m = {1:"FWD", 2:"BWD", 3:"LEFT", 4:"RIGHT", 5:"STOP"}
            if not self.secuencia_activa:
                self.log(f"Manual CMD: {m.get(valor,'?')}")

    def cambiar_velocidad(self):
        val = self.ui.slider_velocidad.value()
        self.ui.lbl_valor_vel.setText(f"PWM: {val}")
        if val < 10: val = 10
        if self.conectado: self.enviar_comando(val, "VEL")

    def keyPressEvent(self, event):
        """Mapeo de teclado (WASD)."""
        if self.ui.stackedWidget.currentIndex() != 0: return
        if not self.conectado or event.isAutoRepeat(): return
        
        k = event.key()
        if k == Qt.Key_W: self.manual_start(1)
        elif k == Qt.Key_S: self.manual_start(2)
        elif k == Qt.Key_A: self.manual_start(3)
        elif k == Qt.Key_D: self.manual_start(4)
        elif k == Qt.Key_Space: self.manual_stop()

    def keyReleaseEvent(self, event):
        """Detiene el robot al soltar la tecla."""
        if self.ui.stackedWidget.currentIndex() != 0: return
        if not self.conectado or event.isAutoRepeat(): return
        
        k = event.key()
        if k in [Qt.Key_W, Qt.Key_S, Qt.Key_A, Qt.Key_D]:
            self.manual_stop()

    def log(self, texto):
        """Escribe en el cuadro de texto de la UI y hace autoscroll."""
        self.ui.txt_log.append(f">> {texto}")
        sb = self.ui.txt_log.verticalScrollBar()
        sb.setValue(sb.maximum())

    def closeEvent(self, event):
        """Limpieza al cerrar la ventana."""
        self.abortar_secuencia()
        if self.conectado:
            self.ros_node.destroy_node()
            if rclpy.ok(): rclpy.shutdown()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
