"""
PROGRAMA 2: ENTRENADOR DE INTELIGENCIA ARTIFICIAL
-------------------------------------------------
Procesa los datos recolectados y genera el modelo predictivo.
Utiliza RandomForestClassifier de Scikit-Learn.
"""

import sys
import os
import pandas as pd
import joblib
from sklearn.model_selection import train_test_split
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score
from PySide6.QtWidgets import QApplication, QMainWindow
from PySide6.QtCore import QThread, Signal
from interfaz2_ui import Ui_MainWindow

class TrainingWorker(QThread):
    """
    Hilo de ejecución para el entrenamiento del modelo ML.
    Evita congelar la interfaz gráfica durante el procesamiento.
    """
    log_signal = Signal(str)
    progress_signal = Signal(int)
    result_signal = Signal(float)
    finished_signal = Signal()

    def run(self):
        csv_file = 'datos_entrenamiento.csv'
        pkl_file = 'cerebro_robot.pkl'

        self.log_signal.emit("--- INICIANDO PROCESO DE ENTRENAMIENTO ---")
        self.progress_signal.emit(10)

        if not os.path.exists(csv_file):
            self.log_signal.emit(f"ERROR: Archivo {csv_file} no encontrado.")
            self.finished_signal.emit()
            return

        try:
            # Carga de datos
            df = pd.read_csv(csv_file)
            if len(df) < 10:
                self.log_signal.emit("ERROR: Datos insuficientes para entrenar.")
                self.finished_signal.emit()
                return
            
            self.log_signal.emit(f"Registros cargados: {len(df)}")
            self.progress_signal.emit(30)

            # Preprocesamiento
            X = df.iloc[:, 0:2] # Features: AnguloError, Distancia
            y = df.iloc[:, 2]   # Target: Comando
            
            X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)
            self.progress_signal.emit(50)

            # Entrenamiento
            self.log_signal.emit("Entrenando Random Forest Classifier...")
            clf = RandomForestClassifier(n_estimators=100, random_state=42)
            clf.fit(X_train, y_train)
            self.progress_signal.emit(80)

            # Evaluación
            acc = accuracy_score(y_test, clf.predict(X_test)) * 100
            self.log_signal.emit(f"Entrenamiento completado. Precisión: {acc:.2f}%")

            # Exportación
            joblib.dump(clf, pkl_file)
            self.log_signal.emit(f"Modelo guardado en {pkl_file}")
            
            self.progress_signal.emit(100)
            self.result_signal.emit(acc)

        except Exception as e:
            self.log_signal.emit(f"EXCEPCIÓN: {str(e)}")
        
        finally:
            self.finished_signal.emit()

class TrainerApp(QMainWindow):
    """
    Interfaz gráfica para la gestión del entrenamiento.
    """
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui.btn_cargar.clicked.connect(self.check_csv)
        self.ui.btn_entrenar.clicked.connect(self.start_training)
        self.check_csv()

    def check_csv(self):
        """Verifica la existencia y cantidad de registros del dataset."""
        if os.path.exists('datos_entrenamiento.csv'):
            try:
                count = len(pd.read_csv('datos_entrenamiento.csv'))
                self.ui.lbl_contador.setText(f"Registros: {count}")
                self.log(f"Dataset detectado: {count} registros.")
            except:
                self.ui.lbl_contador.setText("Error en CSV")
        else:
            self.ui.lbl_contador.setText("Registros: 0")

    def start_training(self):
        """Inicia el worker de entrenamiento."""
        self.ui.btn_entrenar.setEnabled(False)
        self.ui.progressBar.setValue(0)
        
        self.worker = TrainingWorker()
        self.worker.log_signal.connect(self.log)
        self.worker.progress_signal.connect(self.ui.progressBar.setValue)
        self.worker.result_signal.connect(self.show_result)
        self.worker.finished_signal.connect(lambda: self.ui.btn_entrenar.setEnabled(True))
        
        self.worker.start()

    def show_result(self, acc):
        self.ui.lbl_accuracy.setText(f"{acc:.2f}%")
        color = "#2ecc71" if acc > 80 else "#e74c3c"
        self.ui.lbl_accuracy.setStyleSheet(f"color: {color};")

    def log(self, text):
        self.ui.txt_log.appendPlainText(f">> {text}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = TrainerApp()
    window.show()
    sys.exit(app.exec())