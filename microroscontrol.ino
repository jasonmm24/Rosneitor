/*
 * ======================================================================================
 * PROYECTO: ROBOT MÓVIL CONTROLADO POR ROS 2 (MICRO-ROS)
 * AUTOR:    Jonathan Jason Medina Martinez
 * FECHA:    Diciembre 2025
 * * DESCRIPCIÓN:
 * Este firmware permite controlar un robot diferencial de 2 motores mediante el protocolo
 * ROS 2 sobre WiFi. El sistema distingue entre comandos de dirección y velocidad
 * basándose en el valor entero recibido.
 * * HARDWARE:
 * - Microcontrolador: ESP32 DevKit V1
 * - Driver de Motores: L298N
 * - Comunicación: WiFi (UDP Agent)
 * * PROTOCOLO DE COMUNICACIÓN (/car_cmd - Int32):
 * - 1: ADELANTE
 * - 2: ATRÁS
 * - 3: IZQUIERDA
 * - 4: DERECHA
 * - 5: DETENER
 * - [10 - 255]: AJUSTE DE VELOCIDAD (PWM)
 * ======================================================================================
 */

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

// --- Configuración de Red y Agente micro-ROS ---
const char *ssid = "Programación Avanzada";
const char *password = "1234567890";
const char *agent_ip = "192.168.3.11";
size_t agent_port = 8888;

// --- Definición de Pines (Driver L298N) ---

const int ENA = 4;  // PWM Motor A
const int IN1 = 2;  // Dirección Motor A
const int IN2 = 15; // Dirección Motor A
const int ENB = 12; // PWM Motor B
const int IN3 = 13; // Dirección Motor B
const int IN4 = 14; // Dirección Motor B

// --- Variables de Estado Global ---
int pwm_speed = 180;    // Velocidad inicial (0-255)
int current_action = 5; // Estado inicial (5 = STOP)

// --- Objetos y Estructuras ROS 2 ---
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

/**
 * @brief Controla el hardware de los motores basándose en el estado global.
 * * Interpreta `current_action` para establecer la dirección de los pines del L298N.
 * Aplica `pwm_speed` a los pines de habilitación (ENA/ENB).
 * Incluye una protección para evitar PWM insuficiente (<60) a menos que se ordene detener.
 */
void drive_motors() {
  int speed = pwm_speed;
  
  // Umbral mínimo de potencia para vencer la inercia (stall protection)
  if(current_action != 5 && speed < 60) speed = 60;

  if (current_action == 1) {        // ADELANTE
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, speed);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, speed);
  }
  else if (current_action == 2) {   // ATRÁS
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, speed);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, speed);
  }
  else if (current_action == 3) {   // IZQUIERDA (Giro sobre eje)
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, speed);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, speed);
  }
  else if (current_action == 4) {   // DERECHA (Giro sobre eje)
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, speed);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, speed);
  }
  else {                            // DETENER / DEFAULT
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

/**
 * @brief Callback de suscripción para el tópico /car_cmd.
 * * Implementa la lógica de multiplexación del protocolo:
 * - Valores < 10: Se interpretan como cambios de estado de movimiento.
 * - Valores >= 10: Se interpretan como configuración de velocidad (PWM).
 * * @param msgin Puntero al mensaje recibido (std_msgs/Int32).
 */
void subscription_callback(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  int data = msg->data;

  if (data < 10) {
    current_action = data;
  } else {
    pwm_speed = data;
    if (pwm_speed > 255) pwm_speed = 255; // Saturación máxima
  }
  
  // Actualizar motores inmediatamente tras recibir comando
  drive_motors();
}

/**
 * @brief Configuración inicial del sistema.
 * * 1. Inicializa Serial y Pines.
 * 2. Configura el transporte micro-ROS (WiFi).
 * 3. Inicializa el nodo, el ejecutor y la suscripción.
 */
void setup() {
  Serial.begin(115200);
  delay(2000); 
  
  Serial.println("");
  Serial.println("===============================");
  Serial.println(">> INICIANDO SISTEMA...");
  
  // Configuración de pines
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  drive_motors(); // Asegurar estado inicial detenido

  // Configuración de transporte WiFi
  Serial.print(">> CONECTANDO WIFI: "); Serial.println(ssid);
  Serial.print(">> AGENTE IP: "); Serial.println(agent_ip);

  set_microros_wifi_transports((char*)ssid, (char*)password, (char*)agent_ip, agent_port);
  
  Serial.println(">> WIFI INICIADO, CONECTANDO A ROS...");
  delay(2000);

  // Inicialización de la pila micro-ROS
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_carro", "", &support);

  Serial.println(">> CREANDO SUSCRIPTOR...");
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/car_cmd");

  // Configuración del ejecutor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
  
  Serial.println(">> SISTEMA LISTO Y CORRIENDO!");
  Serial.println("===============================");
}

/**
 * @brief Bucle principal.
 * * Ejecuta las tareas pendientes de micro-ROS (procesar callbacks) 
 * con un timeout de 100ms.
 */
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  delay(10);
}
