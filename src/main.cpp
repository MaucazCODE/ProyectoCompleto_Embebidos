#include <AccelStepper.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "apwifieeprommode.h"
#include <EEPROM.h>

// Definir el servidor web
WebServer server(80);

// Pines motor 1 (izquierdo)
#define IN1_M1 14
#define IN2_M1 27
#define IN3_M1 26
#define IN4_M1 25

// Pines motor 2 (derecho)
#define IN1_M2 19
#define IN2_M2 18
#define IN3_M2 5
#define IN4_M2 32

// Definir puntos - AUMENTADO para más capacidad
#define MAX_PUNTOS 500
int historialAngulos[MAX_PUNTOS];
int historialDistancias[MAX_PUNTOS];
int numPuntos = 0;

// NUEVOS ARRAYS: Coordenadas absolutas de obstáculos
float obstaculosX[MAX_PUNTOS];
float obstaculosY[MAX_PUNTOS];

// CPUs a utilizar
#define PRO_CPU 0
#define APP_CPU 1
#define NOAFF_CPU tskNO_AFFINITY

// Crear objetos de motores
AccelStepper motor1(AccelStepper::FULL4WIRE, IN1_M1, IN3_M1, IN2_M1, IN4_M1);
AccelStepper motor2(AccelStepper::FULL4WIRE, IN1_M2, IN3_M2, IN2_M2, IN4_M2);

// Sensor LIDAR
VL53L0X sensor;

// Constantes
const int pasosPorGrado = 2048 / 360; // Motor 28BYJ-48 // Ajusta según pruebas
const int pasosPorMM = 50; // pasos para avanzar un mm // Ajusta según pruebas
const int margenSeguridad = 300; // mm, radio del robot

// Variables
int mejorAngulo = 0;
int mayorDistancia = 0;
float ultimoAngulo = 0;
float ultimaDistancia = 0;

// Posición del robot (en mm)
float robotX = 0;
float robotY = 0;
float robotAngulo = 0; // Ángulo actual del robot (0° = hacia adelante)

// NUEVAS VARIABLES: Coordenadas del punto de origen/centro
float origenX = 0; // Posición inicial X del robot (centro de referencia)
float origenY = 0; // Posición inicial Y del robot (centro de referencia)

// Variable de control de mapeo
bool isMappingActive = false; // Nueva variable

// Variables para control de escaneo
bool nuevoEscaneoCompleto = false;
int puntosAntesDeCiclo = 0;

// Constante para evitar duplicados
const float TOLERANCIA_DUPLICADOS = 50.0; // 50mm de tolerancia para considerar puntos duplicados

// Prototipos
void escanearYBuscar();
void girarRobot(int angulo);
void avanzarRobot(int mm);
bool esPuntoDuplicado(float nuevaX, float nuevaY);
void mostrarCoordenadas(float x, float y, String tipo); // NUEVA FUNCIÓN

void TestHwm(char *taskName);
void TaskESCANEO(void *pvParameters);
void TaskROTARCOM(void *pvParameters);

// Para sincronización
SemaphoreHandle_t xSemaphore = NULL;
volatile int tareasTerminadas = 0;  
// Hay 2 tareas paralelas por loop, que gire 360 y que escanee
const int TOTAL_TAREAS = 2;

void setup() {
  Serial.begin(115200);
  // Inicializar EEPROM
  EEPROM.begin(512);

  // ESTABLECER PUNTO DE ORIGEN/CENTRO DE REFERENCIA
  origenX = robotX; // Guardar posición inicial como centro (0, 0)
  origenY = robotY; // Guardar posición inicial como centro (0, 0)
  // Task paralelos
  // Estas tareas solo se iniciarán si isMappingActive es true.
  // Las crearemos y eliminaremos dinámicamente en el loop para evitar que corran indefinidamente.

  // Intentar conectarse a la red guardada (la de la laptop/hotspot)
  // Si no puede, crea el AP para registrar la red
  iniciarConexionWiFi("ESP32_AP", "clave1234"); // Cambia nombre y clave si lo deseas

  Wire.begin(21, 22); // SDA, SCL para VL53L0X


  sensor.init();
  sensor.setTimeout(500);
  sensor.startContinuous();

  
  motor1.setMaxSpeed(800);
  motor1.setAcceleration(400);

  motor2.setMaxSpeed(800);
  motor2.setAcceleration(400);

  // Se inicia "semáforo"
  xSemaphore = xSemaphoreCreateBinary();
  // Liberamos inicialmente
  xSemaphoreGive(xSemaphore); 
  delay(1000);
  
  Serial.println("Sistema iniciado. Esperando comando START...");
}

void loop() {
  // Manejar el servidor web
  server.handleClient();
  loopServidorWeb();
  
  if (isMappingActive) {
    // Recordar cuántos puntos teníamos antes del ciclo
    puntosAntesDeCiclo = numPuntos;
    
    // Escanear continuamente para mostrar el mapa del entorno
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
      tareasTerminadas = 0;
      // Se reinicia el contador por ciclo
      xSemaphoreGive(xSemaphore);
      
      xTaskCreatePinnedToCore(TaskESCANEO, "TaskESCANEO", 4096, NULL, 1, NULL, APP_CPU);
      xTaskCreatePinnedToCore(TaskROTARCOM, "TaskROTARCOM", 4096, NULL, 1, NULL, APP_CPU);
    }
  
    // AMBAS deben terminar
    while (true) {
      xSemaphoreTake(xSemaphore, portMAX_DELAY);
      if (tareasTerminadas >= TOTAL_TAREAS) {
        xSemaphoreGive(xSemaphore);
        break;
      }
      xSemaphoreGive(xSemaphore);
      delay(10);
      // Seguridad
    }
  
    // Verificar si hay nuevos puntos
    if (numPuntos > puntosAntesDeCiclo) {
      nuevoEscaneoCompleto = true;
      Serial.println("Nuevos puntos detectados: " + String(numPuntos - puntosAntesDeCiclo));
      Serial.println("Total de puntos acumulados: " + String(numPuntos));
    }
    
    delay(3000); // Pausa entre escaneos (3 segundos)
    girarRobot(mejorAngulo);
    int distancia_rec = mayorDistancia-margenSeguridad;
    avanzarRobot(distancia_rec);
  }
}

// ---------- FUNCIONES -------------

// FUNCIÓN NUEVA: Mostrar coordenadas relativas al origen
void mostrarCoordenadas(float x, float y, String tipo) {
  float relativaX = x - origenX;
  float relativaY = y - origenY;
  Serial.println("=== COORDENADAS " + tipo + " ===");
  Serial.println("Absolutas: X=" + String(x, 1) + " Y=" + String(y, 1));
  Serial.println("Relativas al origen: X=" + String(relativaX, 1) + " Y=" + String(relativaY, 1));
}

// FUNCIÓN NUEVA: Verificar si un punto ya existe
bool esPuntoDuplicado(float nuevaX, float nuevaY) {
  for (int i = 0; i < numPuntos; i++) {
    float distanciaExistente = sqrt(pow(obstaculosX[i] - nuevaX, 2) + 
                                  pow(obstaculosY[i] - nuevaY, 2));
    if (distanciaExistente < TOLERANCIA_DUPLICADOS) {
      return true; // Es un punto duplicado
    }
  }
  return false; // No es duplicado
}

void escanearYBuscar() {
  mejorAngulo = 0;
  mayorDistancia = 0;
  
  Serial.println("Iniciando escaneo 360°...");

  // IDA: 0° a 360° (sentido antihorario/izquierda)
  for (int angulo = 0; angulo <= 360; angulo += 5) {
    int dist = sensor.readRangeContinuousMillimeters();
    Serial.print("→ Ángulo: "); Serial.print(angulo);
    Serial.print(" mm: "); Serial.println(dist);

    // Actualizar último escaneo
    ultimoAngulo = (float)angulo;
    ultimaDistancia = (float)dist;

    // Solo agregar puntos válidos que estén dentro del rango
    if (!sensor.timeoutOccurred() && dist < 2000 && dist > 30) { // Filtrar lecturas muy cercanas también
      if (numPuntos < MAX_PUNTOS) {
        // CALCULAR COORDENADAS ABSOLUTAS
        float anguloAbsoluto = angulo + robotAngulo;
        float anguloRad = anguloAbsoluto * 3.14159265 / 180.0;
        float nuevaX = robotX + dist * cos(anguloRad);
        float nuevaY = robotY + dist * sin(anguloRad);
        
        // VERIFICAR SI YA EXISTE UN PUNTO SIMILAR
        if (!esPuntoDuplicado(nuevaX, nuevaY)) {
          // Solo agregar si no es duplicado
          historialAngulos[numPuntos] = angulo;
          historialDistancias[numPuntos] = dist;
          obstaculosX[numPuntos] = nuevaX;
          obstaculosY[numPuntos] = nuevaY;
          
          numPuntos++;
          Serial.println("Punto agregado #" + String(numPuntos) + " - Ángulo: " + String(angulo) + "° Distancia: " + String(dist) + "mm");
          // MOSTRAR COORDENADAS RELATIVAS AL ORIGEN
          mostrarCoordenadas(nuevaX, nuevaY, "OBSTÁCULO");
        } else {
          Serial.println("Punto duplicado ignorado - Ángulo: " + String(angulo) + "° Distancia: " + String(dist) + "mm");
        }
      } else {
        Serial.println("Límite de puntos alcanzado (" + String(MAX_PUNTOS) + ")");
      }
    }

    // Verificar conexión WiFi
    if (WiFi.status() == WL_CONNECTED) {
      // No enviar datos individuales, solo verificar conexión
    } else {
      Serial.println("WiFi desconectado");
    }

    // Buscar la mejor dirección para moverse
    if (!sensor.timeoutOccurred() && dist > mayorDistancia && dist < 2000) {
      mayorDistancia = dist;
      mejorAngulo = angulo;
    }
    
    delay(200); // Escanea cada 200 milisegundos
  }
  
  Serial.println("Escaneo completado. Mejor dirección: " + String(mejorAngulo) + "° (" + String(mayorDistancia) + "mm)");
}

void girarRobot(int angulo) {
  if (angulo == 0) return; // No girar si el ángulo es 0
  
  int pasosGiro = 6.516 * map(angulo, 0, 360, 0, 2048);

  Serial.println("Girará " + String(angulo) + "° Tomará " + String(pasosGiro) + " pasos");

  motor1.moveTo(motor1.currentPosition() - pasosGiro); // Izquierda atrás
  motor2.moveTo(motor2.currentPosition() + pasosGiro); // Derecha adelante

  while (motor1.distanceToGo() != 0 || motor2.distanceToGo() != 0) {
    motor1.run();
    motor2.run();
  }
    
  // Actualizar ángulo del robot
  robotAngulo += angulo;
  if (robotAngulo >= 360) robotAngulo -= 360;
  if (robotAngulo < 0) robotAngulo += 360;
  
  Serial.println("Robot giró " + String(angulo) + "°. Ángulo actual: " + String(robotAngulo) + "°");
}

void avanzarRobot(int mm) {
  if (mm <= 0) {
    Serial.println("No hay espacio seguro para avanzar");
    return;
  }
  
  int pasosAvance = 3.012 * map(mm, 0, 360, 0, 2048);
  Serial.println("Debe avanzar " + String(mm) + "mm Tomará " + String(pasosAvance) + " pasos");

  motor1.moveTo(motor1.currentPosition() + pasosAvance);
  motor2.moveTo(motor2.currentPosition() + pasosAvance);

  while (motor1.distanceToGo() != 0 && motor2.distanceToGo() != 0) {
    motor1.run();
    motor2.run();
  }
    
  // Actualizar posición del robot
  float radianes = robotAngulo * 3.14159265 / 180.0;
  robotX += mm * cos(radianes);
  robotY += mm * sin(radianes);
  
  Serial.println("Avance completado");
  // MOSTRAR POSICIÓN DEL ROBOT RELATIVA AL ORIGEN
  mostrarCoordenadas(robotX, robotY, "ROBOT");
  Serial.println("Ángulo robot: " + String(robotAngulo, 1) + "°");
}

void TestHwm(char *taskName) {
  static int stack_hwm, stack_hwm_temp;

  stack_hwm_temp = uxTaskGetStackHighWaterMark(nullptr);
  if (!stack_hwm || (stack_hwm_temp < stack_hwm)) {
    stack_hwm = stack_hwm_temp;
    Serial.printf("%s has stack hwm %u\n", taskName, stack_hwm);
  }
}

void TaskESCANEO(void *pvParameters) {
  escanearYBuscar();
  
  xSemaphoreTake(xSemaphore, portMAX_DELAY);
  tareasTerminadas++;
  xSemaphoreGive(xSemaphore);
  
  vTaskDelete(NULL);
}

void TaskROTARCOM(void *pvParameters) {
  girarRobot(360);
  
  xSemaphoreTake(xSemaphore, portMAX_DELAY);
  tareasTerminadas++;
  xSemaphoreGive(xSemaphore);
  
  vTaskDelete(NULL);
}