#ifndef AP_WIFI_EEPROM_MODE_H
#define AP_WIFI_EEPROM_MODE_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>
#include "WiFi.h"

// Definir puntos - AUMENTADO para más capacidad
#define MAX_PUNTOS 500

// Variables externas
extern int numPuntos;
extern float ultimoAngulo;
extern float ultimaDistancia;
extern float robotX;
extern float robotY;
extern float robotAngulo;
extern float obstaculosX[MAX_PUNTOS];
extern float obstaculosY[MAX_PUNTOS];
extern WebServer server;

// Nuevo: Variable de control global
extern bool isMappingActive;
extern int mejorAngulo;
extern int mayorDistancia;

// --- Utilidades EEPROM ---
String leerStringDeEEPROM(int direccion) {
    String cadena = "";
    char caracter = EEPROM.read(direccion);
    int i = 0;
    while (caracter != '\0' && i < 100) {
        cadena += caracter;
        i++;
        caracter = EEPROM.read(direccion + i);
    }
    return cadena;
}

void escribirStringEnEEPROM(int direccion, String cadena) {
    int longitudCadena = cadena.length();
    for (int i = 0; i < longitudCadena; i++) {
        EEPROM.write(direccion + i, cadena[i]);
    }
    EEPROM.write(direccion + longitudCadena, '\0');
    EEPROM.commit();
}

// --- Endpoint para obtener datos actualizados (JSON) ---
void handleGetData() {
    String json = "{";
    json += "\"numPuntos\":" + String(numPuntos) + ",";
    json += "\"ultimoAngulo\":" + String(ultimoAngulo, 1) + ",";
    json += "\"ultimaDistancia\":" + String(ultimaDistancia, 1) + ",";
    json += "\"robotX\":" + String(robotX, 1) + ",";
    json += "\"robotY\":" + String(robotY, 1) + ",";
    json += "\"robotAngulo\":" + String(robotAngulo, 1) + ",";
    json += "\"isMapping\":" +String(isMappingActive ? "true" : "false") + ","; // Nuevo: estado de mapeo
    
    // Usar las coordenadas absolutas ya calculadas
    json += "\"obstaculos\":[";
    for (int i = 0; i < numPuntos; i++) {
        json += "{\"x\":" + String(obstaculosX[i], 1) + ",\"y\":" + String(obstaculosY[i], 1) + "}";
        if (i < numPuntos - 1) json += ",";
    }
    json += "]}";
    
    server.send(200, "application/json", json);
}

// --- Manejadores para los nuevos botones ---
void handleStart() {
    isMappingActive = true;
    server.send(200, "text/plain", "Mapeo iniciado.");
}

void handleReset() {
    isMappingActive = false;
    numPuntos = 0;
    ultimoAngulo = 0;
    ultimaDistancia = 0;
    robotX = 0;
    robotY = 0;
    robotAngulo = 0;
    mejorAngulo = 0;
    mayorDistancia = 0;
    server.send(200, "text/plain", "Mapeo reiniciado.");
}

// --- Página principal (Mapa Cartesiano) ---
void handleRoot() {
    int canvasSize = 600;
    
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Mapa Robot ESP32</title></head><body>";
    html += "<div style='text-align: center;'>";
    html += "<h1>Mapa del Entorno</h1>";
    html += "<button onclick='controlRobot(\"start\")'>Start</button>";
    html += "<button onclick='controlRobot(\"reset\")'>Reset</button>";
    html += "</div>";
    html += "<div style='display: flex; justify-content: center; margin-top: 20px;'>";
    html += "<canvas id='mapaCanvas' width='" + String(canvasSize) + "' height='" + String(canvasSize) + "' style='border: 1px solid black;'></canvas>";
    html += "</div>";

    // JavaScript para mapa cartesiano en tiempo real y control de botones
    html += "<script>";
    html += "let canvas = document.getElementById('mapaCanvas');";
    html += "let ctx = canvas.getContext('2d');";
    html += "let canvasSize = " + String(canvasSize) + ";";
    html += "let trayectoriaRobot = [{x: 0, y: 0}];";
    html += "let escalaPixelPorMM = 0.15;";
    html += "let offsetX = 0, offsetY = 0;";
    html += "let isMapping = false;";

    html += "function mundoACanvas(x, y) {";
    html += "  return {";
    html += "    x: (canvasSize / 2) + (x * escalaPixelPorMM) + offsetX,";
    html += "    y: (canvasSize / 2) - (y * escalaPixelPorMM) + offsetY";
    html += "  };";
    html += "}";

    html += "function dibujarMapa(robotX, robotY, robotAngulo, obstaculos) {";
    html += "  ctx.clearRect(0, 0, canvasSize, canvasSize);";
    html += "  ";
    html += "  let posRobot = mundoACanvas(robotX, robotY);";
    html += "  ";
    // Dibujar trayectoria del robot
    html += "  if(trayectoriaRobot.length > 1) {";
    html += "    ctx.strokeStyle = 'blue';";
    html += "    ctx.lineWidth = 2;";
    html += "    ctx.beginPath();";
    html += "    let primerPunto = mundoACanvas(trayectoriaRobot[0].x, trayectoriaRobot[0].y);";
    html += "    ctx.moveTo(primerPunto.x, primerPunto.y);";
    html += "    for(let i = 1; i < trayectoriaRobot.length; i++) {";
    html += "      let punto = mundoACanvas(trayectoriaRobot[i].x, trayectoriaRobot[i].y);";
    html += "      ctx.lineTo(punto.x, punto.y);";
    html += "    }";
    html += "    ctx.stroke();";
    html += "  }";
    html += "  ";
    // Dibujar obstáculos
    html += "  obstaculos.forEach(obstaculo => {";
    html += "    let pos = mundoACanvas(obstaculo.x, obstaculo.y);";
    html += "    ctx.beginPath();";
    html += "    ctx.arc(pos.x, pos.y, 3, 0, 2 * Math.PI);";
    html += "    ctx.fillStyle = 'red';";
    html += "    ctx.fill();";
    html += "  });";
    html += "  ";
    // Dibujar robot con orientación
    html += "  ctx.beginPath();";
    html += "  ctx.arc(posRobot.x, posRobot.y, 5, 0, 2 * Math.PI);";
    html += "  ctx.fillStyle = 'green';";
    html += "  ctx.fill();";
    html += "  ";
    // Flecha de dirección del robot
    html += "  let anguloRad = robotAngulo * Math.PI / 180;";
    html += "  let flechaX = posRobot.x + Math.cos(anguloRad) * 10;";
    html += "  let flechaY = posRobot.y - Math.sin(anguloRad) * 10;";
    html += "  ctx.beginPath();";
    html += "  ctx.moveTo(posRobot.x, posRobot.y);";
    html += "  ctx.lineTo(flechaX, flechaY);";
    html += "  ctx.strokeStyle = 'orange';";
    html += "  ctx.lineWidth = 2;";
    html += "  ctx.stroke();";
    html += "}";

    html += "function actualizarDatos() {";
    html += "  if(!isMapping) return;";
    html += "  fetch('/get-data')";
    html += "    .then(response => response.json())";
    html += "    .then(data => {";
    html += "      if(!data.isMapping) { isMapping = false; return; }"; // Detener si el servidor dice que no hay mapeo
    html += "      let ultimaPosicion = trayectoriaRobot[trayectoriaRobot.length - 1];";
    html += "      if(Math.abs(ultimaPosicion.x - data.robotX) > 10 || Math.abs(ultimaPosicion.y - data.robotY) > 10) {";
    html += "        trayectoriaRobot.push({x: data.robotX, y: data.robotY});";
    html += "        if(trayectoriaRobot.length > 50) trayectoriaRobot.shift();";
    html += "      }";
    html += "      dibujarMapa(data.robotX, data.robotY, data.robotAngulo, data.obstaculos);";
    html += "    })";
    html += "    .catch(error => { console.error('Error:', error); });";
    html += "}";
    
    html += "function controlRobot(command) {";
    html += "  fetch('/control?action=' + command)";
    html += "    .then(response => {";
    html += "      if (command === 'start') { isMapping = true; }";
    html += "      if (command === 'reset') {";
    html += "          isMapping = false;";
    html += "          trayectoriaRobot = [{x: 0, y: 0}];";
    html += "          dibujarMapa(0, 0, 0, []);"; // Limpiar canvas
    html += "      }";
    html += "      console.log('Comando ' + command + ' enviado.');";
    html += "    });";
    html += "}";

    html += "setInterval(actualizarDatos, 1000);"; // Se actualiza cada segundo
    html += "dibujarMapa(0, 0, 0, []);"; // Dibujar mapa inicial
    html += "</script>";

    html += "</body></html>";
    server.send(200, "text/html", html);
}

// --- Manejo de registro WiFi ---
void handleWifi() {
    String ssid = server.arg("ssid");
    String password = server.arg("password");
    escribirStringEnEEPROM(0, ssid);
    escribirStringEnEEPROM(100, password);
    server.send(200, "text/html", "<html><head><meta http-equiv='refresh' content='2;url=/'></head><body style='background:#181a1b; color:#fff; text-align:center; padding-top:100px;'><h1>Red guardada. Reiniciando...</h1></body></html>");
    delay(2000);
    ESP.restart();
}

// --- Intenta conectar a la última red guardada ---
bool conectarUltimaRed() {
    String ssid = leerStringDeEEPROM(0);
    String password = leerStringDeEEPROM(100);
    if (ssid.length() == 0) return false;
    WiFi.disconnect();
    WiFi.begin(ssid.c_str(), password.c_str());
    int cnt = 0;
    while (WiFi.status() != WL_CONNECTED && cnt < 10) {
        delay(1000);
        cnt++;
    }
    return WiFi.status() == WL_CONNECTED;
}

// --- Inicia Access Point y servidor web ---
void iniciarAP(const char* apSsid, const char* apPassword) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(apSsid, apPassword);
    server.on("/", handleRoot);
    server.on("/get-data", handleGetData);
    server.on("/control", [](){
      String action = server.arg("action");
      if (action == "start") {
        handleStart();
      } else if (action == "reset") {
        handleReset();
      } else {
        server.send(400, "text/plain", "Comando desconocido");
      }
    });
    server.on("/wifi", HTTP_POST, handleWifi);
    server.begin();
    Serial.println("Servidor web iniciado en modo AP.");
}

// --- Lógica principal de conexión WiFi ---
void iniciarConexionWiFi(const char* apSsid, const char* apPassword) {
    EEPROM.begin(512);
    if (conectarUltimaRed()) {
        Serial.println("Conectado a la red guardada.");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
        server.on("/", handleRoot);
        server.on("/get-data", handleGetData);
        server.on("/control", [](){
          String action = server.arg("action");
          if (action == "start") {
            handleStart();
          } else if (action == "reset") {
            handleReset();
          } else {
            server.send(400, "text/plain", "Comando desconocido");
          }
        });
        server.on("/wifi", HTTP_POST, handleWifi);
        server.begin();
    } else {
        Serial.println("No se pudo conectar. Iniciando Access Point para registrar nueva red.");
        iniciarAP(apSsid, apPassword);
    }
}

// --- Loop para servidor web ---
void loopServidorWeb() {
    server.handleClient();
}

#endif // AP_WIFI_EEPROM_MODE_H