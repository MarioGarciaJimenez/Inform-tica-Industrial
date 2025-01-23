// MARIO GARCÍA JIMÉNEZ. GRADO EN INGENIERÍA ELECTRÓNICA INDUSTRIAL
// mariogj.03@uma.es
/*
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <Wire.h>

// Configuración de WiFi
#define WIFI_SSID "Redmi Note 8 Pro"
#define WIFI_PASSWORD "mariomario"
#define WIFI_CHANNEL 6

#define SLAVE_ADDRESS 0x08

// Configuración de MQTT
const char *MQTT_BROKER_ADRESS = "broker.mqtt.cool";
const uint16_t MQTT_PORT = 1883;
const char *MQTT_CLIENT_NAME = "car_control";

// Pines del sensor ultrasónico
#define TRIG_PIN 18
#define ECHO_PIN 19

// Pines del controlador de motores
#define MOTOR1_1 27
#define MOTOR1_2 14
#define MOTOR2_1 12
#define MOTOR2_2 13

#define LED_GREEN 15
#define LED_RED 2
#define LED_BLUE 4
#define LED_distance 23

// Variables globales
WiFiClient espClient;
PubSubClient mqttClient(espClient);
String content = ""; // Contenido del mensaje recibido

// Configuración de PWM para el ESP32
const int freq = 5000;    // Frecuencia PWM
const int resolution = 8; // Resolución de 8 bits (0-255)
const int channel1 = 0;   // Canal PWM para MOTOR1_1
const int channel2 = 1;   // Canal PWM para MOTOR1_2
const int channel3 = 2;   // Canal PWM para MOTOR2_1
const int channel4 = 3;   // Canal PWM para MOTOR2_2
volatile bool activated = false;

int speed = 80; // Velocidad inicial (máxima)
int32_t dist = 0;


// Configuración de los pines PWM
void setupPWM()
{
  ledcSetup(channel1, freq, resolution);
  ledcSetup(channel2, freq, resolution);
  ledcSetup(channel3, freq, resolution);
  ledcSetup(channel4, freq, resolution);

  ledcAttachPin(MOTOR1_1, channel1);
  ledcAttachPin(MOTOR1_2, channel2);
  ledcAttachPin(MOTOR2_1, channel3);
  ledcAttachPin(MOTOR2_2, channel4);
}

// Funciones de control de motores
void moverMotor(int m1_pin1, int m1_pin2, int m2_pin1, int m2_pin2)
{
  ledcWrite(channel1, m1_pin1 ? speed : 0);
  ledcWrite(channel2, m1_pin2 ? speed : 0);
  ledcWrite(channel3, m2_pin1 ? speed : 0);
  ledcWrite(channel4, m2_pin2 ? speed : 0);
}

void driveForward()
{
  moverMotor(1, 0, 1, 0); // Adelante
}

void driveBackward()
{
  moverMotor(0, 1, 0, 1); // Atrás
}

void turnLeft()
{
  moverMotor(1, 0, 0, 1); // Girar a la izquierda
}

void turnRight()
{
  moverMotor(0, 1, 1, 0); // Girar a la derecha
}

void stopMotors()
{
  moverMotor(0, 0, 0, 0); // Detener todos los motores
}

// Función para medir distancia con el sensor ultrasónico
long medirDistancia()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2; // Convertir a cm
  return distance;
}

// Control basado en distancia
void controlarDistancia()
{
  if (!activated) // Solo en modo manual
  {
    long distancia = medirDistancia();
    if (distancia < 20)
    {
      //Serial.println("shit");
      digitalWrite(LED_distance, HIGH); // Enciende el LED rojo
    }
    else
    {
      digitalWrite(LED_distance, LOW); // Apaga el LED rojo
    }
  }
}

void control()
{
  if (activated)
  {
    if (dist > 100)
    {
      turnRight();
      Serial.print(dist);
    }
    else if (dist < 60)
    {
      turnLeft();
    }
    else
    {
      driveForward();
    }
  }
}

void receiveEvent(int bytes)
{
  while (Wire.available())
  {
    dist = Wire.read(); // Leer un byte de los datos recibidos
  }

  Serial.print("Distancia recibida: ");
  Serial.println(dist);
  control();
}

// Función para manejar los mensajes MQTT recibidos
void OnMqttReceived(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Mensaje recibido en ");
  Serial.print(topic);
  Serial.print(": ");

  content = "";
  for (unsigned int i = 0; i < length; i++)
  {
    content += (char)payload[i];
  }
  Serial.println(content);

  if (String(topic) == "car/move")
  {
    if (content == "forward")
      driveForward();
    else if (content == "backward")
      driveBackward();
    else if (content == "left")
      turnLeft();
    else if (content == "right")
      turnRight();
    else if (content == "stop")
      stopMotors();
  }

  if (String(topic) == "car/speed")
  {
    int newSpeed = content.toInt();
    speed = constrain(newSpeed, 0, 255); // Limitar el rango de velocidad
    Serial.print("Velocidad ajustada a: ");
    Serial.println(speed);
  }

  if (String(topic) == "car/automatic")
  {
    if (content == "activated")
    {
      activated = true;
      Wire.begin(SLAVE_ADDRESS);    // Iniciar I2C como esclavo
      Wire.onReceive([](int bytes) {
        // Manejar datos recibidos
      });
    }
    if (content == "desactivated")
    {
      activated = false;
      Wire.end();
    }
  }
}

// Conexión a WiFi
void ConnectWiFi()
{
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL);
  Serial.print("Conectando a WiFi: ");
  Serial.print(WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED)
  {
    digitalWrite(LED_BLUE, HIGH);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, LOW);
    delay(100);
    Serial.print(".");
  }
  Serial.println("\nConectado a WiFi!");
  Serial.print("Dirección IP: ");
  Serial.println(WiFi.localIP());
}

// Configuración inicial de MQTT
void InitMqtt()
{
  mqttClient.setServer(MQTT_BROKER_ADRESS, MQTT_PORT);
  mqttClient.setCallback(OnMqttReceived);
}

// Conexión al servidor MQTT
void ConnectMqtt()
{
  while (!mqttClient.connected())
  {
    Serial.println("Intentando conectar al servidor MQTT...");
    if (mqttClient.connect(MQTT_CLIENT_NAME))
    {
      digitalWrite(LED_GREEN, HIGH);
      digitalWrite(LED_BLUE, LOW);
      digitalWrite(LED_RED, LOW);

      Serial.println("¡Conexión exitosa al servidor MQTT!");
      mqttClient.subscribe("car/move");      // Comandos de movimiento
      mqttClient.subscribe("car/speed");     // Comando de velocidad
      mqttClient.subscribe("car/automatic"); // Comando de modo automático o manual
    }
    else
    {
      Serial.print("Falló la conexión, estado=");
      Serial.println(mqttClient.state());
      delay(5000);
    }
  }
}

// Configuración inicial
void setup()
{
  Serial.begin(115200);

  // Configuración de pines de motores como salida
  pinMode(MOTOR1_1, OUTPUT);
  pinMode(MOTOR1_2, OUTPUT);
  pinMode(MOTOR2_1, OUTPUT);
  pinMode(MOTOR2_2, OUTPUT);

  // Configuración de pines de LEDs y sensor ultrasónico
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_distance,OUTPUT);
  setupPWM();
  ConnectWiFi();
  InitMqtt();
}

// Bucle principal
void loop()
{
  if (!mqttClient.connected())
  {
    Serial.println("MQTT desconectado. Intentando reconectar...");
    ConnectMqtt();
  }
  //control();
  mqttClient.loop();
  //Serial.println(dist);
  controlarDistancia(); // Llama al control del sensor ultrasónico
}


*/










































// JUJUJU

#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <Wire.h>

// Configuración de WiFi
#define WIFI_SSID "Redmi Note 8 Pro"
#define WIFI_PASSWORD "mariomario"
#define WIFI_CHANNEL 6

#define SLAVE_ADDRESS 0x08

// Configuración de MQTT
const char *MQTT_BROKER_ADRESS = "broker.mqtt.cool";
const uint16_t MQTT_PORT = 1883;
const char *MQTT_CLIENT_NAME = "car_control";
int32_t dist = 0;
// Pines del controlador de motores
#define MOTOR1_1 27
#define MOTOR1_2 14
#define MOTOR2_1 12
#define MOTOR2_2 13

#define led_green 15
#define led_red 2
#define led_blue 4
#define LED_distance 23

// Pines del sensor ultrasónico
#define TRIG_PIN 18
#define ECHO_PIN 19


// Variables globales
WiFiClient espClient;
PubSubClient mqttClient(espClient);
String content = ""; // Contenido del mensaje recibido

// Configuración de PWM para el ESP32
const int freq = 5000;    // Frecuencia PWM
const int resolution = 8; // Resolución de 8 bits (0-255)
const int channel1 = 0;   // Canal PWM para MOTOR1_1
const int channel2 = 1;   // Canal PWM para MOTOR1_2
const int channel3 = 2;   // Canal PWM para MOTOR2_1
const int channel4 = 3;   // Canal PWM para MOTOR2_2
volatile bool activated = false;

int speed = 80; // Velocidad inicial (máxima)

// Configuración de los pines PWM
void setupPWM()
{
  ledcSetup(channel1, freq, resolution);
  ledcSetup(channel2, freq, resolution);
  ledcSetup(channel3, freq, resolution);
  ledcSetup(channel4, freq, resolution);

  ledcAttachPin(MOTOR1_1, channel1);
  ledcAttachPin(MOTOR1_2, channel2);
  ledcAttachPin(MOTOR2_1, channel3);
  ledcAttachPin(MOTOR2_2, channel4);
}

// Funciones de control de motores
void moverMotor(int m1_pin1, int m1_pin2, int m2_pin1, int m2_pin2)
{
  ledcWrite(channel1, m1_pin1 ? speed : 0);
  ledcWrite(channel2, m1_pin2 ? speed : 0);
  ledcWrite(channel3, m2_pin1 ? speed : 0);
  ledcWrite(channel4, m2_pin2 ? speed : 0);
}

void driveForward()
{
  moverMotor(1, 0, 1, 0); // Adelante
}

void driveBackward()
{
  moverMotor(0, 1, 0, 1); // Atrás
}

void turnLeft()
{
  moverMotor(1, 0, 0, 1); // Girar a la izquierda
}

void turnRight()
{
  moverMotor(0, 1, 1, 0); // Girar a la derecha
}

void stopMotors()
{
  moverMotor(0, 0, 0, 0); // Detener todos los motores
}

// dist, se gestiona así porque se pasan en 1 byte.

void control()
{
  if (activated)
  {
    if (dist > 100)
    {
      turnRight();
    }
    else if (dist < 60)
    {
      turnLeft();
    }
    else
    {
      driveForward();
    }
  }
}

void receiveEvent(int bytes)
{
  while (Wire.available())
  {
    dist = Wire.read(); // Leer un byte de los datos recibidos
  }

  Serial.print("Distancia recibida: ");
  Serial.println(dist);
  control();
}

void OnMqttReceived(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Mensaje recibido en ");
  Serial.print(topic);
  Serial.print(": ");

  content = "";
  for (unsigned int i = 0; i < length; i++)
  {
    content += (char)payload[i];
  }
  Serial.println(content);

  if (String(topic) == "car/move")
  {
    if (content == "forward")
      driveForward();
    else if (content == "backward")
      driveBackward();
    else if (content == "left")
      turnLeft();
    else if (content == "right")
      turnRight();
    else if (content == "stop")
      stopMotors();
  }

  if (String(topic) == "car/speed")
  {
    int newSpeed = content.toInt();
    speed = constrain(newSpeed, 0, 255); // Limitar el rango de velocidad
    Serial.print("Velocidad ajustada a: ");
    Serial.println(speed);
  }

  if (String(topic) == "car/automatic")
  {
    if (content == "activated")
    {
      activated = true;
      Wire.begin(SLAVE_ADDRESS);    // Iniciar I2C como esclavo
      Wire.onReceive(receiveEvent); // Función para manejar los datos recibidos
    }
    if (content == "desactivated")
    {
      Wire.end();
    }
  }
}

// Conexión a WiFi
void ConnectWiFi()
{
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL);
  Serial.print("Conectando a WiFi: ");
  Serial.print(WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED)
  {
    digitalWrite(led_blue, HIGH);
    digitalWrite(led_green, LOW);
    digitalWrite(led_red, LOW);
    delay(100);
    Serial.print(".");
  }
  Serial.println("\nConectado a WiFi!");
  Serial.print("Dirección IP: ");
  Serial.println(WiFi.localIP());
}

// Configuración inicial de MQTT
void InitMqtt()
{
  mqttClient.setServer(MQTT_BROKER_ADRESS, MQTT_PORT);
  mqttClient.setCallback(OnMqttReceived);
}

// Conexión al servidor MQTT
void ConnectMqtt()
{
  while (!mqttClient.connected())
  {
    Serial.println("Intentando conectar al servidor MQTT...");
    if (mqttClient.connect(MQTT_CLIENT_NAME))
    {
      digitalWrite(led_green, HIGH);
      digitalWrite(led_blue, LOW);
      digitalWrite(led_red, LOW);
      
      Serial.println("¡Conexión exitosa al servidor MQTT!");
      mqttClient.subscribe("car/move");      // Comandos de movimiento
      mqttClient.subscribe("car/speed");     // Comando de velocidad
      mqttClient.subscribe("car/automatic"); // Comando de si es modo automatico o manual
    }
    else
    {
      Serial.print("Falló la conexión, estado=");
      Serial.println(mqttClient.state());
      delay(5000);
    }
  }
  pinMode(led_red,HIGH);
  pinMode(led_blue,LOW);
  pinMode(led_green,LOW);
}

long medirDistancia()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2; // Convertir a cm
  return distance;
}

// Control basado en distancia
void controlarDistancia()
{
  if (activated) // Solo en automatico
  {
    long distancia = medirDistancia();
    if (distancia < 20)
    {
      //Serial.println("shit");
      digitalWrite(LED_distance, HIGH); // Enciende el LED rojo
      stopMotors();
    }
    else
    {
      digitalWrite(LED_distance, LOW); // Apaga el LED rojo
    }
  }
}



// Configuración inicial
void setup()
{
  Serial.begin(115200);

  // Configuración de pines de motores como salida
  pinMode(MOTOR1_1, OUTPUT);
  pinMode(MOTOR1_2, OUTPUT);
  pinMode(MOTOR2_1, OUTPUT);
  pinMode(MOTOR2_2, OUTPUT);

  pinMode(led_green, OUTPUT);
  pinMode(led_red, OUTPUT);
  pinMode(led_blue, OUTPUT);
  pinMode(LED_distance,OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  setupPWM();
  ConnectWiFi();
  InitMqtt();
}

// Bucle principal
void loop()
{
  if (!mqttClient.connected())
  {
    Serial.println("MQTT desconectado. Intentando reconectar...");
    ConnectMqtt();
  }
  mqttClient.loop();
  controlarDistancia(); 
}


/////////////////////////////////////////////////////////



/*
// MARIO GARCÍA JIMÉNEZ. GRADO EN INGENIERÍA ELECTRÓNICA INDUSTRIAL
// mariogj.03@uma.es
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>

// Configuración de WiFi
#define WIFI_SSID "Redmi Note 8 Pro"
#define WIFI_PASSWORD "mariomario"
#define WIFI_CHANNEL 6

// Configuración de MQTT
const char *MQTT_BROKER_ADRESS = "broker.mqtt.cool";
const uint16_t MQTT_PORT = 1883;
const char *MQTT_CLIENT_NAME = "car_control";

// Pines del controlador de motores
#define MOTOR1_1 27
#define MOTOR1_2 14
#define MOTOR2_1 12
#define MOTOR2_2 13

// Variables globales
WiFiClient espClient;
PubSubClient mqttClient(espClient);
String content = ""; // Contenido del mensaje recibido

// Configuración de PWM para el ESP32
const int freq = 5000;    // Frecuencia PWM
const int resolution = 8; // Resolución de 8 bits (0-255)
const int channel1 = 0;   // Canal PWM para MOTOR1_1
const int channel2 = 1;   // Canal PWM para MOTOR1_2
const int channel3 = 2;   // Canal PWM para MOTOR2_1
const int channel4 = 3;   // Canal PWM para MOTOR2_2

int speed = 255; // Velocidad inicial (máxima)



// Configuración de los pines PWM
void setupPWM()
{
  ledcSetup(channel1, freq, resolution);
  ledcSetup(channel2, freq, resolution);
  ledcSetup(channel3, freq, resolution);
  ledcSetup(channel4, freq, resolution);

  ledcAttachPin(MOTOR1_1, channel1);
  ledcAttachPin(MOTOR1_2, channel2);
  ledcAttachPin(MOTOR2_1, channel3);
  ledcAttachPin(MOTOR2_2, channel4);
}

// Funciones de control de motores
void moverMotor(int m1_pin1, int m1_pin2, int m2_pin1, int m2_pin2)
{
  ledcWrite(channel1, m1_pin1 ? speed : 0);
  ledcWrite(channel2, m1_pin2 ? speed : 0);
  ledcWrite(channel3, m2_pin1 ? speed : 0);
  ledcWrite(channel4, m2_pin2 ? speed : 0);
}

void driveForward()
{
  moverMotor(1, 0, 1, 0); // Adelante
}

void driveBackward()
{
  moverMotor(0, 1, 0, 1); // Atrás
}

void turnLeft()
{
  moverMotor(1, 0, 0, 1); // Girar a la izquierda
}

void turnRight()
{
  moverMotor(0, 1, 1, 0); // Girar a la derecha
}

void stopMotors()
{
  moverMotor(0, 0, 0, 0); // Detener todos los motores
}

void OnMqttReceived(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Mensaje recibido en ");
  Serial.print(topic);
  Serial.print(": ");

  content = "";
  for (unsigned int i = 0; i < length; i++)
  {
    content += (char)payload[i];
  }
  Serial.println(content);

  if (String(topic) == "car/move")
  {
    if (content == "forward")
      driveForward();
    else if (content == "backward")
      driveBackward();
    else if (content == "left")
      turnLeft();
    else if (content == "right")
      turnRight();
    else if (content == "stop")
      stopMotors();
  }

  if (String(topic) == "car/speed")
  {
    int newSpeed = content.toInt();
    speed = constrain(newSpeed, 0, 255); // Limitar el rango de velocidad
    Serial.print("Velocidad ajustada a: ");
    Serial.println(speed);
  }

  if(String(topic) == "car/automatic")
  {

  }


}

// Conexión a WiFi
void ConnectWiFi()
{
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL);
  Serial.print("Conectando a WiFi: ");
  Serial.print(WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
    Serial.print(".");
  }
  Serial.println("\nConectado a WiFi!");
  Serial.print("Dirección IP: ");
  Serial.println(WiFi.localIP());
}

// Configuración inicial de MQTT
void InitMqtt()
{
  mqttClient.setServer(MQTT_BROKER_ADRESS, MQTT_PORT);
  mqttClient.setCallback(OnMqttReceived);
}

// Conexión al servidor MQTT
void ConnectMqtt()
{
  while (!mqttClient.connected())
  {
    Serial.println("Intentando conectar al servidor MQTT...");
    if (mqttClient.connect(MQTT_CLIENT_NAME))
    {
      Serial.println("¡Conexión exitosa al servidor MQTT!");
      mqttClient.subscribe("car/move");  // Comandos de movimiento
      mqttClient.subscribe("car/speed"); // Comando de velocidad
    }
    else
    {
      Serial.print("Falló la conexión, estado=");
      Serial.println(mqttClient.state());
      delay(5000);
    }
  }
}

// Configuración inicial
void setup()
{
  Serial.begin(115200);

  // Configuración de pines de motores como salida
  pinMode(MOTOR1_1, OUTPUT);
  pinMode(MOTOR1_2, OUTPUT);
  pinMode(MOTOR2_1, OUTPUT);
  pinMode(MOTOR2_2, OUTPUT);

  // Configuración de PWM
  setupPWM();
  // Conectar a WiFi y configurar MQTT
  ConnectWiFi();
  InitMqtt();
}

// Bucle principal
void loop()
{
  if (!mqttClient.connected())
  {
    Serial.println("MQTT desconectado. Intentando reconectar...");
    ConnectMqtt();
  }
  mqttClient.loop();
}


*/

/*

#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>

// Configuración de WiFi
#define WIFI_SSID "Redmi Note 8 Pro"
#define WIFI_PASSWORD "mariomario"
#define WIFI_CHANNEL 6

// Configuración de MQTT
const char *MQTT_BROKER_ADRESS = "broker.mqtt.cool";
const uint16_t MQTT_PORT = 1883;
const char *MQTT_CLIENT_NAME = "";

// Pines del controlador de motores
#define MOTOR1_1 27
#define MOTOR1_2 14
#define MOTOR2_1 12
#define MOTOR2_2 13

// Variables
WiFiClient espClient;
PubSubClient mqttClient(espClient);
String payload;
String content = "";


// Función para conectar al WiFi
void ConnectWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL);
  Serial.print("Connecting to WiFi ");
  Serial.print(WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  Serial.println(" Connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// Inicializar MQTT

// Control de motores
void driveForward() {
  digitalWrite(MOTOR1_1, HIGH);
  digitalWrite(MOTOR1_2, LOW);
  digitalWrite(MOTOR2_1, HIGH);
  digitalWrite(MOTOR2_2, LOW);
}

void driveBackward() {
  digitalWrite(MOTOR1_1, LOW);
  digitalWrite(MOTOR1_2, HIGH);
  digitalWrite(MOTOR2_1, LOW);
  digitalWrite(MOTOR2_2, HIGH);
}

void turnLeft() {
  digitalWrite(MOTOR1_1, LOW);
  digitalWrite(MOTOR1_2, HIGH);
  digitalWrite(MOTOR2_1, HIGH);
  digitalWrite(MOTOR2_2, LOW);
}

void turnRight() {
  digitalWrite(MOTOR1_1, HIGH);
  digitalWrite(MOTOR1_2, LOW);
  digitalWrite(MOTOR2_1, LOW);
  digitalWrite(MOTOR2_2, HIGH);
}

void stopMotors() {
  digitalWrite(MOTOR1_1, LOW);
  digitalWrite(MOTOR1_2, LOW);
  digitalWrite(MOTOR2_1, LOW);
  digitalWrite(MOTOR2_2, LOW);
}


// Función para manejar los mensajes MQTT recibidos
void OnMqttReceived(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message received on ");
  Serial.print(topic);
  Serial.print(": ");

  content = "";
  for (unsigned int i = 0; i < length; i++) {
    content += (char)payload[i];
  }
  Serial.println(content);

  // Interpretar el comando
  if (content == "forward") {
    driveForward();
  } else if (content == "backward") {
    driveBackward();
  } else if (content == "left") {
    turnLeft();
  } else if (content == "right") {
    turnRight();
  } else if (content == "stop") {
    stopMotors();
  }
}

void InitMqtt() {
  mqttClient.setServer(MQTT_BROKER_ADRESS, MQTT_PORT);
  mqttClient.setCallback(OnMqttReceived);
}

// Conexión al servidor MQTT
void ConnectMqtt() {
  while (!mqttClient.connected()) {
    Serial.println("Attempting MQTT connection...");
    if (mqttClient.connect(MQTT_CLIENT_NAME)) {
      Serial.println("Connected to MQTT broker!");
      mqttClient.subscribe("car");
    } else {
      Serial.print("Connection failed, state=");
      Serial.println(mqttClient.state());
      delay(5000);
    }
  }
}


// Configuración inicial
void setup() {
  Serial.begin(115200);

  // Configurar pines de motores
  pinMode(MOTOR1_1, OUTPUT);
  pinMode(MOTOR1_2, OUTPUT);
  pinMode(MOTOR2_1, OUTPUT);
  pinMode(MOTOR2_2, OUTPUT);

  ConnectWiFi();
  InitMqtt();
}

// Bucle principal
void loop() {
  if (!mqttClient.connected()) {
    Serial.print("MQTT connection failed, state=");
    Serial.println(mqttClient.state());
    ConnectMqtt();
  }
  mqttClient.loop();
}

*/