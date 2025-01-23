// MARIO GARCÍA JIMÉNEZ. GRADO EN INGENIERÍA ELECTRÓNICA INDUSTRIAL
// mariogj.03@uma.es
#include <Arduino.h>
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <WiFi.h>
#include <esp_log.h>
#include <MQTTPubSubClient.h>
#include "Buffer.h"
#include <Wire.h>
#include <MPU6050.h>
#include <math.h>
#include <TinyGPS++.h>
#include <DHT.h>
#include <esp_log.h>

#define ledGreen 15
#define ledRed 5
#define ledBlue 18

MPU6050 mpu;
Buffer buffer; // Tamaño por defecto (15) de la cola
#define BUZZER_PIN 23
#define GPS_BAUDRATE 9600
TinyGPSPlus gps;

#define DHTPIN 4      // Pin al que está conectado el DHT11
#define DHTTYPE DHT11 // Tipo de sensor DHT (DHT11 en este caso)
DHT dht(DHTPIN, DHTTYPE);

const char *ssid = "Redmi Note 8 Pro";
const char *password = "mariomario";
const int TASK_WIFI_SIZE = 8; // Se ha decidido usar unos 8KB porque con 1KB, la pila rebosa.

WiFiClient client;
MQTTPubSubClient mqtt;

const char *broker = "waspbrain.es";
const char *mqtt_username = "sda_g1";
const char *mqtt_password = "curso2425";
const char *client_id = "mgjc";

std::mutex thresholdAcceleration_mutex;

float thresholdAcceleration = 9.5; // Umbral de aceleración m/s^2

int color = 0;

void process_state(int &color)
{
  // Apaga todos los LEDs al inicio para evitar estados residuales
  digitalWrite(ledGreen, LOW);
  digitalWrite(ledRed, LOW);
  digitalWrite(ledBlue, LOW);

  // Enciende el LED correspondiente según el color recibido
  if (color == 0)
  {
    digitalWrite(ledBlue, HIGH);
  }
  else if (color == 1)
  {
    digitalWrite(ledRed, HIGH);
  }
  else if (color == 2)
  {
    digitalWrite(ledGreen, HIGH);
  }
  else
  {
  }
}

void TaskConnectWiFi(void *pvParameters)
{
  WiFi.begin(ssid, password);
  do
  {
    // Si el estado del wifi es aún, no conectado se está reiniciando hasta que lo esté
    if (WiFi.status() != WL_CONNECTED)
    {
      WiFi.begin(ssid, password); // Inicia la conexión
      auto ini = millis();        // Variable que sirve para almacenar un tiempo 'ini'
      auto current = ini;
      color = 1;
      process_state(color);
      ESP_LOGD("TaskConnectWiFi", "Reconectando");
      // Serial.println("Reconectando");
      // Mientras que no se conecte, y la espera sea menor de 5s estaremos printeando '.' como espera.
      while (WiFi.status() != WL_CONNECTED && (current - ini) <= 5000)
      {
        Serial.print(".");
        vTaskDelay(200);
        current = millis();
      }
      ESP_LOGD("TaskConnectWiFi", "");
    }

    if (WiFi.status() == WL_CONNECTED)
    {
      
      process_state(color);
      ESP_LOGD("TaskConnectWiFi", "WiFi connected");
      Buffer ::Message msg;
      msg.topic = "sda/g1/connected/wifi";
      msg.payload = "true"; // Enviar 'true' como texto plano
      // buffer.setMessageToSend(msg);

      if (!mqtt.isConnected())
      {
        color = 0;
        client.connect(broker, 1886);
        mqtt.begin(client);
        mqtt.setKeepAliveTimeout(60); // ESTABA EN 10
        mqtt.setWill("sda/g1/state", "disconnected", true);
        mqtt.connect(client_id, mqtt_username, mqtt_password);

        auto ini = millis(); // Variable que sirve para almacenar un tiempo 'ini'
        auto current = ini;
        while (!mqtt.isConnected() && (current - ini) < 10000) // ESTABA EN 5000
        {
          vTaskDelay(200);
          current = millis();
        }
      }

      if (mqtt.isConnected())
      {
        color = 2;
        process_state(color);
        ESP_LOGD("TaskConnectWiFi", "Mqtt connected");
        msg.topic = "sda/g1/connected/MQTT";
        msg.payload = "true"; 
        //buffer.setMessageToSend(msg);
        mqtt.subscribe("sda/g1/recivido", [&](const String &payload, const size_t size)
                       {
            ESP_LOGD("TaskConnectWiFiReception", "Message received on %s", "sda/g1/recivido");
            Buffer::Message msg;
            msg.topic = "sda/g1/recivido";
            msg.payload = payload.c_str();
            buffer.setReceivedMessage(msg); });

        mqtt.subscribe("sda/g1/encender_led", [&](const String &payload, const size_t size)
                       {
            ESP_LOGD("TaskConnectWiFiReception", "Message received on %s", "sda/g1/encender_led");
            Buffer::Message msg;
            msg.topic = "sda/g1/encender_led";
            msg.payload = payload.c_str();
            buffer.setReceivedMessage(msg); });

        mqtt.subscribe("sda/g1/thresh", [&](const String &payload, const size_t size)
                       {
            ESP_LOGD("TaskConnectWiFiReception", "Message received on %s", "sda/g1/thresh");
            Buffer::Message msg;
            msg.topic = "sda/g1/thresh";
            msg.payload = payload.c_str();
            buffer.setReceivedMessage(msg); });
      }
    }
    if (WiFi.status() == WL_CONNECTED && mqtt.isConnected())
    {
      mqtt.update();
      if (buffer.isSendEmpty() == false)
      {
        Buffer ::Message msg = buffer.getMessageToSend();
        ESP_LOGD("TaskConnectWiFi", "Message to send: %s", msg.topic.c_str());
        mqtt.publish(msg.topic.c_str(), msg.payload.c_str());
      }
    }
    vTaskDelay(500);
  } while (true);
  vTaskDelete(NULL);
}

void ProcessReceivedMessages(void *pvParameters)
{
  while (true)
  {
    if (!buffer.isReceptionEmpty())
    {
      Buffer::Message rec_msg = buffer.getReceivedMessage();
      if (rec_msg.topic == "sda/g1/encender_led")
      {

        bool estado = rec_msg.payload == "true";
        digitalWrite(2, estado ? HIGH : LOW);
        ESP_LOGD("ProcRecMsgs", "LED set to %s", estado ? "ON" : "OFF");
      }
      
      JsonDocument object;
      auto error = deserializeJson(object, rec_msg.payload.c_str());
      if (!error)
      {
        if (rec_msg.topic == "sda/g1/thresh")
        {
          if (object.containsKey("thresh"))
          {
            {
              std::lock_guard<std::mutex> lock(thresholdAcceleration_mutex);
              thresholdAcceleration = object["thresh"].as<float>();
            }
            ESP_LOGD("ProcRecMsgs", "Received thresh %f", thresholdAcceleration);
          }
        }
      }
    }
    vTaskDelay(5000);
  }
}

void SensorMPU6050(void *pvParameters)
{
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection())
  {
    ESP_LOGE("MPU6050", "Error al conectar con el MPU6050");
  }

  pinMode(BUZZER_PIN, OUTPUT); // Configurar el pin del buzzer como salida
  int alarmCounter = 0;        // Contador de activaciones de la alarma

  while (true)
  {
    Buffer ::Message msg;
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    float ax_g = ax / 16384.0 * 9.81;
    float ay_g = ay / 16384.0 * 9.81;
    float az_g = az / 16384.0 * 9.81;
    float totalAcceleration = sqrt(pow(ax_g, 2) + pow(ay_g, 2) + pow(az_g, 2));

    ESP_LOGD("MPU6050", "Total acceleration %f", totalAcceleration);

    float threshAcc;

    {
      std::lock_guard<std::mutex> lock(thresholdAcceleration_mutex);
      threshAcc = thresholdAcceleration;
    }

    if (totalAcceleration > threshAcc)
    {
      alarmCounter++; // Incrementar el contador de alarmas
      ESP_LOGD("MPU6050", "Alarm triggered %d times", alarmCounter);

      // Crear y enviar el mensaje
      JsonDocument doc;
      doc["raw"]["ax_g"] = ax_g;
      doc["raw"]["ay_g"] = ay_g;
      doc["raw"]["az_g"] = az_g;
      doc["acc"] = totalAcceleration;

      String msg_payload;
      serializeJson(doc, msg_payload);

      msg.topic = "sda/g1/alert/accelerometer";
      msg.payload = msg_payload.c_str();
      buffer.setMessageToSend(msg);
    }

    if (alarmCounter >= 3)
    {
      msg.topic = "sda/g1/alert/alarm";
      msg.payload = "true"; // Enviar 'true' como texto plano
      buffer.setMessageToSend(msg);

      // Activar el buzzer con una secuencia de tonos
      for (int i = 0; i < 5; i++)
      {
        tone(BUZZER_PIN, 1000); // Tono alto (1000 Hz)
        vTaskDelay(pdMS_TO_TICKS(200));
        tone(BUZZER_PIN, 500); // Tono bajo (500 Hz)
        vTaskDelay(pdMS_TO_TICKS(200));
      }
      noTone(BUZZER_PIN); // Apagar el buzzer
      alarmCounter = 0;   // Reiniciar el contador de alarmas

      msg.topic = "sda/g1/alert/alarm";
      msg.payload = "false"; // Enviar 'true' como texto plano
      buffer.setMessageToSend(msg);
    }

    vTaskDelay(500);
  }
}

// PRUEBAS:

void SensorNEO6M(void *pvParameters)
{
  Serial2.begin(GPS_BAUDRATE); // Inicializa Serial2 con la tasa de baudios del GPS
  while (true)
  {
    if (Serial2.available() > 0)
    {

      char c = Serial2.read();
      if (gps.encode(c)) // Leer cada carácter del GPS
      {
        ESP_LOGW("NEO6M", "Leyendo caracteres");
        if (gps.location.isValid())
        {
          ESP_LOGD("NEO6M", "Lat, Lng: %f, %f", gps.location.lat(), gps.location.lng());

          JsonDocument doc;
          doc["latitud"] = gps.location.lat();
          doc["longitud"] = gps.location.lng();

          // Serialización de JSON
          String msg_payload;
          serializeJson(doc, msg_payload);

          // Configuración y envío
          Buffer::Message msg;
          msg.topic = "sda/g1/alert/gps";
          msg.payload = msg_payload.c_str();
          buffer.setMessageToSend(msg);
        }
        else
        {
          ESP_LOGW("NEO6M", "Invalid GPS data");
        }
      }
    }
    vTaskDelay(200);
  }
}

void SensorDHT11(void *pvParameters)
{
  dht.begin();
  while (true)
  {
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();
    if (isnan(humidity) || isnan(temperature))
    {
      ESP_LOGE("DHT11", "Error reading sensor");
    }
    else
    {
      JsonDocument doc;
      doc["temperature"] = temperature;
      doc["humidity"] = humidity;

      String msg_payload;
      serializeJson(doc, msg_payload);
      Buffer::Message msg;
      msg.topic = "sda/g1/alert/temperature";
      msg.payload = msg_payload.c_str();
      buffer.setMessageToSend(msg);
      ESP_LOGD("DHT11", "Sensor values have been sent via MQTT");
    }
    vTaskDelay(10000);
  }
}

void setup()
{

  pinMode(2, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(ledRed, OUTPUT);
  pinMode(ledBlue, OUTPUT);
  esp_log_level_set("*", ESP_LOG_NONE);
  esp_log_level_set("ProcRecMsgs", ESP_LOG_DEBUG);
  esp_log_level_set("TaskConnectWiFi", ESP_LOG_DEBUG);
  esp_log_level_set("TaskConnectWiFiReception", ESP_LOG_DEBUG);
  esp_log_level_set("MPU6050", ESP_LOG_DEBUG);
  esp_log_level_set("NEO6M", ESP_LOG_DEBUG);
  esp_log_level_set("DHT11", ESP_LOG_DEBUG);

  Serial.begin(115200);
  delay(10);
  xTaskCreatePinnedToCore(TaskConnectWiFi, "Connect WiFi", TASK_WIFI_SIZE * 1024, NULL, 1, NULL, 0); // Ultimo parámetro en 1
  xTaskCreatePinnedToCore(ProcessReceivedMessages, "ProcRecMsgs", TASK_WIFI_SIZE * 1024, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(SensorMPU6050, "SensorMPU6050", 4 * 1024, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(SensorNEO6M, "SensorNEO6M", 4 * 1024, NULL, 1, NULL, 1); // NEW
  xTaskCreatePinnedToCore(SensorDHT11, "DHT11", 4 * 1024, NULL, 1, NULL, 1);
}

void loop()
{
  delay(1000);
}
