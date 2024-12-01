#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <DHT.h>

// Configuration structure
struct Config
{
  // WiFi settings
  const char *ssid;
  const char *password;
  // MQTT settings
  const char *mqtt_server;
  int mqtt_port;
  const char *mqtt_username;
  const char *mqtt_password;
  // Timing settings
  unsigned long mqtt_retry_interval;
  unsigned long sensor_read_interval;
  unsigned long wifi_retry_interval;
};

// Sensor data structure
struct SensorData
{
  float moisture;
  float temperature;
  float pH;
  float ec;
  unsigned long timestamp;
  bool isValid;
};

// Connection state enum
enum ConnectionState
{
  DISCONNECTED,
  CONNECTING,
  CONNECTED,
  ERROR
};

// Global configuration
const Config config = {
    "Desert2.4",              // ssid
    "Kmarfi@orangedesert@21", // password
    "192.168.29.236",         // mqtt_server - use your actual MQTT server IP
    1883,                     // mqtt_port
    "",                       // mqtt_username
    "",                       // mqtt_password
    5000,                     // mqtt_retry_interval
    5000,                     // sensor_read_interval
    10000                     // wifi_retry_interval
};

// Sensor pins
#define SOIL_MOISTURE_PIN 36 // Analog pin for soil moisture sensor
#define SOIL_TEMP_PIN 4      // DHT22 pin for soil temperature
#define SOIL_PH_PIN 39       // Analog pin for pH sensor
#define SOIL_EC_PIN 34       // Analog pin for EC (electrical conductivity) sensor
#define DHT_TYPE DHT22

// LED pin
#define STATUS_LED_PIN 2

// Create objects
WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(SOIL_TEMP_PIN, DHT_TYPE);

// Variables for sensor readings
float soilMoisture;
float soilTemperature;
float soilPH;
float soilEC;
unsigned long lastMsg = 0;
const int msgInterval = 5000; // Send data every 5 seconds

bool validReadings = false;

// Global state variables
ConnectionState wifiState = DISCONNECTED;
ConnectionState mqttState = DISCONNECTED;
SensorData sensorData = {0};
unsigned long lastMqttRetry = 0;
unsigned long lastSensorRead = 0;

// Function to blink the LED
void blinkLED()
{
  static unsigned long lastBlink = 0;
  static bool ledState = false;

  if (millis() - lastBlink > 500)
  { // Blink every 500ms
    ledState = !ledState;
    digitalWrite(STATUS_LED_PIN, ledState);
    lastBlink = millis();
  }
}

// Function to connect to Wi-Fi
void setup_wifi()
{
  delay(10);
  Serial.println("Connecting to WiFi");
  WiFi.begin(config.ssid, config.password);

  while (WiFi.status() != WL_CONNECTED)
  {
    blinkLED(); // Blink LED while connecting
    delay(100);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Turn LED on to indicate successful connection
  digitalWrite(STATUS_LED_PIN, HIGH);
}

// Function to connect to MQTT broker
void reconnect()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), config.mqtt_username, config.mqtt_password))
    {
      Serial.println("connected");
      digitalWrite(STATUS_LED_PIN, HIGH); // Turn LED on when connected
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.print("MQTT state: ");
      Serial.println(client.state());

      Serial.println(" retrying in 5 seconds");
      for (int i = 0; i < 50; i++)
      { // Blink LED for 5 seconds (50 * 100ms)
        blinkLED();
        delay(100);
      }
    }
  }
}

// Function to read soil moisture
float readSoilMoisture()
{
  int rawValue = analogRead(SOIL_MOISTURE_PIN);
  return map(rawValue, 4095, 0, 0, 100);
}

// Function to read soil pH
float readSoilPH()
{
  int rawValue = analogRead(SOIL_PH_PIN);
  float voltage = (rawValue * 3.3) / 4095.0;
  return (3.3 * voltage); // Adjust based on calibration
}

// Function to read soil EC
float readSoilEC()
{
  int rawValue = analogRead(SOIL_EC_PIN);
  float voltage = (rawValue * 3.3) / 4095.0;
  return voltage * 1000; // Convert to μS/cm
}

// Function to validate sensor readings
bool validateReadings()
{
  if (isnan(soilTemperature) || isnan(soilMoisture) || isnan(soilPH) || isnan(soilEC))
  {
    return false;
  }
  if (soilMoisture < 0 || soilMoisture > 100 ||
      soilTemperature < -10 || soilTemperature > 60 ||
      soilPH < 0 || soilPH > 14)
  {
    return false;
  }
  return true;
}

// MQTT callback function
void mqtt_callback(char *topic, byte *payload, unsigned int length)
{
  // Handle incoming messages
  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0';
  Serial.printf("Message received on topic %s: %s\n", topic, message);
}

bool connectToWiFi()
{
  if (WiFi.status() == WL_CONNECTED)
    return true;

  wifiState = CONNECTING;
  digitalWrite(STATUS_LED_PIN, LOW);

  WiFi.begin(config.ssid, config.password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20)
  {
    blinkLED();
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    wifiState = CONNECTED;
    digitalWrite(STATUS_LED_PIN, HIGH);
    Serial.printf("\nConnected to WiFi. IP: %s\n", WiFi.localIP().toString().c_str());
    return true;
  }

  wifiState = ERROR;
  return false;
}

const char* getMQTTStateString(int state) {
    switch (state) {
        case -4: return "MQTT_CONNECTION_TIMEOUT";
        case -3: return "MQTT_CONNECTION_LOST";
        case -2: return "MQTT_CONNECT_FAILED";
        case -1: return "MQTT_DISCONNECTED";
        case 0: return "MQTT_CONNECTED";
        case 1: return "MQTT_CONNECT_BAD_PROTOCOL";
        case 2: return "MQTT_CONNECT_BAD_CLIENT_ID";
        case 3: return "MQTT_CONNECT_UNAVAILABLE";
        case 4: return "MQTT_CONNECT_BAD_CREDENTIALS";
        case 5: return "MQTT_CONNECT_UNAUTHORIZED";
        default: return "MQTT_UNKNOWN";
    }
}

bool connectToMqtt()
{
  if (client.connected())
  {
    Serial.println("MQTT already connected");
    return true;
  }

  mqttState = CONNECTING;
  String clientId = "ESP32Client-" + String(random(0xffff), HEX);
  
  Serial.printf("Attempting MQTT connection to %s:%d...\n", config.mqtt_server, config.mqtt_port);
  Serial.printf("ClientID: %s\n", clientId.c_str());

  if (client.connect(clientId.c_str(), config.mqtt_username, config.mqtt_password))
  {
    mqttState = CONNECTED;
    Serial.println("MQTT Connected successfully");
    
    // Subscribe to topics
    if (client.subscribe("soil/control"))
    {
      Serial.println("Subscribed to soil/control");
    }
    else
    {
      Serial.println("Failed to subscribe to soil/control");
    }
    return true;
  }

  mqttState = ERROR;
  Serial.printf("MQTT Connection failed, state: %s\n", getMQTTStateString(client.state()));
  return false;
}

SensorData readSensors()
{
  SensorData data;
  data.moisture = readSoilMoisture();
  data.temperature = dht.readTemperature();
  data.pH = readSoilPH();
  data.ec = readSoilEC();
  data.timestamp = millis();

  // Validate readings
  data.isValid = !(isnan(data.temperature) || isnan(data.moisture) ||
                   isnan(data.pH) || isnan(data.ec) ||
                   data.moisture < 0 || data.moisture > 100 ||
                   data.temperature < -10 || data.temperature > 60 ||
                   data.pH < 0 || data.pH > 14);

  return data;
}

void publishSensorData(const SensorData &data)
{
  if (!data.isValid)
  {
    Serial.println("Invalid sensor data, skipping publication");
    return;
  }

  StaticJsonDocument<200> doc;
  doc["moisture"] = data.moisture;
  doc["temperature"] = data.temperature;
  doc["pH"] = data.pH;
  doc["ec"] = data.ec;
  doc["timestamp"] = data.timestamp;

  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);

  if (client.publish("soil/sensors", jsonBuffer, true))
  { // true for retained message
    Serial.println("Published: " + String(jsonBuffer));
  }
  else
  {
    Serial.println("Failed to publish message");
  }
}

void setup()
{
  Serial.begin(115200);

  // Initialize pins
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(SOIL_MOISTURE_PIN, INPUT);
  pinMode(SOIL_PH_PIN, INPUT);
  pinMode(SOIL_EC_PIN, INPUT);

  // Initialize sensors
  dht.begin();

  // Setup MQTT client
  client.setServer(config.mqtt_server, config.mqtt_port);
  client.setCallback(mqtt_callback);
  client.setKeepAlive(60);
  client.setSocketTimeout(10);
  
  // Print configuration
  Serial.printf("MQTT Server: %s:%d\n", config.mqtt_server, config.mqtt_port);
  
  // Initial connections
  if (connectToWiFi())
  {
    Serial.println("Attempting initial MQTT connection...");
    connectToMqtt();
  }
}

void loop()
{
  unsigned long currentMillis = millis();
  static unsigned long lastDebugPrint = 0;

  // Print debug info every 5 seconds
  if (currentMillis - lastDebugPrint >= 5000)
  {
    lastDebugPrint = currentMillis;
    Serial.printf("WiFi State: %d, MQTT State: %d (%s)\n", 
        wifiState, mqttState, getMQTTStateString(client.state()));
  }

  // Handle WiFi connection
  if (WiFi.status() != WL_CONNECTED)
  {
    if (currentMillis - lastMqttRetry >= config.wifi_retry_interval)
    {
      connectToWiFi();
      lastMqttRetry = currentMillis;
    }
  }
  // Handle MQTT connection
  else if (!client.connected())
  {
    if (currentMillis - lastMqttRetry >= config.mqtt_retry_interval)
    {
      connectToMqtt();
      lastMqttRetry = currentMillis;
    }
  }
  // Normal operation
  else
  {
    client.loop();
    // Read and publish sensor data
    if (currentMillis - lastSensorRead >= config.sensor_read_interval)
    {
      sensorData = readSensors();
      publishSensorData(sensorData);
      lastSensorRead = currentMillis;
    }
  }
}
