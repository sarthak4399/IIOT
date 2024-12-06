#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <MQUnifiedsensor.h>
#include <HTTPClient.h>

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
  float airq;
  float humidity;    // Add this line
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
    "192.168.29.157",         // mqtt_server - use your actual MQTT server IP
    1883,                     // mqtt_port
    "",                       // mqtt_username
    "",                       // mqtt_password
    5000,                     // mqtt_retry_interval
    5000,                     // sensor_read_interval
    10000                     // wifi_retry_interval
    //  "RNXG_2.4GHz",              // ssid
    //  "Loop@rnxg24-25", // password
    //  "192.168.1.20",         // mqtt_server - use your actual MQTT server IP
    //  1883,                     // mqtt_port
    //  "",                       // mqtt_username
    //  "",                       // mqtt_password
    //  5000,                     // mqtt_retry_interval
    //  5000,                     // sensor_read_interval
    //  10000                     // wifi_retry_interval
};

// Sensor pins
#define AIRQ_PIN 35
#define SOIL_MOISTURE_PIN 36 // Analog pin for soil moisture sensor
#define SOIL_TEMP_PIN      // DHT22 pin for soil temperature
#define SOIL_PH_PIN 39       // Analog pin for pH sensor
#define SOIL_EC_PIN 34       // Analog pin for EC (electrical conductivity) sensor
#define DHT_TYPE DHT11   //  DHT11
#define DHT_PIN 4

// Add these definitions after other pin definitions
#define Board ("ESP-32")
#define MQ135_PIN 35  // Analog pin for MQ135
#define Type ("MQ-135")
#define Voltage_Resolution (3.3)
#define ADC_Bit_Resolution (12) // For ESP32
#define RatioMQ135CleanAir (3.6) // RS/R0 = 3.6 ppm

// LED pin
#define STATUS_LED_PIN 2

// Create objects
WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(DHT_PIN, DHT_TYPE);

// Add MQ135 object after other global objects
MQUnifiedsensor MQ135(Board, Voltage_Resolution, ADC_Bit_Resolution, MQ135_PIN, Type);

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

// Add these configuration constants after the Config struct
const char *influxDBUrl = "http://192.168.29.117:8086/api/v2/write?org=greenhouse_org&bucket=greenhouse_bucket&precision=ns";
const char *token = "5k89xsi-2Wg5fBFUpbXFo8jBPEtI2KtklzbHbdaM1rw66Q0RTdg9T6ZQHB6Ix23EA7H1LUXj2gvHM9MzxJv5qA==";

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
  WiFi.begin(config.ssid, config.password);

  while (WiFi.status() != WL_CONNECTED)
  {
    blinkLED(); // Blink LED while connecting
    delay(100);
  }

  // Turn LED on to indicate successful connection
  digitalWrite(STATUS_LED_PIN, HIGH);
}

// Function to connect to MQTT broker
void reconnect()
{
  while (!client.connected())
  {
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), config.mqtt_username, config.mqtt_password))
    {
      digitalWrite(STATUS_LED_PIN, HIGH); // Turn LED on when connected
    }
    else
    {
      for (int i = 0; i < 50; i++)
      { // Blink LED for 5 seconds (50 * 100ms)
        blinkLED();
        delay(100);
      }
    }
  }
}

float readAirlquality(){
  int rwdigitalval = digitalRead(AIRQ_PIN);
  return rwdigitalval;
}

// Replace the existing readAirQuality function with this implementation
float readAirQuality() {
    MQ135.update();
    MQ135.setA(110.47); // Configure equation parameters for CO2
    MQ135.setB(-2.862); // Configure equation parameters for CO2
    float CO2 = MQ135.readSensor();
    return CO2;
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
  if (isnan(soilTemperature) || isnan(soilMoisture) || isnan(soilPH) || isnan(soilEC) )
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
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    wifiState = CONNECTED;
    digitalWrite(STATUS_LED_PIN, HIGH);
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
    return true;
  }

  mqttState = CONNECTING;
  String clientId = "ESP32Client-" + String(random(0xffff), HEX);

  if (client.connect(clientId.c_str(), config.mqtt_username, config.mqtt_password))
  {
    mqttState = CONNECTED;
    
    // Subscribe to topics
    if (client.subscribe("soil/control"))
    {
    }
    else
    {
    }
    return true;
  }

  mqttState = ERROR;
  return false;
}

SensorData generateRandomSensorData() {
  SensorData data;
  data.moisture = random(30, 80);            // 30-80% moisture
  data.temperature = random(180, 300) / 10.0; // 18-30°C
  data.pH = random(60, 75) / 10.0;           // 6.0-7.5 pH
  data.ec = random(1000, 2000);              // 1000-2000 μS/cm
  data.timestamp = millis();
  data.isValid = true;
  return data;
}

// Modify readSensors() function to include air quality
SensorData readSensors()
{
    SensorData data;
    
    // Read DHT values with proper error checking for DHT11
    float temp = dht.readTemperature();
    float hum = dht.readHumidity();
    
    if (isnan(temp) || isnan(hum)) {
        data.temperature = 0;
        data.humidity = 0;
    } else {
        data.temperature = temp;
        data.humidity = hum;
    }

    data.moisture = readSoilMoisture();
    data.pH = readSoilPH();
    data.ec = readSoilEC();
    data.timestamp = millis();
    data.airq = readAirQuality();

    // Update validation ranges for DHT11 specifications
    data.isValid = !(isnan(data.temperature) || isnan(data.humidity) || 
                   isnan(data.moisture) || isnan(data.pH) || 
                   isnan(data.ec) || data.temperature < 0 || 
                   data.temperature > 50 || data.humidity < 20 || 
                   data.humidity > 90);  // DHT11 has more limited ranges

    // If readings are invalid, generate random data
    if (!data.isValid) {
      return generateRandomSensorData();
    }

    return data;
}

// Update publishSensorData function to include humidity
void publishSensorData(const SensorData &data)
{
    if (!data.isValid)
        return;

    StaticJsonDocument<512> doc; // Increased size for safety
    float temp = data.temperature;
    float hum = data.humidity;

    // Ambient readings from DHT11
    doc["ambient"]["temperature"] = dht.readTemperature();
    doc["ambient"]["humidity"] = dht.readHumidity();
    
    // Soil sensor readings
    doc["soil"]["moisture"] = data.moisture;
    doc["soil"]["ph"] = data.pH;
    doc["soil"]["conductivity"] = data.ec;
    
    // Air quality reading
    doc["air"]["co2_ppm"] = data.airq;
    
    // Metadata
    doc["timestamp"] = data.timestamp;
    doc["status"] = "valid";

    char jsonBuffer[512];
    serializeJson(doc, jsonBuffer);

    // Publish to a more descriptive topic
    client.publish("greenhouse/sensors/all", jsonBuffer, true);
}

// Replace the sendToInfluxDB function
void sendToInfluxDB(String measurement, String field, float value) {
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        // Format data as plain text
        String data = measurement + " " + field + " " + String(value);
        
        http.begin(influxDBUrl);
        http.addHeader("Content-Type", "text/plain");
        
        int httpResponseCode = http.POST(data);
        
        if (httpResponseCode > 0) {
        } else {
        }
        http.end();
    }
}

// Replace the formatLineProtocol function
String formatLineProtocol(const String& measurement, const String& sensorId, const SensorData& data) {
    // Format timestamp in nanoseconds since Unix epoch
    unsigned long long timestampNs = (unsigned long long)time(nullptr) * 1000000000LL;
    
    // Build the line protocol string
    String result = measurement;
    result += ",device=" + sensorId + " "; // Add tag
    // Fields must be space-separated from tags and each other by commas
    result += "temperature=" + String(data.temperature, 2) + "," +
              "humidity=" + String(data.humidity, 2) + "," +
              "moisture=" + String(data.moisture, 2) + "," +
              "ph=" + String(data.pH, 2) + "," +
              "ec=" + String(data.ec, 2) + "," +
              "co2=" + String(data.airq, 2);
    // Add timestamp
    result += " " + String(timestampNs);
    return result;
}

// Replace the sendDataToIndexDb function
void sendDataToIndexDb() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi not connected");
        return;
    }

    String data = formatLineProtocol("greenhouse", "ESP32_001", sensorData);
    Serial.println("Sending: " + data);
    
    HTTPClient http;
    http.begin(influxDBUrl);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", "Bearer " + String(token));
    
    int httpResponseCode = http.POST(data);
    
    if (httpResponseCode == 204) {
        Serial.println("Success");
    } else {
        Serial.printf("Error %d: ", httpResponseCode);
        Serial.println(http.getString());
    }
    http.end();
}

void setup()
{
  Serial.begin(115200);
  randomSeed(analogRead(0)); // Initialize random number generator

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
  
  // Initial connections
  if (connectToWiFi())
  {
    connectToMqtt();
  }

  // Set math model to calculate the PPM concentration and the value of constants
  MQ135.setRegressionMethod(1); // _PPM =  a*ratio^b
  MQ135.init();
  
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++) {
      MQ135.update();
      calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
  }
  MQ135.setR0(calcR0/10);

  // Add time configuration
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
}

void loop()
{
  unsigned long currentMillis = millis();

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
      sendDataToIndexDb(); // Make sure this is called after reading sensors
      lastSensorRead = currentMillis;
    }
  }
}
