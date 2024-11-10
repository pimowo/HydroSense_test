#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <ArduinoHA.h>

// WiFi login data
const char* ssid = "pimowo";
const char* password = "ckH59LRZQzCDQFiUgj";

// MQTT login data
const char* mqtt_server = "192.168.1.14";
const int mqtt_port = 1883;
const char* mqtt_user = "hydrosense";
const char* mqtt_password = "hydrosense";

// Pins
#define TRIG_PIN D6
#define ECHO_PIN D7
#define SENSOR_PIN D5  // Water level sensor
#define PUMP_PIN D1    // Pump
#define BUZZER_PIN D2  // Buzzer
#define BUTTON_PIN D3  // Alarm reset button

// Creating a Wi-Fi client object for communication
WiFiClient espClient;
// Initializing the Home Assistant device named "HydroSense"
HADevice device("HydroSense");
// Initializing the MQTT object using the Wi-Fi client and device
HAMqtt haMqtt(espClient, device);
// Sensors
HASensor waterSensor("water");  // Water sensor - informs Home Assistant about the current state of the water sensor (on/off).
HASensor pumpSensor("pump");  // Pump control - allows turning the pump on and off via Home Assistant.
HASensor waterLevelPercent("water_level_percent");  // Tank fill percentage - informs Home Assistant about the current water level in the tank in percent.
HASensor waterVolumeLiters("water_volume_liters");  // Water volume in liters - informs Home Assistant about the amount of water in the tank in liters.
HASensor reserveSensor("reserve");  // Reserve sensor
HASensor pomiarSensor("pomiar");  // Distance sensor - current reading in mm
HASensor waterEmptySensor("water_empty");  // "No water" sensor for HA
// Switches
HASwitch buzzerSwitch("buzzer");  // Buzzer control - allows turning the buzzer on and off via Home Assistant.
HASwitch alarmSwitch("alarm_switch");  // Displaying and resetting the alarm - allows displaying the alarm status and manually clearing it.
HASwitch serviceSwitch("service_mode");  // Service mode - switch allowing to turn the service mode on or off (ignoring some functions).
// Numeric settings
HANumber pumpTimeNumber("pump_time");  // Pump working time - allows setting the pump working time, with an accuracy of full seconds.
HANumber delayTimeNumber("delay_time");  // Delay time - allows setting the delay before turning on the pump, with an accuracy of full seconds.
HANumber tankDiameterHA("tank_diameter");  // Tank diameter - allows setting the tank diameter in Home Assistant.
HANumber distanceFullHA("distance_full");  // Full tank distance - setting the sensor distance from the water surface when the tank is full.
HANumber distanceEmptyHA("distance_empty");  // Empty tank distance - setting the sensor distance from the tank bottom when the tank is empty.
HANumber reserveThresholdHA("reserve_threshold");  // Reserve threshold setting

long duration;
int distance;
bool pumpState = false;
unsigned long previousMillis = 0;
const long interval = 2000; // Measurement interval in milliseconds

// New variables
int pompa_opoznienie = 5;
int pompa_praca = 60;
int zbiornik_pelny = 65;
int zbiornik_pusty = 510;
int zbiornik_rezerwa = 450;
int zbiornik_histereza = 10;
bool alarm = false;
bool brak_wody = false;
bool dzwiek = false;

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(SENSOR_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  setup_wifi();

  // OTA configuration
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    Serial.println("Start OTA: " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
}

void setup_wifi() {
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!haMqtt.isConnected()) {
    Serial.print("Attempting MQTT connection...");
    if (haMqtt.begin(mqtt_server, mqtt_port, mqtt_user, mqtt_password)) {
      Serial.println("connected");
      haMqtt.subscribe("hydrosense/control");
    } else {
      Serial.println("failed, try again in 5 seconds");
      delay(5000);
    }
  }
}

void measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;

  zbiornik_pelny = distance < 10;
  zbiornik_pusty = distance > 100;
  zbiornik_rezerwa = distance < 20 && distance > 10;

  if (zbiornik_pelny) { // Full tank
    if (!pumpState) {
      digitalWrite(PUMP_PIN, HIGH);
      pumpState = true;
    }
  } else {
    if (pumpState) {
      digitalWrite(PUMP_PIN, LOW);
      pumpState = false;
    }
  }

  if (zbiornik_pelny && distance < 5) { // Alarm threshold
    digitalWrite(BUZZER_PIN, HIGH);
    dzwiek = true;
  } else {
    digitalWrite(BUZZER_PIN, LOW);
    dzwiek = false;
  }
}

void loop() {
  if (!haMqtt.isConnected()) {
    reconnect();
  }
  haMqtt.loop();

  ArduinoOTA.handle();

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    measureDistance();
  }
}
