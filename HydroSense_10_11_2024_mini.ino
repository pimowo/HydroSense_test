#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include <ArduinoHA.h>
#include <ArduinoOTA.h>
#include <ESP8266WebServer.h>
#include <CRC32.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define BUILD_DATE TOSTRING(__DATE__)
#define STACK_PROTECTOR 512 // bytes

namespace HydroSense {

  constexpr uint8_t PIN_TRIG = D6;
  constexpr uint8_t PIN_ECHO = D7;
  constexpr uint8_t PIN_SENSOR = D5;
  constexpr uint8_t PIN_PUMP = D1;
  constexpr uint8_t PIN_BUZZER = D2;
  constexpr uint8_t PIN_BUTTON = D3;

  constexpr unsigned long SENSOR_CHECK_INTERVAL = 1000; // ms
  constexpr unsigned long MQTT_RETRY_INTERVAL = 5000;   // ms
  constexpr float TANK_DIAMETER_MIN = 50.0f;           // mm
  constexpr float TANK_DIAMETER_MAX = 250.0f;          // mm

  const char* WIFI_SSID = "pimowo";
  const char* WIFI_PASSWORD = "ckH59LRZQzCDQFiUgj";

  const char* MQTT_BROKER = "192.168.1.14";
  const uint16_t MQTT_PORT = 1883;
  const char* MQTT_USER = "hydrosense";
  const char* MQTT_PASSWORD = "hydrosense";
  const char* MQTT_CLIENT_ID = "HydroSense_ESP8266";

  class WaterLevelSensor {

    public:
      WaterLevelSensor() : lastMeasurement(0), distance(0.0f) {}
    
      float measureDistance() {
        if (millis() - lastMeasurement < SENSOR_CHECK_INTERVAL) {
          return distance;
        }
          
        float measurements[3];
        for (int i = 0; i < 3; ++i) {
          digitalWrite(PIN_TRIG, LOW);
          delayMicroseconds(2);
          digitalWrite(PIN_TRIG, HIGH);
          delayMicroseconds(10);
          digitalWrite(PIN_TRIG, LOW);
          
          measurements[i] = pulseIn(PIN_ECHO, HIGH) * 0.343f / 2.0f;
          delayMicroseconds(50);
        }
          
        distance = (measurements[0] + measurements[1] + measurements[2]) / 3.0f;
        lastMeasurement = millis();
        return distance;
      }
    
    private:
      unsigned long lastMeasurement;
      float distance;
  };

  class PumpController {
    
    public:
      enum class State {
        IDLE,
        DELAYED_START,
        RUNNING,
        ERROR
      };
    
      PumpController() : 
      state(State::IDLE),
      startTime(0),
      pumpDuration(30000), // 30 sekund
      delayDuration(5000)  // 5 sekund
      {}
    
      void update() {
        unsigned long currentTime = millis();
        
        switch (state) {
          case State::DELAYED_START:
            if (currentTime - startTime >= delayDuration) {
              startPump();
            }
            break;
                
          case State::RUNNING:
            if (currentTime - startTime >= pumpDuration) {
              stopPump();
            }
            break;
                
          default:
            break;
        }
      }
    
      void requestStart() {
        if (state == State::IDLE) {
          state = State::DELAYED_START;
          startTime = millis();
        }
      }
    
      void stop() {
        stopPump();
      }
    
    private:
      void startPump() {
        digitalWrite(PIN_PUMP, HIGH);
        state = State::RUNNING;
        startTime = millis();
      }
    
      void stopPump() {
        digitalWrite(PIN_PUMP, LOW);
        state = State::IDLE;
      }
    
      State state;
      unsigned long startTime;
      unsigned long pumpDuration;
      unsigned long delayDuration;
  };

  class Settings {
    
    private:
      static const uint32_t SETTINGS_MAGIC = 0xABCD1234;
      uint32_t m_magic;

      char m_wifiSSID[32];
      char m_wifiPassword[64];

      char m_mqttServer[40];
      uint16_t m_mqttPort;
      char m_mqttUser[40];
      char m_mqttPassword[40];

      float m_tankDiameter;
      float m_tankWidth;
      float m_tankHeight;
      float m_fullDistance;
      float m_emptyDistance;
      float m_reserveLevel;
      float m_reserveHysteresis;

      uint32_t m_pumpDelay;
      uint32_t m_pumpWork;

      bool m_soundEnabled = true;
      bool m_reserveState;

    public:    
      Settings() {
        loadDefaults();
      }

      void loadDefaults() {
        m_magic = SETTINGS_MAGIC;
        
        memset(m_wifiSSID, 0, sizeof(m_wifiSSID));
        memset(m_wifiPassword, 0, sizeof(m_wifiPassword));

        memset(m_mqttServer, 0, sizeof(m_mqttServer));
        m_mqttPort = 1883;
        memset(m_mqttUser, 0, sizeof(m_mqttUser));
        memset(m_mqttPassword, 0, sizeof(m_mqttPassword));

        m_tankDiameter = 200.0f;
        m_tankWidth = 0.0f;
        m_tankHeight = 300.0f;
        m_fullDistance = 50.0f;
        m_emptyDistance = 300.0f;
        m_reserveLevel = 250.0f;
        m_reserveHysteresis = 20.0f;

        m_pumpDelay = 300;
        m_pumpWork = 60;

        m_soundEnabled = true;
        m_reserveState = false;

        save();
      }

      const char* getWiFiSSID() const { return m_wifiSSID; }
      
      void setWiFiCredentials(const char* ssid, const char* password) {
        strncpy(m_wifiSSID, ssid, sizeof(m_wifiSSID) - 1);
        strncpy(m_wifiPassword, password, sizeof(m_wifiPassword) - 1);
        m_wifiSSID[sizeof(m_wifiSSID) - 1] = '\0';
        m_wifiPassword[sizeof(m_wifiPassword) - 1] = '\0';
        save();
      }

      const char* getMqttServer() const { return m_mqttServer; }
      uint16_t getMqttPort() const { return m_mqttPort; }
      const char* getMqttUser() const { return m_mqttUser; }
      const char* getMqttPassword() const { return m_mqttPassword; }
    
      void setMqttConfig(const char* server, uint16_t port, const char* user, const char* password) {
        strncpy(m_mqttServer, server, sizeof(m_mqttServer) - 1);
        m_mqttPort = port;
        strncpy(m_mqttUser, user, sizeof(m_mqttUser) - 1);
        strncpy(m_mqttPassword, password, sizeof(m_mqttPassword) - 1);
        m_mqttServer[sizeof(m_mqttServer) - 1] = '\0';
        m_mqttUser[sizeof(m_mqttUser) - 1] = '\0';
        m_mqttPassword[sizeof(m_mqttPassword) - 1] = '\0';
        save();
      }

      void setReserveLevel(float level) {
        m_reserveLevel = level;
        save();
      }

      float getTankDiameter() const { return m_tankDiameter; }
      float getTankWidth() const { return m_tankWidth; }
      float getTankHeight() const { return m_tankHeight; }
      float getFullDistance() const { return m_fullDistance; }
      float getEmptyDistance() const { return m_emptyDistance; }
      float getReserveLevel() const { return m_reserveLevel; }
      float getReserveHysteresis() const { return m_reserveHysteresis; }

      void setTankDimensions(float diameter, float width, float height, float fullDist, float emptyDist, float reserveLevel) {
        m_tankDiameter = diameter;
        m_tankWidth = width;
        m_tankHeight = height;
        m_fullDistance = fullDist;
        m_emptyDistance = emptyDist;
        m_reserveLevel = reserveLevel;
        save();
      }

      uint32_t getPumpDelay() const { return m_pumpDelay; }
      uint32_t getPumpWork() const { return m_pumpWork; }
    
      void setPumpTiming(uint32_t delay, uint32_t work) {
        m_pumpDelay = delay;
        m_pumpWork = work;
        save();
      }

      bool isSoundEnabled() const { return m_soundEnabled; }
      
      void setSoundEnabled(bool enabled) {
        m_soundEnabled = enabled;
        save();
      }

      bool isInReserve() const { return m_reserveState; }
      bool checkReserveState(float currentDistance) {
        if (!m_reserveState && currentDistance >= m_reserveLevel) {
          m_reserveState = true;
        } else if (m_reserveState && currentDistance <= (m_reserveLevel - m_reserveHysteresis)) {
          m_reserveState = false;
        }
        return m_reserveState;
      }

      void save() {
        EEPROM.begin(512);
        uint16_t addr = 0;

        Serial.println("Zapisywanie ustawień do EEPROM");

        EEPROM.put(addr, m_magic); addr += sizeof(m_magic);
        EEPROM.put(addr, m_wifiSSID); addr += sizeof(m_wifiSSID);
        EEPROM.put(addr, m_wifiPassword); addr += sizeof(m_wifiPassword);
        EEPROM.put(addr, m_mqttServer); addr += sizeof(m_mqttServer);
        EEPROM.put(addr, m_mqttPort); addr += sizeof(m_mqttPort);
        EEPROM.put(addr, m_mqttUser); addr += sizeof(m_mqttUser);
        EEPROM.put(addr, m_mqttPassword); addr += sizeof(m_mqttPassword);
        EEPROM.put(addr, m_tankDiameter); addr += sizeof(m_tankDiameter);
        EEPROM.put(addr, m_tankWidth); addr += sizeof(m_tankWidth);
        EEPROM.put(addr, m_tankHeight); addr += sizeof(m_tankHeight);
        EEPROM.put(addr, m_fullDistance); addr += sizeof(m_fullDistance);
        EEPROM.put(addr, m_emptyDistance); addr += sizeof(m_emptyDistance);
        EEPROM.put(addr, m_reserveLevel); addr += sizeof(m_reserveLevel);
        EEPROM.put(addr, m_reserveHysteresis); addr += sizeof(m_reserveHysteresis);
        EEPROM.put(addr, m_pumpDelay); addr += sizeof(m_pumpDelay);
        EEPROM.put(addr, m_pumpWork); addr += sizeof(m_pumpWork);
        EEPROM.put(addr, m_soundEnabled); addr += sizeof(m_soundEnabled);
        EEPROM.put(addr, m_reserveState); addr += sizeof(m_reserveState);

        EEPROM.commit();
        EEPROM.end();

        Serial.println("Ustawienia zapisane");
      }

      void load() {
        EEPROM.begin(512);
        uint16_t addr = 0;

        Serial.println("Ładowanie ustawień z EEPROM");

        EEPROM.get(addr, m_magic); addr += sizeof(m_magic);

        if (m_magic != SETTINGS_MAGIC) {
          Serial.println("Wykryto niezainicjalizowany EEPROM - ładowanie wartości domyślnych");
          EEPROM.end();
          loadDefaults();
          return;
        }

        EEPROM.get(addr, m_wifiSSID); addr += sizeof(m_wifiSSID);
        EEPROM.get(addr, m_wifiPassword); addr += sizeof(m_wifiPassword);
        EEPROM.get(addr, m_mqttServer); addr += sizeof(m_mqttServer);
        EEPROM.get(addr, m_mqttPort); addr += sizeof(m_mqttPort);
        EEPROM.get(addr, m_mqttUser); addr += sizeof(m_mqttUser);
        EEPROM.get(addr, m_mqttPassword); addr += sizeof(m_mqttPassword);
        EEPROM.get(addr, m_tankDiameter); addr += sizeof(m_tankDiameter);
        EEPROM.get(addr, m_tankWidth); addr += sizeof(m_tankWidth);
        EEPROM.get(addr, m_tankHeight); addr += sizeof(m_tankHeight);
        EEPROM.get(addr, m_fullDistance); addr += sizeof(m_fullDistance);
        EEPROM.get(addr, m_emptyDistance); addr += sizeof(m_emptyDistance);
        EEPROM.get(addr, m_reserveLevel); addr += sizeof(m_reserveLevel);
        EEPROM.get(addr, m_reserveHysteresis); addr += sizeof(m_reserveHysteresis);
        EEPROM.get(addr, m_pumpDelay); addr += sizeof(m_pumpDelay);
        EEPROM.get(addr, m_pumpWork); addr += sizeof(m_pumpWork);
        EEPROM.get(addr, m_soundEnabled); addr += sizeof(m_soundEnabled);
        EEPROM.get(addr, m_reserveState); addr += sizeof(m_reserveState);

        EEPROM.end();

        Serial.println("Ustawienia załadowane");
      }

      void printSettings() {
        Serial.printf("m_wifiSSID: %s\n", m_wifiSSID);
        Serial.printf("m_wifiPassword: %s\n", m_wifiPassword);
        Serial.printf("m_mqttServer: %s\n", m_mqttServer);
        Serial.printf("m_mqttPort: %d\n", m_mqttPort);
        Serial.printf("m_mqttUser: %s\n", m_mqttUser);
        Serial.printf("m_mqttPassword: %s\n", m_mqttPassword);
        Serial.printf("m_tankDiameter: %.1f mm\n", m_tankDiameter);
        Serial.printf("m_tankWidth: %.1f mm\n", m_tankWidth);
        Serial.printf("m_tankHeight: %.1f mm\n", m_tankHeight);
        Serial.printf("m_fullDistance: %.1f mm\n", m_fullDistance);
        Serial.printf("m_emptyDistance: %.1f mm\n", m_emptyDistance);
        Serial.printf("m_reserveLevel: %.1f mm\n", m_reserveLevel);
        Serial.printf("m_reserveHysteresis: %.1f mm\n", m_reserveHysteresis);
        Serial.printf("m_pumpDelay: %d s\n", m_pumpDelay);
        Serial.printf("m_pumpWork: %d s\n", m_pumpWork);
        Serial.printf("m_soundEnabled: %d\n", m_soundEnabled);
        Serial.printf("m_reserveState: %d\n", m_reserveState);
      }
  };

  class HydroSenseApp {
    private:
      char mqtt_broker[40] = "";
      char mqtt_user[40] = "";
      char mqtt_password[40] = "";
      uint16_t mqtt_port = 1883;
    
      WiFiManager wm;
    
      static constexpr unsigned long WIFI_CHECK_INTERVAL = 30000;  // ms
      static constexpr unsigned long UPDATE_INTERVAL = 1000;       // ms
      
      WiFiClient wifiClient;
      String deviceId;
      HADevice device;
      HAMqtt mqtt;
      ESP8266WebServer webServer;
      unsigned long lastWiFiCheck;
      unsigned long lastMqttRetry;
      unsigned long lastUpdateTime;

      HASensor haWaterLevelSensor;
      HASensor haWaterLevelPercentSensor;
      HABinarySensor reserveSensor;

      Settings settings;
      WaterLevelSensor levelSensor;
      PumpController pumpController;

      String createSensorConfig(const char* id, const char* name, const char* unit) {
        String config = "{";
        config += "\"name\":\"" + String(name) + "\",";
        config += "\"device_class\":\"" + String(id) + "\",";
        config += "\"state_topic\":\"hydrosense/" + String(id) + "/state\",";
        config += "\"unit_of_measurement\":\"" + String(unit) + "\",";
        config += "\"unique_id\":\"hydrosense_" + deviceId + "_" + String(id) + "\",";
        config += "\"device\":{";
        config += "\"identifiers\":[\"hydrosense_" + deviceId + "\"],";
        config += "\"name\":\"HydroSense\",";
        config += "\"model\":\"HS ESP8266\",";
        config += "\"manufacturer\":\"PMW\"";
        config += "}}";
        return config;
      }

      float calculateWaterPercentage(float waterLevel) {
        float maxLevel = settings.getEmptyDistance() - settings.getFullDistance();
        if (maxLevel <= 0) return 0.0f;
        
        float currentLevel = settings.getEmptyDistance() - waterLevel;
        float percentage = (currentLevel / maxLevel) * 100.0f;
        return constrain(percentage, 0.0f, 100.0f);
      }

    public:
      void handleButton() {
        static unsigned long pressStartTime = 0;
        static bool wasPressed = false;
        static bool serviceMode = false;
        static bool longPressActionExecuted = false;
        
        bool isPressed = (digitalRead(PIN_BUTTON) == LOW);
        
        if (isPressed && !wasPressed) {
          pressStartTime = millis();
          wasPressed = true;
          longPressActionExecuted = false;
        } else if (isPressed && wasPressed) {
          if (!longPressActionExecuted && (millis() - pressStartTime >= 1000)) {
            Serial.println("Wykonuję kasowanie alarmu...");
            digitalWrite(PIN_BUZZER, LOW);
            longPressActionExecuted = true;
          }
        } else if (!isPressed && wasPressed) {
          wasPressed = false;
        }
      }
      
      void update() {
        levelSensor.measureDistance();
        float waterLevel = levelSensor.measureDistance();
        float waterPercentage = calculateWaterPercentage(waterLevel);

        haWaterLevelSensor.setValue(waterLevel);
        haWaterLevelPercentSensor.setValue(waterPercentage);
        reserveSensor.setState(settings.checkReserveState(waterLevel));

        pumpController.update();
      }
  };     

  HydroSenseApp hydroSenseApp;

  #include <ESP8266WiFi.h>
#include <EEPROM.h>
#include <ArduinoHA.h>
#include <ArduinoOTA.h>
#include <ESP8266WebServer.h>
#include <CRC32.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define BUILD_DATE TOSTRING(__DATE__)
#define STACK_PROTECTOR 512 // bytes

namespace HydroSense {

  constexpr uint8_t PIN_TRIG = D6;
  constexpr uint8_t PIN_ECHO = D7;
  constexpr uint8_t PIN_SENSOR = D5;
  constexpr uint8_t PIN_PUMP = D1;
  constexpr uint8_t PIN_BUZZER = D2;
  constexpr uint8_t PIN_BUTTON = D3;

  constexpr unsigned long SENSOR_CHECK_INTERVAL = 1000; // ms
  constexpr unsigned long MQTT_RETRY_INTERVAL = 5000;   // ms
  constexpr float TANK_DIAMETER_MIN = 50.0f;           // mm
  constexpr float TANK_DIAMETER_MAX = 250.0f;          // mm

  const char* WIFI_SSID = "pimowo";
  const char* WIFI_PASSWORD = "ckH59LRZQzCDQFiUgj";

  const char* MQTT_BROKER = "192.168.1.14";
  const uint16_t MQTT_PORT = 1883;
  const char* MQTT_USER = "hydrosense";
  const char* MQTT_PASSWORD = "hydrosense";
  const char* MQTT_CLIENT_ID = "HydroSense_ESP8266";

  class WaterLevelSensor {

    public:
      WaterLevelSensor() : lastMeasurement(0), distance(0.0f) {}
    
      float measureDistance() {
        if (millis() - lastMeasurement < SENSOR_CHECK_INTERVAL) {
          return distance;
        }
          
        float measurements[3];
        for (int i = 0; i < 3; ++i) {
          digitalWrite(PIN_TRIG, LOW);
          delayMicroseconds(2);
          digitalWrite(PIN_TRIG, HIGH);
          delayMicroseconds(10);
          digitalWrite(PIN_TRIG, LOW);
          
          measurements[i] = pulseIn(PIN_ECHO, HIGH) * 0.343f / 2.0f;
          delayMicroseconds(50);
        }
          
        distance = (measurements[0] + measurements[1] + measurements[2]) / 3.0f;
        lastMeasurement = millis();
        return distance;
      }
    
    private:
      unsigned long lastMeasurement;
      float distance;
  };

  class PumpController {
    
    public:
      enum class State {
        IDLE,
        DELAYED_START,
        RUNNING,
        ERROR
      };
    
      PumpController() : 
      state(State::IDLE),
      startTime(0),
      pumpDuration(30000), // 30 sekund
      delayDuration(5000)  // 5 sekund
      {}
    
      void update() {
        unsigned long currentTime = millis();
        
        switch (state) {
          case State::DELAYED_START:
            if (currentTime - startTime >= delayDuration) {
              startPump();
            }
            break;
                
          case State::RUNNING:
            if (currentTime - startTime >= pumpDuration) {
              stopPump();
            }
            break;
                
          default:
            break;
        }
      }
    
      void requestStart() {
        if (state == State::IDLE) {
          state = State::DELAYED_START;
          startTime = millis();
        }
      }
    
      void stop() {
        stopPump();
      }
    
    private:
      void startPump() {
        digitalWrite(PIN_PUMP, HIGH);
        state = State::RUNNING;
        startTime = millis();
      }
    
      void stopPump() {
        digitalWrite(PIN_PUMP, LOW);
        state = State::IDLE;
      }
    
      State state;
      unsigned long startTime;
      unsigned long pumpDuration;
      unsigned long delayDuration;
  };

  class Settings {
    
    private:
      static const uint32_t SETTINGS_MAGIC = 0xABCD1234;
      uint32_t m_magic;

      char m_wifiSSID[32];
      char m_wifiPassword[64];

      char m_mqttServer[40];
      uint16_t m_mqttPort;
      char m_mqttUser[40];
      char m_mqttPassword[40];

      float m_tankDiameter;
      float m_tankWidth;
      float m_tankHeight;
      float m_fullDistance;
      float m_emptyDistance;
      float m_reserveLevel;
      float m_reserveHysteresis;

      uint32_t m_pumpDelay;
      uint32_t m_pumpWork;

      bool m_soundEnabled = true;
      bool m_reserveState;

    public:    
      Settings() {
        loadDefaults();
      }

      void loadDefaults() {
        m_magic = SETTINGS_MAGIC;
        
        memset(m_wifiSSID, 0, sizeof(m_wifiSSID));
        memset(m_wifiPassword, 0, sizeof(m_wifiPassword));

        memset(m_mqttServer, 0, sizeof(m_mqttServer));
        m_mqttPort = 1883;
        memset(m_mqttUser, 0, sizeof(m_mqttUser));
        memset(m_mqttPassword, 0, sizeof(m_mqttPassword));

        m_tankDiameter = 200.0f;
        m_tankWidth = 0.0f;
        m_tankHeight = 300.0f;
        m_fullDistance = 50.0f;
        m_emptyDistance = 300.0f;
        m_reserveLevel = 250.0f;
        m_reserveHysteresis = 20.0f;

        m_pumpDelay = 300;
        m_pumpWork = 60;

        m_soundEnabled = true;
        m_reserveState = false;

        save();
      }

      const char* getWiFiSSID() const { return m_wifiSSID; }
      
      void setWiFiCredentials(const char* ssid, const char* password) {
        strncpy(m_wifiSSID, ssid, sizeof(m_wifiSSID) - 1);
        strncpy(m_wifiPassword, password, sizeof(m_wifiPassword) - 1);
        m_wifiSSID[sizeof(m_wifiSSID) - 1] = '\0';
        m_wifiPassword[sizeof(m_wifiPassword) - 1] = '\0';
        save();
      }

      const char* getMqttServer() const { return m_mqttServer; }
      uint16_t getMqttPort() const { return m_mqttPort; }
      const char* getMqttUser() const { return m_mqttUser; }
      const char* getMqttPassword() const { return m_mqttPassword; }
    
      void setMqttConfig(const char* server, uint16_t port, const char* user, const char* password) {
        strncpy(m_mqttServer, server, sizeof(m_mqttServer) - 1);
        m_mqttPort = port;
        strncpy(m_mqttUser, user, sizeof(m_mqttUser) - 1);
        strncpy(m_mqttPassword, password, sizeof(m_mqttPassword) - 1);
        m_mqttServer[sizeof(m_mqttServer) - 1] = '\0';
        m_mqttUser[sizeof(m_mqttUser) - 1] = '\0';
        m_mqttPassword[sizeof(m_mqttPassword) - 1] = '\0';
        save();
      }

      void setReserveLevel(float level) {
        m_reserveLevel = level;
        save();
      }

      float getTankDiameter() const { return m_tankDiameter; }
      float getTankWidth() const { return m_tankWidth; }
      float getTankHeight() const { return m_tankHeight; }
      float getFullDistance() const { return m_fullDistance; }
      float getEmptyDistance() const { return m_emptyDistance; }
      float getReserveLevel() const { return m_reserveLevel; }
      float getReserveHysteresis() const { return m_reserveHysteresis; }

      void setTankDimensions(float diameter, float width, float height, float fullDist, float emptyDist, float reserveLevel) {
        m_tankDiameter = diameter;
        m_tankWidth = width;
        m_tankHeight = height;
        m_fullDistance = fullDist;
        m_emptyDistance = emptyDist;
        m_reserveLevel = reserveLevel;
        save();
      }

      uint32_t getPumpDelay() const { return m_pumpDelay; }
      uint32_t getPumpWork() const { return m_pumpWork; }
    
      void setPumpTiming(uint32_t delay, uint32_t work) {
        m_pumpDelay = delay;
        m_pumpWork = work;
        save();
      }

      bool isSoundEnabled() const { return m_soundEnabled; }
      
      void setSoundEnabled(bool enabled) {
        m_soundEnabled = enabled;
        save();
      }

      bool isInReserve() const { return m_reserveState; }
      bool checkReserveState(float currentDistance) {
        if (!m_reserveState && currentDistance >= m_reserveLevel) {
          m_reserveState = true;
        } else if (m_reserveState && currentDistance <= (m_reserveLevel - m_reserveHysteresis)) {
          m_reserveState = false;
        }
        return m_reserveState;
      }

      void save() {
        EEPROM.begin(512);
        uint16_t addr = 0;

        Serial.println("Zapisywanie ustawień do EEPROM");

        EEPROM.put(addr, m_magic); addr += sizeof(m_magic);
        EEPROM.put(addr, m_wifiSSID); addr += sizeof(m_wifiSSID);
        EEPROM.put(addr, m_wifiPassword); addr += sizeof(m_wifiPassword);
        EEPROM.put(addr, m_mqttServer); addr += sizeof(m_mqttServer);
        EEPROM.put(addr, m_mqttPort); addr += sizeof(m_mqttPort);
        EEPROM.put(addr, m_mqttUser); addr += sizeof(m_mqttUser);
        EEPROM.put(addr, m_mqttPassword); addr += sizeof(m_mqttPassword);
        EEPROM.put(addr, m_tankDiameter); addr += sizeof(m_tankDiameter);
        EEPROM.put(addr, m_tankWidth); addr += sizeof(m_tankWidth);
        EEPROM.put(addr, m_tankHeight); addr += sizeof(m_tankHeight);
        EEPROM.put(addr, m_fullDistance); addr += sizeof(m_fullDistance);
        EEPROM.put(addr, m_emptyDistance); addr += sizeof(m_emptyDistance);
        EEPROM.put(addr, m_reserveLevel); addr += sizeof(m_reserveLevel);
        EEPROM.put(addr, m_reserveHysteresis); addr += sizeof(m_reserveHysteresis);
        EEPROM.put(addr, m_pumpDelay); addr += sizeof(m_pumpDelay);
        EEPROM.put(addr, m_pumpWork); addr += sizeof(m_pumpWork);
        EEPROM.put(addr, m_soundEnabled); addr += sizeof(m_soundEnabled);
        EEPROM.put(addr, m_reserveState); addr += sizeof(m_reserveState);

        EEPROM.commit();
        EEPROM.end();

        Serial.println("Ustawienia zapisane");
      }

      void load() {
        EEPROM.begin(512);
        uint16_t addr = 0;

        Serial.println("Ładowanie ustawień z EEPROM");

        EEPROM.get(addr, m_magic); addr += sizeof(m_magic);

        if (m_magic != SETTINGS_MAGIC) {
          Serial.println("Wykryto niezainicjalizowany EEPROM - ładowanie wartości domyślnych");
          EEPROM.end();
          loadDefaults();
          return;
        }

        EEPROM.get(addr, m_wifiSSID); addr += sizeof(m_wifiSSID);
        EEPROM.get(addr, m_wifiPassword); addr += sizeof(m_wifiPassword);
        EEPROM.get(addr, m_mqttServer); addr += sizeof(m_mqttServer);
        EEPROM.get(addr, m_mqttPort); addr += sizeof(m_mqttPort);
        EEPROM.get(addr, m_mqttUser); addr += sizeof(m_mqttUser);
        EEPROM.get(addr, m_mqttPassword); addr += sizeof(m_mqttPassword);
        EEPROM.get(addr, m_tankDiameter); addr += sizeof(m_tankDiameter);
        EEPROM.get(addr, m_tankWidth); addr += sizeof(m_tankWidth);
        EEPROM.get(addr, m_tankHeight); addr += sizeof(m_tankHeight);
        EEPROM.get(addr, m_fullDistance); addr += sizeof(m_fullDistance);
        EEPROM.get(addr, m_emptyDistance); addr += sizeof(m_emptyDistance);
        EEPROM.get(addr, m_reserveLevel); addr += sizeof(m_reserveLevel);
        EEPROM.get(addr, m_reserveHysteresis); addr += sizeof(m_reserveHysteresis);
        EEPROM.get(addr, m_pumpDelay); addr += sizeof(m_pumpDelay);
        EEPROM.get(addr, m_pumpWork); addr += sizeof(m_pumpWork);
        EEPROM.get(addr, m_soundEnabled); addr += sizeof(m_soundEnabled);
        EEPROM.get(addr, m_reserveState); addr += sizeof(m_reserveState);

        EEPROM.end();

        Serial.println("Ustawienia załadowane");
      }

      void printSettings() {
        Serial.printf("m_wifiSSID: %s\n", m_wifiSSID);
        Serial.printf("m_wifiPassword: %s\n", m_wifiPassword);
        Serial.printf("m_mqttServer: %s\n", m_mqttServer);
        Serial.printf("m_mqttPort: %d\n", m_mqttPort);
        Serial.printf("m_mqttUser: %s\n", m_mqttUser);
        Serial.printf("m_mqttPassword: %s\n", m_mqttPassword);
        Serial.printf("m_tankDiameter: %.1f mm\n", m_tankDiameter);
        Serial.printf("m_tankWidth: %.1f mm\n", m_tankWidth);
        Serial.printf("m_tankHeight: %.1f mm\n", m_tankHeight);
        Serial.printf("m_fullDistance: %.1f mm\n", m_fullDistance);
        Serial.printf("m_emptyDistance: %.1f mm\n", m_emptyDistance);
        Serial.printf("m_reserveLevel: %.1f mm\n", m_reserveLevel);
        Serial.printf("m_reserveHysteresis: %.1f mm\n", m_reserveHysteresis);
        Serial.printf("m_pumpDelay: %d s\n", m_pumpDelay);
        Serial.printf("m_pumpWork: %d s\n", m_pumpWork);
        Serial.printf("m_soundEnabled: %d\n", m_soundEnabled);
        Serial.printf("m_reserveState: %d\n", m_reserveState);
      }
  };

  class HydroSenseApp {
    private:
      char mqtt_broker[40] = "";
      char mqtt_user[40] = "";
      char mqtt_password[40] = "";
      uint16_t mqtt_port = 1883;
    
      WiFiManager wm;
    
      static constexpr unsigned long WIFI_CHECK_INTERVAL = 30000;  // ms
      static constexpr unsigned long UPDATE_INTERVAL = 1000;       // ms
      
      WiFiClient wifiClient;
      String deviceId;
      HADevice device;
      HAMqtt mqtt;
      ESP8266WebServer webServer;
      unsigned long lastWiFiCheck;
      unsigned long lastMqttRetry;
      unsigned long lastUpdateTime;

      HASensor haWaterLevelSensor;
      HASensor haWaterLevelPercentSensor;
      HABinarySensor reserveSensor;

      Settings settings;
      WaterLevelSensor levelSensor;
      PumpController pumpController;

      String createSensorConfig(const char* id, const char* name, const char* unit) {
        String config = "{";
        config += "\"name\":\"" + String(name) + "\",";
        config += "\"device_class\":\"" + String(id) + "\",";
        config += "\"state_topic\":\"hydrosense/" + String(id) + "/state\",";
        config += "\"unit_of_measurement\":\"" + String(unit) + "\",";
        config += "\"unique_id\":\"hydrosense_" + deviceId + "_" + String(id) + "\",";
        config += "\"device\":{";
        config += "\"identifiers\":[\"hydrosense_" + deviceId + "\"],";
        config += "\"name\":\"HydroSense\",";
        config += "\"model\":\"HS ESP8266\",";
        config += "\"manufacturer\":\"PMW\"";
        config += "}}";
        return config;
      }

      float calculateWaterPercentage(float waterLevel) {
        float maxLevel = settings.getEmptyDistance() - settings.getFullDistance();
        if (maxLevel <= 0) return 0.0f;
        
        float currentLevel = settings.getEmptyDistance() - waterLevel;
        float percentage = (currentLevel / maxLevel) * 100.0f;
        return constrain(percentage, 0.0f, 100.0f);
      }

    public:
      void handleButton() {
        static unsigned long pressStartTime = 0;
        static bool wasPressed = false;
        static bool serviceMode = false;
        static bool longPressActionExecuted = false;
        
        bool isPressed = (digitalRead(PIN_BUTTON) == LOW);
        
        if (isPressed && !wasPressed) {
          pressStartTime = millis();
          wasPressed = true;
          longPressActionExecuted = false;
        } else if (isPressed && wasPressed) {
          if (!longPressActionExecuted && (millis() - pressStartTime >= 1000)) {
            Serial.println("Wykonuję kasowanie alarmu...");
            digitalWrite(PIN_BUZZER, LOW);
            longPressActionExecuted = true;
          }
        } else if (!isPressed && wasPressed) {
          wasPressed = false;
        }
      }
      
      void update() {
        levelSensor.measureDistance();
        float waterLevel = levelSensor.measureDistance();
        float waterPercentage = calculateWaterPercentage(waterLevel);

        haWaterLevelSensor.setValue(waterLevel);
        haWaterLevelPercentSensor.setValue(waterPercentage);
        reserveSensor.setState(settings.checkReserveState(waterLevel));

        pumpController.update();
      }
  };     

HydroSenseApp hydroSenseApp;

void setup() {
  Serial.begin(115200);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_SENSOR, INPUT);
  pinMode(PIN_PUMP, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");

  mqtt.begin(MQTT_BROKER, MQTT_PORT, wifiClient);
  while (!mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
    delay(500);
    Serial.print("*");
  }
  Serial.println("MQTT connected");

  settings.load();
}

void loop() {
  hydroSenseApp.update();

  hydroSenseApp.handleButton();

  if (millis() - hydroSenseApp.lastUpdateTime >= HydroSenseApp::UPDATE_INTERVAL) {
    hydroSenseApp.lastUpdateTime = millis();
    hydroSenseApp.update();
  }

  if (millis() - hydroSenseApp.lastWiFiCheck >= HydroSenseApp::WIFI_CHECK_INTERVAL) {
    hydroSenseApp.lastWiFiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.reconnect();
    }
    if (!mqtt.connected()) {
      mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD);
    }
  }
}
