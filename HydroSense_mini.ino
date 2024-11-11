#include <ESP8266WiFi.h>  // Biblioteka do obsługi WiFi na ESP8266
#include <ArduinoOTA.h>  // Biblioteka do obsługi Over-the-Air (OTA) aktualizacji
#include <ArduinoHA.h>  // Biblioteka do integracji z Home Assistant przez MQTT

// ========== SYSTEM CONFIGURATION ==========
// Konfiguracja systemu
const char* WIFI_SSID = "pimowo";
const char* WIFI_PASSWORD = "ckH59LRZQzCDQFiUgj";
const char* MQTT_SERVER = "192.168.1.14";
const int MQTT_PORT = 1883;
const char* MQTT_USER = "hydrosense";
const char* MQTT_PASSWORD = "hydrosense";

// ========== SR04 MEASUREMENT CONFIGURATION ==========
const int MEASUREMENTS = 10;          // Liczba pomiarów
const int OUTLIERS = 2;              // Liczba odrzucanych skrajnych wartości
const unsigned long MEASUREMENT_DELAY = 1000;  // Opóźnienie między pomiarami (1s)

// ========== PIN DEFINITIONS ==========
// Definicje pinów
#define PIN_ULTRASONIC_TRIG D6 // Pin trigger czujnika ultradźwiękowego
#define PIN_ULTRASONIC_ECHO D7 // Pin echo czujnika ultradźwiękowego
#define PIN_WATER_LEVEL D5 // Czujnik poziomu wody
#define PIN_PUMP D1 // Sterowanie pompą
#define PIN_BUZZER D2 // Sygnalizator dźwiękowy
#define PIN_RESET_BUTTON D3 // Przycisk resetu alarmu

// ========== COMMUNICATION OBJECTS ==========
// Obiekty komunikacyjne
WiFiClient espClient;
HADevice device("HydroSense");
HAMqtt haMqtt(espClient, device);

// ========== HOME ASSISTANT SENSORS ==========
// Czujniki Home Assistant
HASensor sensorWaterPresence("water"); // Sensor wody
HASensor sensorPumpStatus("pump"); // Pompa
HASensor sensorWaterLevel("water_level_percent"); // Poziom wody w %
HASensor sensorWaterVolume("water_volume_liters"); // Objętość wody
HASensor sensorReserveStatus("reserve"); // Status rezerwy
HASensor sensorDistance("distance"); // Pomiar odległości
HASensor sensorWaterEmpty("water_empty"); // Status braku wody

// ========== HOME ASSISTANT SWITCHES ==========
// Przełączniki Home Assistant
HASwitch switchBuzzer("buzzer"); // Dzwięk
HASwitch switchAlarm("alarm_switch"); // Alarm
HASwitch switchServiceMode("service_mode");  // Serwis

// ========== HOME ASSISTANT CONFIGURATION PARAMETERS ==========
// Parametry konfiguracyjne Home Assistant
HANumber configPumpTime("pump_time");
HANumber configDelayTime("delay_time");
HANumber configTankDiameter("tank_diameter");
HANumber configDistanceFull("distance_full");
HANumber configDistanceEmpty("distance_empty");
HANumber configReserveThreshold("reserve_threshold");

// ========== SYSTEM VARIABLES ==========
// Zmienne systemowe
struct SystemStatus {
    long ultrasonicDuration;
    int currentDistance;
    bool isPumpActive;
    unsigned long lastMeasurementTime;
    unsigned long lastPumpActivationTime;
    const unsigned long MEASUREMENT_INTERVAL = 2000;  // ms
    unsigned long pumpStartDelay = 0;  // Czas rozpoczęcia odliczania opóźnienia
    bool isPumpDelayActive = false;  // Flaga wskazująca czy trwa odliczanie opóźnienia
    bool isWaterLevelLow = false;  // Stan czujnika wody (niski = true)
    bool lastPumpState = false;  // Poprzedni stan pompy do wykrywania zmian
} status;

// ========== TANK CONFIGURATION ==========
// Konfiguracja zbiornika
struct TankParameters {
  int pumpDelay = 5;
  int pumpWorkTime = 60;
  int distanceWhenFull = 65;
  int distanceWhenEmpty = 510;
  int reserveLevel = 450;
  int hysteresis = 10;
  bool alarmActive = false;
  bool waterEmpty = false;
  bool buzzerActive = false;
} tankConfig;

// ========== WELCOME MELODY ==========
// Melodia powitalna
struct WelcomeMelody {
  static const int NOTE_C4 = 262;
  static const int NOTE_D4 = 294;
  static const int NOTE_E4 = 330;
  static const int NOTE_F4 = 349;
  static const int NOTE_G4 = 392;
  
  int notes[5] = {NOTE_C4, NOTE_E4, NOTE_G4, NOTE_E4, NOTE_C4};
  int durations[5] = {200, 200, 200, 200, 400};
  int count = 5;
} melody;

// ========== SYSTEM FUNCTIONS ==========
// Funkcje systemowe
void initializeSystem() {
    Serial.begin(115200);
    configurePins();
    
    // Upewnij się, że pompa jest wyłączona na starcie i zaktualizuj stan w HA
    stopPump();
    status.lastPumpState = false;
    sensorPumpStatus.setValue("OFF");  // Wysyłamy początkowy stan do HA
    
    initializeWiFi();
    setupOTA();
    playWelcomeMelody();
}

// Funkcja OTA
void setupOTA() {
  ArduinoOTA.onStart([]() {
    Serial.println("Start OTA");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nKoniec OTA");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Postęp: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Błąd[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Błąd autoryzacji");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Błąd rozpoczęcia");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Błąd połączenia");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Błąd odbioru");
    else if (error == OTA_END_ERROR) Serial.println("Błąd zakończenia");
  });
  ArduinoOTA.begin();
}

// Funkcja konfigurująca piny
void configurePins() {
  pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);  // Ustawia pin trigger czujnika ultradźwiękowego jako wyjście
  pinMode(PIN_ULTRASONIC_ECHO, INPUT);   // Ustawia pin echo czujnika ultradźwiękowego jako wejście
  pinMode(PIN_PUMP, OUTPUT);  // Ustawia pin pompy jako wyjście
  pinMode(PIN_BUZZER, OUTPUT);  // Ustawia pin sygnalizatora dźwiękowego jako wyjście
  pinMode(PIN_RESET_BUTTON, INPUT_PULLUP);  // Ustawia pin przycisku resetu z wbudowanym rezystorem pull-up
  pinMode(PIN_WATER_LEVEL, INPUT_PULLUP);  // Ustawia pin czujnika poziomu wody z wbudowanym rezystorem pull-up
  
  digitalWrite(PIN_PUMP, LOW);  // Ustawia stan niski na pinie pompy (pompa wyłączona)
  digitalWrite(PIN_BUZZER, LOW);  // Ustawia stan niski na pinie sygnalizatora dźwiękowego (sygnalizator wyłączony)
}

// Funkcja inicjalizująca połączenie WiFi
void initializeWiFi() {
  Serial.print("Łączenie z siecią WiFi: ");
  Serial.println(WIFI_SSID);  // Wyświetla nazwę sieci WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);  // Rozpoczyna próbę połączenia z siecią WiFi
  
  unsigned long connectionStartTime = millis();  // Zapisuje czas rozpoczęcia połączenia
  while (WiFi.status() != WL_CONNECTED && millis() - connectionStartTime < 10000) {
    handleSystemTasks();  // Wykonuje zadania systemowe podczas oczekiwania na połączenie
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Połączono z WiFi");  // Informuje o pomyślnym połączeniu
    Serial.print("Adres IP: ");
    Serial.println(WiFi.localIP());  // Wyświetla adres IP urządzenia
  } else {
    Serial.println("Błąd połączenia WiFi");  // Informuje o nieudanej próbie połączenia
  }
}

// Funkcja obsługująca zadania systemowe
void handleSystemTasks() {
  if (haMqtt.isConnected()) {
    haMqtt.loop();  // Obsługuje komunikację MQTT, jeśli połączono
  }
  ArduinoOTA.handle();  // Obsługuje zadania OTA (Over-the-Air)
}

void playWelcomeMelody() {
  for (int i = 0; i < melody.count; i++) {
    tone(PIN_BUZZER, melody.notes[i]);
    unsigned long noteStartTime = millis();
    while (millis() - noteStartTime < melody.durations[i]) {
      handleSystemTasks();
    }
    noTone(PIN_BUZZER);
    
    noteStartTime = millis();
    while (millis() - noteStartTime < 50) {
      handleSystemTasks();
    }
  }
}

int getStableReading() {
    float measurements[MEASUREMENTS];
    
    for(int i = 0; i < MEASUREMENTS; i++) {
        // Wykonanie pojedynczego pomiaru
        digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(PIN_ULTRASONIC_TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
        
        long duration = pulseIn(PIN_ULTRASONIC_ECHO, HIGH);
        measurements[i] = (duration * 0.343) / 2;  // Obliczenie odległości w mm
        
        delay(MEASUREMENT_DELAY);
        handleSystemTasks();  // Obsługa zadań systemowych podczas oczekiwania
    }
    
    // Sortowanie bąbelkowe
    for(int i = 0; i < MEASUREMENTS-1; i++) {
        for(int j = 0; j < MEASUREMENTS-i-1; j++) {
            if(measurements[j] > measurements[j+1]) {
                float temp = measurements[j];
                measurements[j] = measurements[j+1];
                measurements[j+1] = temp;
            }
        }
    }
    
    // Obliczenie średniej z środkowych wartości
    float sum = 0;
    for(int i = OUTLIERS; i < MEASUREMENTS-OUTLIERS; i++) {
        sum += measurements[i];
    }
    
    return (int)(sum / (MEASUREMENTS - 2*OUTLIERS));
}

// Funkcja wykonująca pomiar odległości przy użyciu czujnika ultradźwiękowego
void performMeasurement() {
    status.currentDistance = getStableReading();
    sensorDistance.setValue(String(status.currentDistance).c_str());
}

void updateSystemStatus() {
  bool isTankFull = status.currentDistance < tankConfig.distanceWhenFull;
  bool isTankEmpty = status.currentDistance > tankConfig.distanceWhenEmpty;
  bool isReserveLevel = status.currentDistance > tankConfig.reserveLevel && !isTankEmpty;
  
  int waterLevelPercentage = map(status.currentDistance, 
                                tankConfig.distanceWhenEmpty, 
                                tankConfig.distanceWhenFull, 
                                0, 100);
  
  // Konwersja wartości na String
  sensorWaterLevel.setValue(String(waterLevelPercentage).c_str());
  sensorReserveStatus.setValue(isReserveLevel ? "ON" : "OFF");
  sensorWaterEmpty.setValue(isTankEmpty ? "ON" : "OFF");
  
  controlPump(isTankFull);
}

void controlPump(bool shouldPumpBeActive) {
  if (shouldPumpBeActive && !status.isPumpActive) {
    digitalWrite(PIN_PUMP, HIGH);
    status.isPumpActive = true;
    sensorPumpStatus.setValue("ON");
  } else if (!shouldPumpBeActive && status.isPumpActive) {
    digitalWrite(PIN_PUMP, LOW);
    status.isPumpActive = false;
    sensorPumpStatus.setValue("OFF");
  }
}

void updatePumpControl() {
    bool currentWaterLevel = digitalRead(PIN_WATER_LEVEL);  // Odczyt stanu czujnika
    
    // Aktualizacja stanu czujnika wody
    status.isWaterLevelLow = currentWaterLevel == LOW;
    
    // Wysoki poziom wody - natychmiastowe wyłączenie pompy
    if (!status.isWaterLevelLow && status.isPumpActive) {
        stopPump();
        status.isPumpDelayActive = false;
        status.pumpStartDelay = 0;
        return;
    }
    
    // Niski poziom wody - rozpocznij odliczanie jeśli jeszcze nie rozpoczęto
    if (status.isWaterLevelLow && !status.isPumpActive && !status.isPumpDelayActive) {
        status.isPumpDelayActive = true;
        status.pumpStartDelay = millis();
        Serial.println("Rozpoczęcie odliczania opóźnienia pompy");
        return;
    }
    
    // Sprawdzenie czy minął czas opóźnienia
    if (status.isPumpDelayActive && !status.isPumpActive) {
        if (millis() - status.pumpStartDelay >= (tankConfig.pumpDelay * 1000)) {
            startPump();
            status.isPumpDelayActive = false;
            return;
        }
    }
    
    // Sprawdzenie czasu pracy pompy
    if (status.isPumpActive) {
        if (millis() - status.lastPumpActivationTime >= (tankConfig.pumpWorkTime * 1000)) {
            stopPump();
            return;
        }
    }
}

void startPump() {
    digitalWrite(PIN_PUMP, HIGH);
    status.isPumpActive = true;
    status.lastPumpActivationTime = millis();
    
    // Wysyłaj do HA tylko jeśli stan się zmienił
    if (status.lastPumpState != status.isPumpActive) {
        sensorPumpStatus.setValue("ON");
        status.lastPumpState = status.isPumpActive;
        Serial.println("Pompa włączona - stan wysłany do HA");
    }
}

void stopPump() {
    digitalWrite(PIN_PUMP, LOW);
    status.isPumpActive = false;
    
    // Wysyłaj do HA tylko jeśli stan się zmienił
    if (status.lastPumpState != status.isPumpActive) {
        sensorPumpStatus.setValue("OFF");
        status.lastPumpState = status.isPumpActive;
        Serial.println("Pompa wyłączona - stan wysłany do HA");
    }
}

void handleButtonPress() {
  if (digitalRead(PIN_RESET_BUTTON) == LOW) {
    tankConfig.alarmActive = false;
    // Używamy setState zamiast setValue dla HASwitch
    switchAlarm.setState(false);  
    unsigned long debounceStartTime = millis();
    while (millis() - debounceStartTime < 50) {
      handleSystemTasks();
    }
  }
}

void setup() {
  initializeSystem();
}

void loop() {
    unsigned long currentTime = millis();
    
    if (!haMqtt.isConnected()) {
        reconnectMQTT();
    }
    
    handleSystemTasks();
    
    if (currentTime - status.lastMeasurementTime >= status.MEASUREMENT_INTERVAL) {
        status.lastMeasurementTime = currentTime;
        performMeasurement();
        updateSystemStatus();
    }
    
    // Dodaj obsługę pompy
    updatePumpControl();
    
    handleButtonPress();
}

void reconnectMQTT() {
  if (!haMqtt.isConnected()) {
    Serial.print("Próba połączenia MQTT... ");
    if (haMqtt.begin(MQTT_SERVER, MQTT_PORT, MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("połączono");
      haMqtt.subscribe("hydrosense/control");
    } else {
      Serial.println("nieudane");
    }
  }
}
