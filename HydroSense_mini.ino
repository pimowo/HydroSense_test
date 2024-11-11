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
HASensor sensorDistance("distance"); // Pomiar odległości w mm
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
  int currentDistance;
  bool isPumpActive;
  unsigned long lastMeasurementTime;
  unsigned long lastPumpActivationTime;
  const unsigned long MEASUREMENT_INTERVAL = 60000;  // 60 sekund (1 minuta) między cyklami pomiarów
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
  bool alarmActive = false;
  bool waterEmpty = false;
  bool buzzerActive = false;
} tankConfig;

// Stałe dla czujnika ultradźwiękowego
const int MAX_DISTANCE_MM = 4000;  // maksymalny dystans w mm
const int MIN_DISTANCE_MM = 20;    // minimalny dystans w mm
const unsigned long MEASUREMENT_TIMEOUT = 23529; // timeout dla 4m
const unsigned long MEASUREMENT_INTERVAL = 100;  // 100ms między pomiarami

// Zmienne globalne dla pomiarów
unsigned long lastMeasurementTime = 0;
int lastValidDistance = 0;

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

  // Konfiguracja urządzenia dla Home Assistant
  device.setName("HydroSense");  // Ustaw nazwę urządzenia
  device.setModel("HS ESP8266");  // Ustaw model urządzenia
  device.setManufacturer("PMW");  // Ustaw producenta
  device.setSoftwareVersion("11.11.24");  // Ustaw wersję oprogramowania
  // Czujnik wody
  sensorWaterPresence.setName("Czujnik wody");
  sensorWaterPresence.setIcon("mdi:water"); 
  // Pompa
  sensorPumpStatus.setName("Pompa");
  sensorPumpStatus.setIcon("mdi:water-pump");
  sensorPumpStatus.setValue("OFF");
  // Czujnik odległości
  sensorDistance.setName("Pomiar odległości");
  sensorDistance.setIcon("mdi:ruler");
  sensorDistance.setUnitOfMeasurement("mm");
  // Poziom wody
  sensorWaterLevel.setName("Zapełnienie zbiornika");
  sensorWaterLevel.setIcon("mdi:water-percent");
  sensorWaterLevel.setUnitOfMeasurement("%");
  // Oobjętość wody
  sensorWaterVolume.setName("Ilość wody");
  sensorWaterVolume.setIcon("mdi:cup-water");
  sensorWaterVolume.setUnitOfMeasurement("l");
  // Rezerwa
  sensorReserveStatus.setName("Rezerwa");
  sensorReserveStatus.setIcon("mdi:alert-outline");
  // Brak wody
  sensorWaterEmpty.setName("Brak wody");
  sensorWaterEmpty.setIcon("mdi:water-off");
  // Dźwięk
  switchBuzzer.setName("Dzwięk");
  switchBuzzer.setIcon("mdi:bell");
  // Alarm
  switchAlarm.setName("Alarm");
  switchAlarm.setIcon("mdi:alert");
  // Serwis
  switchServiceMode.setName("Serwis");
  switchServiceMode.setIcon("mdi:tools");
  
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

// Funkcja dodająca pomiar do bufora
void addMeasurement(int measurement) {
    if (measurement <= 0) {
        return;
    }
    
    distanceBuffer.values[distanceBuffer.index] = measurement;
    distanceBuffer.index = (distanceBuffer.index + 1) % BUFFER_SIZE;
    
    if (distanceBuffer.index == 0) {
        distanceBuffer.isFull = true;
    }
}

// Funkcja obliczająca średnią z bufora
int calculateAverage() {
    if (!distanceBuffer.isFull) {
        return -1;
    }
    
    int sum = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        sum += distanceBuffer.values[i];
    }
    
    return sum / BUFFER_SIZE;
}

// Pojedynczy pomiar
int takeSingleMeasurement() {
    ESP.wdtFeed();
    
    digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
    
    yield();
    
    unsigned long duration = pulseIn(PIN_ULTRASONIC_ECHO, HIGH, MEASUREMENT_TIMEOUT);
    
    if (duration == 0) {
        return -1;
    }
    
    // Przeliczenie na dystans w mm
    int distance = (int)((duration * 0.343) / 2);
    
    if (distance >= MIN_DISTANCE_MM && distance <= MAX_DISTANCE_MM) {
        return distance;
    }
    
    return -1;
}

int measureDistance() {
    ESP.wdtFeed();
    
    digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
    
    yield();
    
    unsigned long duration = pulseIn(PIN_ULTRASONIC_ECHO, HIGH, MEASUREMENT_TIMEOUT);
    
    if (duration == 0) {
        return -1;  // Błąd pomiaru
    }
    
    int distance = (int)((duration * 0.343) / 2);
    
    if (distance >= MIN_DISTANCE_MM && distance <= MAX_DISTANCE_MM) {
        return distance;
    }
    
    return -1;  // Wynik poza zakresem
}

// Główna funkcja pomiaru
void performMeasurement() {
    unsigned long currentTime = millis();
    
    // Sprawdź czy minął wymagany czas między pomiarami
    if (currentTime - lastMeasurementTime < MEASUREMENT_INTERVAL) {
        return;
    }
    
    lastMeasurementTime = currentTime;
    
    // Wykonaj 3 pomiary i weź średnią
    int sum = 0;
    int validMeasurements = 0;
    
    for (int i = 0; i < 3; i++) {
        int measurement = measureDistance();
        if (measurement > 0) {
            sum += measurement;
            validMeasurements++;
        }
        yield();
    }
    
    // Jeśli mamy przynajmniej 2 prawidłowe pomiary
    if (validMeasurements >= 2) {
        int avgDistance = sum / validMeasurements;
        
        // Sprawdź czy zmiana nie jest zbyt duża
        if (abs(avgDistance - lastValidDistance) < 100 || lastValidDistance == 0) {
            lastValidDistance = avgDistance;
            status.currentDistance = avgDistance;
            sensorDistance.setValue(String(avgDistance).c_str());
        }
    }
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

    measurementBuffer.index = 0;
    measurementBuffer.isFull = false;
    measurementBuffer.lastMeasurementTime = 0;
    measurementBuffer.lastValidDistance = 0;
    memset(measurementBuffer.values, 0, sizeof(measurementBuffer.values));
}

void loop() {
    static unsigned long lastWDTFeed = 0;
    unsigned long currentTime = millis();
    
    // Karm watchdoga co 50ms
    if (currentTime - lastWDTFeed >= 50) {
        ESP.wdtFeed();
        lastWDTFeed = currentTime;
    }
    
    if (!haMqtt.isConnected()) {
        reconnectMQTT();
        yield();
    }
    
    handleSystemTasks();
    yield();
    
    performMeasurement();
    yield();
    
    if (currentTime - status.lastMeasurementTime >= status.MEASUREMENT_INTERVAL) {
        status.lastMeasurementTime = currentTime;
        updateSystemStatus();
        yield();
    }
    
    updatePumpControl();
    yield();
    
    handleButtonPress();
    yield();
    
    delay(1);
}
