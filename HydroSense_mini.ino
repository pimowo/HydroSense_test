#include <ESP8266WiFi.h>
#include <ArduinoHA.h>

// Konfiguracja WiFi i MQTT
const char* WIFI_SSID = "pimowo";
const char* WIFI_PASSWORD = "ckH59LRZQzCDQFiUgj";
const char* MQTT_SERVER = "192.168.1.14";
const char* MQTT_USER = "hydrosense";
const char* MQTT_PASSWORD = "hydrosense";

// Piny
#define PIN_ULTRASONIC_TRIG D6
#define PIN_ULTRASONIC_ECHO D7
#define PIN_WATER_LEVEL D5
#define PIN_PUMP D1

// Konfiguracja zbiornika
const int DISTANCE_WHEN_FULL = 65;    // mm
const int DISTANCE_WHEN_EMPTY = 510;  // mm
const int PUMP_DELAY = 5;             // sekundy
const int PUMP_WORK_TIME = 60;        // sekundy

// Obiekty do komunikacji
WiFiClient client;
HADevice device("HydroSense");
HAMqtt mqtt(client, device);

// Sensory
HASensor sensorDistance("distance");
HASensor sensorWaterLevel("water_level_percent");
HASensor sensorPumpStatus("pump");
HASensor sensorWaterPresence("water");

// Status systemu
struct {
    bool isPumpActive = false;
    unsigned long pumpStartTime = 0;
    bool isPumpDelayActive = false;
    unsigned long pumpDelayStartTime = 0;
} status;

// Funkcja obliczająca poziom wody w procentach
int calculateWaterLevel(int distance) {
    // Upewnij się, że distance jest w zakresie
    if (distance < DISTANCE_WHEN_FULL) distance = DISTANCE_WHEN_FULL;
    if (distance > DISTANCE_WHEN_EMPTY) distance = DISTANCE_WHEN_EMPTY;
    
    // Oblicz procent wypełnienia
    float percentage = (float)(DISTANCE_WHEN_EMPTY - distance) / 
                      (float)(DISTANCE_WHEN_EMPTY - DISTANCE_WHEN_FULL) * 
                      100.0;
    
    return (int)percentage;
}

void setup() {
    Serial.begin(115200);
    
    // Konfiguracja pinów
    pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
    pinMode(PIN_ULTRASONIC_ECHO, INPUT);
    pinMode(PIN_WATER_LEVEL, INPUT_PULLUP);
    pinMode(PIN_PUMP, OUTPUT);
    digitalWrite(PIN_PUMP, LOW);
    
    // Połączenie z WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nPołączono z WiFi");
    
    // Konfiguracja HA
    device.setName("HydroSense");
    device.setModel("HS ESP8266");
    device.setManufacturer("PMW");
    device.setSoftwareVersion("11.11.24");
    
    sensorDistance.setName("Pomiar odległości");
    sensorDistance.setUnitOfMeasurement("mm");
    sensorDistance.setIcon("mdi:ruler");
    
    sensorWaterLevel.setName("Poziom wody");
    sensorWaterLevel.setUnitOfMeasurement("%");
    sensorWaterLevel.setIcon("mdi:water-percent");
    
    sensorPumpStatus.setName("Status pompy");
    sensorPumpStatus.setIcon("mdi:water-pump");
    
    sensorWaterPresence.setName("Czujnik wody");
    sensorWaterPresence.setIcon("mdi:water");
        
    // Połączenie MQTT
    mqtt.begin(MQTT_SERVER, 1883, MQTT_USER, MQTT_PASSWORD);
}

// Pomiar odległości
int measureDistance() {
    digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
    
    long duration = pulseIn(PIN_ULTRASONIC_ECHO, HIGH, 23529);
    if (duration == 0) return -1;
    
    int distance = (duration * 343) / 2000; // mm
    if (distance >= 20 && distance <= 4000) {
        return distance;
    }
    return -1;
}

// Kontrola pompy
void updatePump() {
    bool waterPresent = (digitalRead(PIN_WATER_LEVEL) == LOW);
    sensorWaterPresence.setValue(waterPresent ? "ON" : "OFF");
    
    // Jeśli nie ma wody, wyłącz pompę
    if (!waterPresent && status.isPumpActive) {
        digitalWrite(PIN_PUMP, LOW);
        status.isPumpActive = false;
        status.pumpStartTime = 0;
        status.isPumpDelayActive = false;
        sensorPumpStatus.setValue("OFF");
        return;
    }
    
    // Jeśli jest woda i pompa nie jest aktywna, rozpocznij odliczanie
    if (waterPresent && !status.isPumpActive && !status.isPumpDelayActive) {
        status.isPumpDelayActive = true;
        status.pumpDelayStartTime = millis();
        return;
    }
    
    // Sprawdź czy minął czas opóźnienia
    if (status.isPumpDelayActive && !status.isPumpActive) {
        if (millis() - status.pumpDelayStartTime >= (PUMP_DELAY * 1000)) {
            digitalWrite(PIN_PUMP, HIGH);
            status.isPumpActive = true;
            status.pumpStartTime = millis();
            status.isPumpDelayActive = false;
            sensorPumpStatus.setValue("ON");
        }
    }
    
    // Sprawdź czas pracy pompy
    if (status.isPumpActive) {
        if (millis() - status.pumpStartTime >= (PUMP_WORK_TIME * 1000)) {
            digitalWrite(PIN_PUMP, LOW);
            status.isPumpActive = false;
            status.pumpStartTime = 0;
            sensorPumpStatus.setValue("OFF");
        }
    }
}

void loop() {
    mqtt.loop();
    
    static unsigned long lastMeasurement = 0;
    if (millis() - lastMeasurement > 1000) {
        // Pomiar odległości
        int distance = measureDistance();
        if (distance > 0) {
            sensorDistance.setValue(String(distance).c_str());
            
            // Obliczanie poziomu wody w procentach
            int waterLevel = calculateWaterLevel(distance);
            sensorWaterLevel.setValue(String(waterLevel).c_str());
            
            Serial.printf("Odległość: %d mm, Poziom: %d%%\n", distance, waterLevel);
        }
        lastMeasurement = millis();
    }
    
    // Aktualizacja pompy
    updatePump();
    
    yield();
}
