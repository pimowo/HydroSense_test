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

// Konfiguracja zbiornika i pomiarów
const int DISTANCE_WHEN_FULL = 65;  // pełny zbiornik - mm
const int DISTANCE_WHEN_EMPTY = 510;  // pusty zbiornik - mm
const int PUMP_DELAY = 5;  // opóźnienie włączenia pompy - sekundy
const int PUMP_WORK_TIME = 60;  // czas pracy pompy - sekundy
const int MEASUREMENTS_COUNT = 5;  // liczba pomiarów do uśrednienia
const int HYSTERESIS = 10;  // histereza - mm
const int DISTANCE_RESERVE = 400;  // dystans dla rezerwy - mm

// Obiekty do komunikacji
WiFiClient client;
HADevice device("HydroSense");
HAMqtt mqtt(client, device);
// Sensory pomiarowe
HASensor sensorDistance("distance");                    // odległość w mm
HASensor sensorWaterLevel("water_level_percent");       // poziom wody w %
// Sensory statusu
HASensor sensorPumpStatus("pump");                     // status pompy
HASensor sensorWaterPresence("water");                 // obecność wody (czujnik)
// Sensory alarmowe
HASensor sensorWaterAlarm("water_alarm");             // alarm niskiego poziomu
HASensor sensorWaterReserve("water_reserve");         // stan rezerwy
HASwitch pumpAlarm("pump_alarm");                     // alarm przepracowania pompy

// Status systemu
struct {
    bool isPumpActive = false;
    unsigned long pumpStartTime = 0;
    bool isPumpDelayActive = false;
    unsigned long pumpDelayStartTime = 0;
    bool pumpSafetyLock = false;
    bool waterAlarmActive = false;   
    bool waterReserveActive = false;
} status;

// Funkcja obliczająca poziom wody w procentach
int calculateWaterLevel(int distance) {
    if (distance < DISTANCE_WHEN_FULL) distance = DISTANCE_WHEN_FULL;
    if (distance > DISTANCE_WHEN_EMPTY) distance = DISTANCE_WHEN_EMPTY;
    
    float percentage = (float)(DISTANCE_WHEN_EMPTY - distance) / 
                      (float)(DISTANCE_WHEN_EMPTY - DISTANCE_WHEN_FULL) * 
                      100.0;
    
    return (int)percentage;
}

// Ulepszona funkcja pomiaru odległości z uśrednianiem
int measureDistance() {
    int measurements[MEASUREMENTS_COUNT];
    int validMeasurements = 0;
    
    // Wykonaj serie pomiarów
    for(int i = 0; i < MEASUREMENTS_COUNT; i++) {
        digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(PIN_ULTRASONIC_TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
        
        long duration = pulseIn(PIN_ULTRASONIC_ECHO, HIGH, 23529);
        if (duration == 0) {
            Serial.println("Błąd pomiaru - timeout");
            continue;
        }
        
        int distance = (duration * 343) / 2000; // mm
        if (distance >= 20 && distance <= 4000) {
            measurements[validMeasurements] = distance;
            validMeasurements++;
            Serial.printf("Pomiar %d: %d mm\n", i+1, distance);
        } else {
            Serial.printf("Pomiar %d: poza zakresem (%d mm)\n", i+1, distance);
        }
        
        delay(50); // krótka przerwa między pomiarami
    }
    
    // Jeśli nie ma wystarczającej liczby poprawnych pomiarów
    if (validMeasurements < 3) {
        Serial.println("Za mało poprawnych pomiarów");
        return -1;
    }
    
    // Sortowanie pomiarów (aby usunąć skrajne wartości)
    for(int i = 0; i < validMeasurements-1; i++) {
        for(int j = 0; j < validMeasurements-i-1; j++) {
            if(measurements[j] > measurements[j+1]) {
                int temp = measurements[j];
                measurements[j] = measurements[j+1];
                measurements[j+1] = temp;
            }
        }
    }
    
    // Obliczenie średniej z pomiarów (pomijając skrajne wartości)
    long sum = 0;
    int start = validMeasurements > 3 ? 1 : 0;
    int end = validMeasurements > 3 ? validMeasurements-1 : validMeasurements;
    
    for(int i = start; i < end; i++) {
        sum += measurements[i];
    }
    
    int average = sum / (end - start);
    Serial.printf("Średnia z %d pomiarów: %d mm\n", end-start, average);
    
    return average;
}

void onPumpAlarmCommand(bool state, HASwitch* sender) {
    if (!state) {
        status.pumpSafetyLock = false;
    }
}

void checkWaterLevels(int distance) {
    // Sprawdzenie alarmu wody
    if (distance >= DISTANCE_WHEN_EMPTY && !status.waterAlarmActive) {
        status.waterAlarmActive = true;
        sensorWaterAlarm.setValue("ON");
        Serial.println("Alarm: Krytycznie niski poziom wody!");
    } else if (distance < (DISTANCE_WHEN_EMPTY - HYSTERESIS) && status.waterAlarmActive) {
        status.waterAlarmActive = false;
        sensorWaterAlarm.setValue("OFF");
        Serial.println("Alarm wody wyłączony");
    }

    // Sprawdzenie rezerwy
    if (distance >= DISTANCE_RESERVE && !status.waterReserveActive) {
        status.waterReserveActive = true;
        sensorWaterReserve.setValue("ON");
        Serial.println("Uwaga: Poziom rezerwy!");
    } 
    else if (distance < (DISTANCE_RESERVE - HYSTERESIS) && status.waterReserveActive) {
        status.waterReserveActive = false;
        sensorWaterReserve.setValue("OFF");
        Serial.println("Poziom powyżej rezerwy");
    }
}

// Kontrola pompy
void updatePump() {
    bool waterPresent = (digitalRead(PIN_WATER_LEVEL) == LOW);
    sensorWaterPresence.setValue(waterPresent ? "ON" : "OFF");
    
    // Sprawdź czy pompa nie pracuje za długo
    if (status.isPumpActive && (millis() - status.pumpStartTime > PUMP_WORK_TIME * 1000)) {
        digitalWrite(PIN_PUMP, LOW);
        status.isPumpActive = false;
        status.pumpStartTime = 0;
        sensorPumpStatus.setValue("OFF");
        status.pumpSafetyLock = true;
        pumpAlarm.setState(true);
        Serial.println("ALARM: Pompa pracowała za długo - aktywowano blokadę bezpieczeństwa!");
        return;
    }
    
    // Sprawdź blokady bezpieczeństwa
    if (status.pumpSafetyLock || status.waterAlarmActive) {
        if (status.isPumpActive) {
            digitalWrite(PIN_PUMP, LOW);
            status.isPumpActive = false;
            status.pumpStartTime = 0;
            sensorPumpStatus.setValue("OFF");
        }
        return;
    }
    
    // Reszta istniejącej logiki pozostaje bez zmian
    if (!waterPresent && status.isPumpActive) {
        digitalWrite(PIN_PUMP, LOW);
        status.isPumpActive = false;
        status.pumpStartTime = 0;
        status.isPumpDelayActive = false;
        sensorPumpStatus.setValue("OFF");
        return;
    }
    
    if (waterPresent && !status.isPumpActive && !status.isPumpDelayActive) {
        status.isPumpDelayActive = true;
        status.pumpDelayStartTime = millis();
        return;
    }
    
    if (status.isPumpDelayActive && !status.isPumpActive) {
        if (millis() - status.pumpDelayStartTime >= (PUMP_DELAY * 1000)) {
            digitalWrite(PIN_PUMP, HIGH);
            status.isPumpActive = true;
            status.pumpStartTime = millis();
            status.isPumpDelayActive = false;
            sensorPumpStatus.setValue("ON");
        }
    }
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
    sensorDistance.setIcon("mdi:ruler");
    sensorDistance.setUnitOfMeasurement("mm");
    
    sensorWaterLevel.setName("Poziom wody");
    sensorWaterLevel.setIcon("mdi:water-percent");
    sensorWaterLevel.setUnitOfMeasurement("%");
        
    sensorPumpStatus.setName("Status pompy");
    sensorPumpStatus.setIcon("mdi:water-pump");
    
    sensorWaterPresence.setName("Czujnik wody");
    sensorWaterPresence.setIcon("mdi:water");

    sensorWaterAlarm.setName("Alarm wody");
    sensorWaterAlarm.setIcon("mdi:tools");
    
    sensorWaterReserve.setName("Rezerwa wody");
    sensorWaterReserve.setIcon("mdi:alert-outline");

    //sensorLowWaterAlarm.setName("Alarm niskiego poziomu");

    // serviceSwitch.setName("Serwis");
    // serviceSwitch.setIcon("mdi:tools");

    pumpAlarm.setName("Alarm pompy");
    pumpAlarm.setIcon("mdi:alert");
    pumpAlarm.onCommand(onPumpAlarmCommand);

    // pumpAlarm.setName("Alarm wody");
    // pumpAlarm.setIcon("mdi:alert");
    // pumpAlarm.onCommand(onPumpAlarmCommand);
    
    // Połączenie MQTT
    mqtt.begin(MQTT_SERVER, 1883, MQTT_USER, MQTT_PASSWORD);
}

void loop() {
    mqtt.loop();
    
    static unsigned long lastMeasurement = 0;
    if (millis() - lastMeasurement > 1000) {
        int distance = measureDistance();
        if (distance > 0) {
            sensorDistance.setValue(String(distance).c_str());
            
            int waterLevel = calculateWaterLevel(distance);
            sensorWaterLevel.setValue(String(waterLevel).c_str());
            
            // Dodaj sprawdzenie poziomów alarmowych
            checkWaterLevels(distance);
            
            Serial.printf("Odległość: %d mm, Poziom: %d%%\n", distance, waterLevel);
        }
        lastMeasurement = millis();
    }
    
    // Modyfikacja updatePump()
    updatePump();
    
    yield();
}
