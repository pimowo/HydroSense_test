#include <ESP8266WiFi.h>
#include <ArduinoHA.h>
#include <EEPROM.h>

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
#define PIN_BUZZER D2  // Buzzer
#define PIN_BUTTON D3  // Przycisk kasowania alarmu

// Stałe dla przycisku
#define LONG_PRESS_TIME 1000  // 1 sekunda dla długiego naciśnięcia

// EEPROM
#define EEPROM_SIZE 512
// Dodajemy adresy EEPROM
#define EEPROM_SOUND_STATE_ADDR 0  // Adres w EEPROM dla stanu dźwięku

// Konfiguracja zbiornika i pomiarów
const int DISTANCE_WHEN_FULL = 65;  // pełny zbiornik - mm
const int DISTANCE_WHEN_EMPTY = 510;  // pusty zbiornik - mm
const int DISTANCE_RESERVE = 450;  // dystans dla rezerwy - mm
const int HYSTERESIS = 10;  // histereza - mm
const int TANK_DIAMETER = 150;  // Średnica zbiornika - mm
const int MEASUREMENTS_COUNT = 5;  // liczba pomiarów do uśrednienia
const int PUMP_DELAY = 5;  // opóźnienie włączenia pompy - sekundy
const int PUMP_WORK_TIME = 60;  // czas pracy pompy - sekundy

// Konfiguracja zbiornika
const struct TankConfig {
    static const int DISTANCE_FULL = 65;    // mm
    static const int DISTANCE_EMPTY = 510;  // mm
    static const int DISTANCE_RESERVE = 450; // mm
    static const int DIAMETER = 150;        // mm
    static const int HYSTERESIS = 10;       // mm
} TANK;

float currentDistance = 0;
float volume = 0;            // Dodajemy też zmienną volume jako globalną
float fillPercentage = 0;    // i fillPercentage

// Obiekty do komunikacji
WiFiClient client;
HADevice device("HydroSense");
HAMqtt mqtt(client, device);
// Sensory pomiarowe
HASensor sensorDistance("water_level");              // było waterLevelSensor
HASensor sensorLevel("water_level_percent");         // było sensorWaterLevel
HASensor sensorVolume("water_volume");               // było sensorWaterVolume

// Sensory statusu
HASensor sensorPump("pump");                        // było sensorPumpStatus
HASensor sensorWater("water");                      // było sensorWaterPresence

// Sensory alarmowe
HASensor sensorAlarm("water_alarm");                // było sensorWaterAlarm
HASensor sensorReserve("water_reserve");            // było sensorWaterReserve

// Przełączniki
HASwitch switchPump("pump_alarm");                  // było pumpAlarm
HASwitch switchService("service_mode");             // było serviceSwitch
HASwitch switchSound("sound_switch");               // było soundSwitch
// Status systemu
struct {
    bool isPumpActive = false;
    unsigned long pumpStartTime = 0;
    bool isPumpDelayActive = false;
    unsigned long pumpDelayStartTime = 0;
    bool pumpSafetyLock = false;
    bool waterAlarmActive = false;   
    bool waterReserveActive = false;
    bool soundEnabled = false;        // stan dźwięku
    bool isServiceMode = false;      // stan trybu serwisowego
} status;

// Struktura dla obsługi przycisku
struct ButtonState {
    bool currentState = HIGH;
    bool lastState = HIGH;
    unsigned long pressedTime = 0;
    unsigned long releasedTime = 0;
    bool isLongPressHandled = false;
} buttonState;

// Funkcja zapisująca stan do EEPROM
void saveToEEPROM() {
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.write(EEPROM_SOUND_STATE_ADDR, status.soundEnabled ? 1 : 0);
    bool saved = EEPROM.commit();
    EEPROM.end();
    Serial.printf("Zapisano stan dźwięku do EEPROM: %s (Status: %s)\n", 
                 status.soundEnabled ? "WŁĄCZONY" : "WYŁĄCZONY",
                 saved ? "OK" : "BŁĄD");
}

// Funkcja odczytująca stan z EEPROM
void loadFromEEPROM() {
    EEPROM.begin(EEPROM_SIZE);
    status.soundEnabled = EEPROM.read(EEPROM_SOUND_STATE_ADDR) == 1;
    EEPROM.end();
    Serial.printf("Wczytano stan dźwięku z EEPROM: %s\n", 
                 status.soundEnabled ? "WŁĄCZONY" : "WYŁĄCZONY");
}

// Funkcja obliczająca poziom wody w procentach
int calculateWaterLevel(int distance) {
    if (distance < DISTANCE_WHEN_FULL) distance = DISTANCE_WHEN_FULL;
    if (distance > DISTANCE_WHEN_EMPTY) distance = DISTANCE_WHEN_EMPTY;
    
    float percentage = (float)(DISTANCE_WHEN_EMPTY - distance) / 
                      (float)(DISTANCE_WHEN_EMPTY - DISTANCE_WHEN_FULL) * 
                      100.0;
    
    return (int)percentage;
}

// Obsługa przełącznika trybu serwisowego
void onServiceSwitchCommand(bool state, HASwitch* sender) {
    status.isServiceMode = state;
    buttonState.lastState = HIGH;
    
    // Aktualizuj stan przełącznika w HA
    switchService.setState(state);   // było serviceSwitch
    
    if (status.isServiceMode && status.isPumpActive) {
        digitalWrite(PIN_PUMP, LOW);
        status.isPumpActive = false;
        status.pumpStartTime = 0;
        sensorPump.setValue("OFF");  // było sensorPumpStatus
    }
    Serial.printf("Tryb serwisowy: %s (przez HA)\n", state ? "WŁĄCZONY" : "WYŁĄCZONY");
}

// Obsługa przełącznika dźwięku
void onSoundSwitchCommand(bool state, HASwitch* sender) {
    status.soundEnabled = state;
    
    // Zapisz do EEPROM
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.write(EEPROM_SOUND_STATE_ADDR, state ? 1 : 0);
    bool saved = EEPROM.commit();
    
    // Aktualizuj stan w HA
    switchSound.setState(state);  // było soundSwitch
    Serial.printf("Zmieniono stan dźwięku na: %s\n", state ? "WŁĄCZONY" : "WYŁĄCZONY");
}

// Funkcja do obsługi przycisku
void handleButton() {
    bool reading = digitalRead(PIN_BUTTON);
    
    // Jeśli stan się zmienił
    if (reading != buttonState.lastState) {
        if (reading == LOW) {
            buttonState.pressedTime = millis();
            buttonState.isLongPressHandled = false;
        } else {
            buttonState.releasedTime = millis();
            
            if (buttonState.releasedTime - buttonState.pressedTime < LONG_PRESS_TIME) {
                // Przełącz tryb serwisowy na przeciwny do aktualnego
                status.isServiceMode = !status.isServiceMode;
                // Aktualizuj stan w HA
                switchService.setState(status.isServiceMode);
                
                Serial.printf("Tryb serwisowy: %s (przez przycisk)\n", 
                            status.isServiceMode ? "WŁĄCZONY" : "WYŁĄCZONY");
                
                if (status.isServiceMode && status.isPumpActive) {
                    digitalWrite(PIN_PUMP, LOW);
                    status.isPumpActive = false;
                    status.pumpStartTime = 0;
                    sensorPump.setValue("OFF");
                }
            }
        }
    }
    
    // Sprawdzenie długiego naciśnięcia
    if (reading == LOW && !buttonState.isLongPressHandled) {
        if (millis() - buttonState.pressedTime >= LONG_PRESS_TIME) {
            status.pumpSafetyLock = false;
            switchPump.setState(false);
            buttonState.isLongPressHandled = true;
            Serial.println("Alarm pompy skasowany");
        }
    }
    
    buttonState.lastState = reading;
    yield(); // Daj czas na obsługę innych zadań
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
            //Serial.println("Błąd pomiaru - timeout");
            continue;
        }
        
        int distance = (duration * 343) / 2000; // mm
        if (distance >= 20 && distance <= 4000) {
            measurements[validMeasurements] = distance;
            validMeasurements++;
            //Serial.printf("Pomiar %d: %d mm\n", i+1, distance);
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
    //Serial.printf("Średnia z %d pomiarów: %d mm\n", end-start, average);
    
    return average;
}

void onPumpAlarmCommand(bool state, HASwitch* sender) {
    if (!state) {
        status.pumpSafetyLock = false;
    }
}

// Kontrola pompy
void updatePump() {
    bool waterPresent = (digitalRead(PIN_WATER_LEVEL) == LOW);
    sensorWater.setValue(waterPresent ? "ON" : "OFF");
    
    // Sprawdź czy nie jest aktywny tryb serwisowy
    if (status.isServiceMode) {
        if (status.isPumpActive) {
            digitalWrite(PIN_PUMP, LOW);
            status.isPumpActive = false;
            status.pumpStartTime = 0;
            sensorPump.setValue("OFF");
        }
        return;
    }

    // Sprawdź czy pompa nie pracuje za długo
    if (status.isPumpActive && (millis() - status.pumpStartTime > PUMP_WORK_TIME * 1000)) {
        digitalWrite(PIN_PUMP, LOW);
        status.isPumpActive = false;
        status.pumpStartTime = 0;
        sensorPump.setValue("OFF");
        status.pumpSafetyLock = true;
        switchPump.setState(true);
        Serial.println("ALARM: Pompa pracowała za długo - aktywowano blokadę bezpieczeństwa!");
        return;
    }
    
    // Sprawdź blokady bezpieczeństwa
    if (status.pumpSafetyLock || status.waterAlarmActive) {
        if (status.isPumpActive) {
            digitalWrite(PIN_PUMP, LOW);
            status.isPumpActive = false;
            status.pumpStartTime = 0;
            sensorPump.setValue("OFF");
        }
        return;
    }
    
    // Reszta istniejącej logiki pozostaje bez zmian
    if (!waterPresent && status.isPumpActive) {
        digitalWrite(PIN_PUMP, LOW);
        status.isPumpActive = false;
        status.pumpStartTime = 0;
        status.isPumpDelayActive = false;
        sensorPump.setValue("OFF");
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
            sensorPump.setValue("ON");
        }
    }
}

void updateAlarmStates(float currentDistance) {
    // Sprawdzenie alarmu braku wody z histerezą
    if (currentDistance >= DISTANCE_WHEN_EMPTY && !status.waterAlarmActive) {
        status.waterAlarmActive = true;
        sensorAlarm.setValue("ON");                // było sensorWaterAlarm
        Serial.println("Alarm: Krytycznie niski poziom wody!");
    } 
    else if (currentDistance < (DISTANCE_WHEN_EMPTY - HYSTERESIS) && status.waterAlarmActive) {
        status.waterAlarmActive = false;
        sensorAlarm.setValue("OFF");               // było sensorWaterAlarm
        Serial.println("Alarm wody wyłączony");
    }

    // Sprawdzenie stanu rezerwy z histerezą
    if (currentDistance >= DISTANCE_RESERVE && !status.waterReserveActive) {
        status.waterReserveActive = true;
        sensorReserve.setValue("ON");             // było sensorWaterReserve
        Serial.println("Uwaga: Poziom rezerwy!");
    } 
    else if (currentDistance < (DISTANCE_RESERVE - HYSTERESIS) && status.waterReserveActive) {
        status.waterReserveActive = false;
        sensorReserve.setValue("OFF");            // było sensorWaterReserve
        Serial.println("Poziom powyżej rezerwy");
    }
}

void updateWaterLevel() {
    currentDistance = measureDistance();
    if (currentDistance < 0) return; // błąd pomiaru
    
    // Obliczenie objętości
    float waterHeight = DISTANCE_WHEN_EMPTY - currentDistance;
    waterHeight = constrain(waterHeight, 0, DISTANCE_WHEN_EMPTY - DISTANCE_WHEN_FULL);
    
    // Obliczenie objętości w litrach (wszystko w mm)
    float radius = TANK_DIAMETER / 2.0;
    volume = PI * (radius * radius) * waterHeight / 1000000.0; // mm³ na litry
    
    // Aktualizacja sensorów pomiarowych
    sensorDistance.setValue(String((int)currentDistance).c_str());
    sensorLevel.setValue(String(calculateWaterLevel(currentDistance)).c_str());
    
    char valueStr[10];
    dtostrf(volume, 1, 1, valueStr);
    sensorVolume.setValue(valueStr);
    
    // Aktualizacja stanów alarmowych
    updateAlarmStates(currentDistance);
    
    // Debug info tylko gdy wartości się zmieniły znacząco
    static float lastReportedDistance = 0;
    if (abs(currentDistance - lastReportedDistance) > 5) {
        Serial.printf("Poziom: %.1f mm, Obj: %.1f L\n", 
                     currentDistance, volume);
        lastReportedDistance = currentDistance;
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("\nStarting HydroSense...");
    
    // Najpierw inicjalizujemy EEPROM
    EEPROM.begin(EEPROM_SIZE);
    delay(100); // Krótkie opóźnienie dla stabilności
    
    // Wczytaj stan dźwięku
    uint8_t savedState = EEPROM.read(EEPROM_SOUND_STATE_ADDR);
    status.soundEnabled = (savedState == 1);
    Serial.printf("Wczytano stan dźwięku z EEPROM (adres %d): %d -> %s\n", 
                 EEPROM_SOUND_STATE_ADDR, 
                 savedState,
                 status.soundEnabled ? "WŁĄCZONY" : "WYŁĄCZONY");
    
    // Reszta konfiguracji pinów
    pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
    pinMode(PIN_ULTRASONIC_ECHO, INPUT);
    pinMode(PIN_WATER_LEVEL, INPUT_PULLUP);
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
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
       
    // Konfiguracja sensorów pomiarowych
    sensorDistance.setName("Odległość od lustra wody");
    sensorDistance.setIcon("mdi:ruler");
    sensorDistance.setUnitOfMeasurement("mm");
    
    sensorWaterLevel.setName("Poziom wody");
    sensorWaterLevel.setIcon("mdi:water-percent");
    sensorWaterLevel.setUnitOfMeasurement("%");
    
    sensorVolume.setName("Objętość wody");
    sensorVolume.setIcon("mdi:water");
    sensorVolume.setUnitOfMeasurement("L");
    
    // Konfiguracja sensorów statusu
    sensorPump.setName("Status pompy");
    sensorPump.setIcon("mdi:water-pump");
    
    sensorWater.setName("Czujnik wody");
    sensorWater.setIcon("mdi:water");
    
    // Konfiguracja sensorów alarmowych
    sensorAlarm.setName("Brak wody");
    sensorAlarm.setIcon("mdi:water-alert");
    sensorAlarm.setValue("OFF");
    
    sensorReserve.setName("Rezerwa wody");
    sensorReserve.setIcon("mdi:alert-outline");
    sensorReserve.setValue("OFF");
    
    // Konfiguracja przełączników
    service.setName("Serwis");
    service.setIcon("mdi:tools");
    service.onCommand(onServiceSwitchCommand);
    service.setState(status.isServiceMode);
    
    sound.setName("Dźwięk");
    sound.setIcon("mdi:volume-high");
    sound.onCommand(onSoundSwitchCommand);
    sound.setState(status.soundEnabled);
    
    pumpAlarm.setName("Alarm pompy");
    pumpAlarm.setIcon("mdi:alert");
    pumpAlarm.onCommand(onPumpAlarmCommand);

    // Połączenie MQTT
    mqtt.begin(MQTT_SERVER, 1883, MQTT_USER, MQTT_PASSWORD);
}

void loop() {
    mqtt.loop();
    
    // Obsługa przycisku
    handleButton();

    static unsigned long lastMeasurement = 0;
    if (millis() - lastMeasurement > 1000) {
        updateWaterLevel();  // Wszystkie pomiary i aktualizacje w jednej funkcji
        lastMeasurement = millis();
    }
    
    // Obsługa pompy
    updatePump();
    
    yield();
}
