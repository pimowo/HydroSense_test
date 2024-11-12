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

// Dodajemy adresy EEPROM
#define EEPROM_SOUND_STATE_ADDR 0

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
HASwitch serviceSwitch("service_mode");     // przełącznik trybu serwisowego
HASwitch soundSwitch("sound_switch");       // przełącznik dźwięku

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
    if (status.isServiceMode && status.isPumpActive) {
        // Wyłącz pompę jeśli jest aktywna
        digitalWrite(PIN_PUMP, LOW);
        status.isPumpActive = false;
        status.pumpStartTime = 0;
        sensorPumpStatus.setValue("OFF");
    }
}

// Obsługa przełącznika dźwięku
void onSoundSwitchCommand(bool state, HASwitch* sender) {
    status.soundEnabled = state;
    EEPROM.write(0, state ? 1 : 0);
    EEPROM.commit();
}

// Funkcja do obsługi przycisku
void handleButton() {
    // Odczyt stanu przycisku
    bool reading = digitalRead(PIN_BUTTON);
    
    // Jeśli stan się zmienił
    if (reading != buttonState.lastState) {
        if (reading == LOW) { // Przycisk wciśnięty
            buttonState.pressedTime = millis();
            buttonState.isLongPressHandled = false;
        } else { // Przycisk zwolniony
            buttonState.releasedTime = millis();
            
            // Krótkie naciśnięcie - przełącz tryb serwisowy
            if (buttonState.releasedTime - buttonState.pressedTime < LONG_PRESS_TIME) {
                status.isServiceMode = !status.isServiceMode;
                serviceSwitch.setState(status.isServiceMode); // Aktualizacja stanu w HA
                if (status.isServiceMode) {
                    Serial.println("Tryb serwisowy: WŁĄCZONY");
                } else {
                    Serial.println("Tryb serwisowy: WYŁĄCZONY");
                }
                
                if (status.isServiceMode && status.isPumpActive) {
                    // Wyłącz pompę jeśli jest aktywna
                    digitalWrite(PIN_PUMP, LOW);
                    status.isPumpActive = false;
                    status.pumpStartTime = 0;
                    sensorPumpStatus.setValue("OFF");
                }
            }
        }
    }
    
    // Sprawdzenie długiego naciśnięcia
    if (reading == LOW && !buttonState.isLongPressHandled) {
        if (millis() - buttonState.pressedTime >= LONG_PRESS_TIME) {
            // Kasowanie alarmu pompy
            status.pumpSafetyLock = false;
            pumpAlarm.setState(false);
            buttonState.isLongPressHandled = true;
            Serial.println("Alarm pompy skasowany");
        }
    }
    
    buttonState.lastState = reading;
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
    
    // Sprawdź czy nie jest aktywny tryb serwisowy
    if (status.isServiceMode) {
        if (status.isPumpActive) {
            digitalWrite(PIN_PUMP, LOW);
            status.isPumpActive = false;
            status.pumpStartTime = 0;
            sensorPumpStatus.setValue("OFF");
        }
        return;
    }

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
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
    pinMode(PIN_PUMP, OUTPUT);
    digitalWrite(PIN_PUMP, LOW);
    
    // Odczyt stanu dźwięku z EEPROM
    status.soundEnabled = EEPROM.read(0) == 1;

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

    sensorWaterAlarm.setName("Brak wody");
    sensorWaterAlarm.setIcon("mdi:water-alert");
    sensorWaterAlarm.setValue("OFF");  // Stan początkowy alarmu wody
    
    sensorWaterReserve.setName("Rezerwa wody");
    sensorWaterReserve.setIcon("mdi:alert-outline");
    sensorWaterReserve.setValue("OFF");    // Stan początkowy rezerwy wody

    serviceSwitch.setName("Serwis");
    serviceSwitch.setIcon("mdi:tools");
    serviceSwitch.onCommand(onServiceSwitchCommand);
    serviceSwitch.setState(status.isServiceMode);
    
    soundSwitch.setName("Dźwięk");
    soundSwitch.setIcon("mdi:volume-high");
    soundSwitch.onCommand(onSoundSwitchCommand);
    soundSwitch.setState(status.soundEnabled);

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
        int distance = measureDistance();
        if (distance > 0) {
            sensorDistance.setValue(String(distance).c_str());
            
            int waterLevel = calculateWaterLevel(distance);
            sensorWaterLevel.setValue(String(waterLevel).c_str());
            
            // Dodaj sprawdzenie poziomów alarmowych
            checkWaterLevels(distance);
            
            //Serial.printf("Odległość: %d mm, Poziom: %d%%\n", distance, waterLevel);
        }
        lastMeasurement = millis();
    }
    
    // Modyfikacja updatePump()
    updatePump();
    
    yield();
}
