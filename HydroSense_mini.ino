// Biblioteki
#include <ESP8266WiFi.h> // Biblioteka do obsługi WiFi dla ESP8266
#include <ArduinoHA.h> // Biblioteka do integracji z Home Assistant
#include <PubSubClient.h>
#include <EEPROM.h> // Biblioteka do obsługi pamięci EEPROM

// Konfiguracja WiFi i MQTT
const char* WIFI_SSID = "pimowo"; // Nazwa sieci WiFi
const char* WIFI_PASSWORD = "ckH59LRZQzCDQFiUgj"; // Hasło do sieci WiFi
const char* MQTT_SERVER = "192.168.1.14"; // Adres IP serwera MQTT (Home Assistant)
const char* MQTT_USER = "hydrosense"; // Użytkownik MQTT
const char* MQTT_PASSWORD = "hydrosense"; // Hasło MQTT

// Definicje pinów ESP8266

// Piny dla czujnika HC-SR04
#define PIN_ULTRASONIC_TRIG D6 // Pin TRIG czujnika ultradźwiękowego
#define PIN_ULTRASONIC_ECHO D7 // Pin ECHO czujnika ultradźwiękowego

#define PIN_WATER_LEVEL D5 // Pin czujnika poziomu wody w akwarium
#define PIN_PUMP D1 // Pin sterowania pompą
#define PIN_BUZZER D2 // Pin buzzera do alarmów dźwiękowych
#define PIN_BUTTON D3 // Pin przycisku do kasowania alarmów

// Zmienne czasowe dla nieblokującego pomiaru odległości
bool ultrasonicInProgress = false; // Flaga trwającego pomiaru
unsigned long lastUltrasonicTrigger = 0; // Czas ostatniego wyzwolenia pomiaru
const unsigned long ULTRASONIC_TIMEOUT = 50; // Timeout pomiaru w ms
const unsigned long MEASUREMENT_INTERVAL = 10000;// Interwał między pomiarami w ms
const unsigned long MQTT_RETRY_INTERVAL = 5000;// Czas między próbami połączenia MQTT w ms
const unsigned long WIFI_CHECK_INTERVAL = 5000;// Czas między sprawdzeniami WiFi w ms
const unsigned long WATCHDOG_TIMEOUT = 8000; // Timeout dla watchdoga w ms

// Konfiguracja przycisku
#define LONG_PRESS_TIME 1000 // Czas długiego naciśnięcia w ms (1 sekunda)

// Konfiguracja EEPROM
#define EEPROM_SIZE 512 // Rozmiar używanej pamięci EEPROM w bajtach

// Adresy w pamięci EEPROM
#define EEPROM_SOUND_STATE_ADDR 0 // Adres przechowywania stanu dźwięku (1 bajt)

// Konfiguracja zbiornika i pomiarów
const int DISTANCE_WHEN_FULL = 65;  // pełny zbiornik - mm
const int DISTANCE_WHEN_EMPTY = 510;  // pusty zbiornik - mm
const int DISTANCE_RESERVE = 450;  // dystans dla rezerwy - mm
const int HYSTERESIS = 10;  // histereza - mm
const int TANK_DIAMETER = 150;  // Średnica zbiornika - mm
const int MEASUREMENTS_COUNT = 3;  // liczba pomiarów do uśrednienia
const int PUMP_DELAY = 5;  // opóźnienie włączenia pompy - sekundy
const int PUMP_WORK_TIME = 60;  // czas pracy pompy - sekundy

float currentDistance = 0;
float volume = 0;            // Dodajemy też zmienną volume jako globalną

// Obiekty do komunikacji
WiFiClient client; // Klient połączenia WiFi
HADevice device("HydroSense"); // Definicja urządzenia dla Home Assistant
HAMqtt mqtt(client, device); // Klient MQTT dla Home Assistant

// Sensory pomiarowe
HASensor sensorDistance("water_level"); // Odległość od lustra wody w mm
HASensor sensorLevel("water_level_percent"); // Poziom wody w zbiorniku w procentach
HASensor sensorVolume("water_volume"); // Objętość wody w litrach

// Sensory statusu
HASensor sensorPump("pump"); // Status pracy pompy (ON/OFF)
HASensor sensorWater("water"); // Status czujnika poziomu w akwarium (ON=niski/OFF=ok)

// Sensory alarmowe
HASensor sensorAlarm("water_alarm"); // Alarm braku wody w zbiorniku dolewki
HASensor sensorReserve("water_reserve"); // Alarm rezerwy w zbiorniku dolewki

// Przełączniki
HASwitch switchPump("pump_alarm"); // Przełącznik resetowania blokady pompy
HASwitch switchService("service_mode"); // Przełącznik trybu serwisowego
HASwitch switchSound("sound_switch"); // Przełącznik dźwięku alarmu            

// Status systemu
struct {
bool isPumpActive = false; // Status pracy pompy
    unsigned long pumpStartTime = 0; // Czas startu pompy
    bool isPumpDelayActive = false; // Status opóźnienia przed startem pompy
    unsigned long pumpDelayStartTime = 0; // Czas rozpoczęcia opóźnienia pompy
    bool pumpSafetyLock = false; // Blokada bezpieczeństwa pompy
    bool waterAlarmActive = false; // Alarm braku wody w zbiorniku dolewki
    bool waterReserveActive = false; // Status rezerwy wody w zbiorniku
    bool soundEnabled = false; // Status włączenia dźwięku alarmu
    bool isServiceMode = false; // Status trybu serwisowego
    unsigned long lastSuccessfulMeasurement = 0; // Czas ostatniego udanego pomiaru
    unsigned long lastSuccessfulMQTTPublish = 0; // Czas ostatniej udanej publikacji MQTT
} status;

// Struktura dla obsługi przycisku
struct ButtonState {
    bool currentState = HIGH; // Aktualny stan przycisku
    bool lastState = HIGH; // Poprzedni stan przycisku
    unsigned long pressedTime = 0; // Czas wciśnięcia przycisku
    unsigned long releasedTime = 0; // Czas puszczenia przycisku
    bool isLongPressHandled = false; // Flaga obsłużonego długiego naciśnięcia
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

// Konfiguracja i zarządzanie połączeniem WiFi
void setupWiFi() {
    // Zmienne statyczne zachowujące wartość między wywołaniami
    static unsigned long lastWiFiCheck = 0;  // Czas ostatniej próby połączenia
    static bool wifiInitiated = false;  // Flaga pierwszej inicjalizacji WiFi
    
    ESP.wdtFeed();  // Reset watchdoga
    
    // Pierwsza inicjalizacja WiFi
    if (!wifiInitiated) {
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);  // Rozpoczęcie połączenia z siecią
        wifiInitiated = true;  // Ustawienie flagi inicjalizacji
        return;
    }
    
    // Sprawdzenie stanu połączenia
    if (WiFi.status() != WL_CONNECTED) {  // Jeśli nie połączono z siecią
        if (millis() - lastWiFiCheck >= WIFI_CHECK_INTERVAL) {  // Sprawdź czy minął interwał
            Serial.print(".");  // Wskaźnik aktywności
            lastWiFiCheck = millis();  // Aktualizacja czasu ostatniej próby
            if (WiFi.status() == WL_DISCONNECTED) {  // Jeśli sieć jest dostępna ale rozłączona
                WiFi.reconnect();  // Próba ponownego połączenia
            }
        }
    }
}

/**
 * Funkcja nawiązująca połączenie z serwerem MQTT (Home Assistant)
 * 
 * @return bool - true jeśli połączenie zostało nawiązane, false w przypadku błędu
 */
bool connectMQTT() {
    Serial.println("\n--- Diagnostyka MQTT ---");
    Serial.printf("WiFi SSID: %s\n", WiFi.SSID().c_str());
    Serial.printf("WiFi siła: %d dBm\n", WiFi.RSSI());
    Serial.printf("IP ESP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("Brama: %s\n", WiFi.gatewayIP().toString().c_str());
    Serial.println("\nKonfiguracja MQTT:");
    Serial.printf("- Serwer: %s\n", MQTT_SERVER);
    Serial.printf("- Port: 1883\n");
    Serial.printf("- Użytkownik: %s\n", MQTT_USER);
    Serial.printf("- Hasło: %s\n", MQTT_PASSWORD);
    
    if (!mqtt.begin(MQTT_SERVER, 1883, MQTT_USER, MQTT_PASSWORD)) {
        Serial.println("\nBŁĄD POŁĄCZENIA MQTT!");
        return false;
    }
    
    Serial.println("MQTT połączono pomyślnie!");
    return true;
}

/**
 * Funkcja obliczająca poziom wody w zbiorniku dolewki w procentach
 * 
 * @param distance - zmierzona odległość od czujnika do lustra wody w mm
 * @return int - poziom wody w procentach (0-100%)
 * 
 * Wzór: ((EMPTY - distance) / (EMPTY - FULL)) * 100
 * - EMPTY (510mm) = 0% wody
 * - FULL (65mm) = 100% wody
 */
int calculateWaterLevel(int distance) {
    // Ograniczenie wartości do zakresu pomiarowego
    if (distance < DISTANCE_WHEN_FULL) distance = DISTANCE_WHEN_FULL;      // Nie mniej niż przy pełnym
    if (distance > DISTANCE_WHEN_EMPTY) distance = DISTANCE_WHEN_EMPTY;    // Nie więcej niż przy pustym
    
    // Obliczenie procentowe poziomu wody
    float percentage = (float)(DISTANCE_WHEN_EMPTY - distance) /          // Różnica: pusty - aktualny
                      (float)(DISTANCE_WHEN_EMPTY - DISTANCE_WHEN_FULL) * // Różnica: pusty - pełny
                      100.0;                                              // Przeliczenie na procenty
    
    return (int)percentage;    // Zwrot wartości całkowitej
}

/**
 * Funkcja obsługująca przełączanie trybu serwisowego z poziomu Home Assistant
 * 
 * @param state - nowy stan przełącznika (true = włączony, false = wyłączony)
 * @param sender - wskaźnik do obiektu przełącznika HA wywołującego funkcję
 * 
 * Działanie:
 * - Aktualizuje flagę trybu serwisowego
 * - Resetuje stan przycisku fizycznego
 * - Aktualizuje stan w Home Assistant
 * - Jeśli tryb serwisowy jest włączany podczas pracy pompy:
 *   - Wyłącza pompę
 *   - Resetuje liczniki czasu pracy
 *   - Aktualizuje status w HA
 */
void onServiceSwitchCommand(bool state, HASwitch* sender) {
    status.isServiceMode = state;                  // Ustawienie flagi trybu serwisowego
    buttonState.lastState = HIGH;                  // Reset stanu przycisku
    
    // Aktualizacja stanu w Home Assistant
    switchService.setState(state);                 // Synchronizacja stanu przełącznika
    
    // Jeśli włączono tryb serwisowy podczas pracy pompy
    if (status.isServiceMode && status.isPumpActive) {
        digitalWrite(PIN_PUMP, LOW);               // Wyłączenie pompy
        status.isPumpActive = false;               // Reset flagi aktywności
        status.pumpStartTime = 0;                  // Reset czasu startu
        sensorPump.setValue("OFF");                // Aktualizacja stanu w HA
    }
    
    // Log zmiany stanu
    Serial.printf("Tryb serwisowy: %s (przez HA)\n", 
                  state ? "WŁĄCZONY" : "WYŁĄCZONY");
}

/**
 * Funkcja obsługująca przełączanie dźwięku alarmu z poziomu Home Assistant
 * 
 * @param state - nowy stan przełącznika (true = włączony, false = wyłączony)
 * @param sender - wskaźnik do obiektu przełącznika HA wywołującego funkcję
 * 
 * Działanie:
 * - Aktualizuje flagę stanu dźwięku
 * - Zapisuje nowy stan do pamięci EEPROM (trwałej)
 * - Aktualizuje stan w Home Assistant
 */
void onSoundSwitchCommand(bool state, HASwitch* sender) {
    status.soundEnabled = state;                   // Aktualizacja flagi stanu dźwięku
    
    // Zapis stanu do pamięci EEPROM
    EEPROM.begin(EEPROM_SIZE);                    // Inicjalizacja EEPROM
    EEPROM.write(EEPROM_SOUND_STATE_ADDR,         // Zapis stanu (1 = włączony, 0 = wyłączony)
                 state ? 1 : 0);
    bool saved = EEPROM.commit();                 // Zatwierdzenie zapisu do EEPROM
    
    // Aktualizacja stanu w Home Assistant
    switchSound.setState(state);                   // Synchronizacja stanu przełącznika
    
    // Log zmiany stanu
    Serial.printf("Zmieniono stan dźwięku na: %s\n", 
                  state ? "WŁĄCZONY" : "WYŁĄCZONY");
}

/**
 * Funkcja obsługująca fizyczny przycisk na urządzeniu
 * 
 * Obsługuje dwa tryby naciśnięcia:
 * - Krótkie (< 1s): przełącza tryb serwisowy
 * - Długie (≥ 1s): kasuje blokadę bezpieczeństwa pompy
 */
void handleButton() {
    bool reading = digitalRead(PIN_BUTTON); // Odczyt stanu przycisku
    
    // Obsługa zmiany stanu przycisku
    if (reading != buttonState.lastState) {
        if (reading == LOW) {  // Przycisk naciśnięty
            buttonState.pressedTime = millis();  // Zapamiętaj czas naciśnięcia
            buttonState.isLongPressHandled = false;  // Reset flagi długiego naciśnięcia
        } else {  // Przycisk zwolniony
            buttonState.releasedTime = millis();  // Zapamiętaj czas zwolnienia
            
            // Sprawdzenie czy to było krótkie naciśnięcie
            if (buttonState.releasedTime - buttonState.pressedTime < LONG_PRESS_TIME) {
                // Przełącz tryb serwisowy
                status.isServiceMode = !status.isServiceMode;
                switchService.setState(status.isServiceMode);  // Aktualizacja w HA
                
                // Log zmiany stanu
                Serial.printf("Tryb serwisowy: %s (przez przycisk)\n", 
                            status.isServiceMode ? "WŁĄCZONY" : "WYŁĄCZONY");
                
                // Jeśli włączono tryb serwisowy podczas pracy pompy
                if (status.isServiceMode && status.isPumpActive) {
                    digitalWrite(PIN_PUMP, LOW);    // Wyłącz pompę
                    status.isPumpActive = false;    // Reset flagi aktywności
                    status.pumpStartTime = 0;       // Reset czasu startu
                    sensorPump.setValue("OFF");     // Aktualizacja w HA
                }
            }
        }
    }
    
    // Obsługa długiego naciśnięcia (reset blokady pompy)
    if (reading == LOW && !buttonState.isLongPressHandled) {
        if (millis() - buttonState.pressedTime >= LONG_PRESS_TIME) {
            status.pumpSafetyLock = false;         // Zdjęcie blokady pompy
            switchPump.setState(false);            // Aktualizacja stanu w HA
            buttonState.isLongPressHandled = true; // Oznacz jako obsłużone
            Serial.println("Alarm pompy skasowany");
        }
    }
    
    buttonState.lastState = reading;               // Zapamiętaj aktualny stan
    yield();                                       // Oddaj sterowanie systemowi
}

/**
 * Nieblokująca funkcja pomiaru odległości z czujnika ultradźwiękowego
 * 
 * Funkcja implementuje:
 * - Średnią z kilku pomiarów dla zwiększenia dokładności
 * - Nieblokującą obsługę czujnika HC-SR04
 * - Walidację wyników pomiaru
 * - Timeout dla zabezpieczenia przed zawieszeniem
 * 
 * @return int - zmierzona odległość w mm lub -1 jeśli pomiar w toku/błędny
 * 
 * Parametry pomiarowe:
 * - Prędkość dźwięku: 343 m/s
 * - Minimalny dystans: 20mm
 * - Maksymalny dystans: 4000mm
 * - Timeout: 23529µs (odpowiada max dystansowi ~4m)
 */
int measureDistanceNonBlocking() {
    static int measurements[MEASUREMENTS_COUNT];    
    static int measurementIndex = 0;               
    static unsigned long echoStartTime = 0;        

    ESP.wdtFeed();

    // Rozpoczęcie nowego pomiaru
    if (!ultrasonicInProgress) {
        if (millis() - lastUltrasonicTrigger >= ULTRASONIC_TIMEOUT) {
            digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
            delayMicroseconds(5);                  
            digitalWrite(PIN_ULTRASONIC_TRIG, HIGH);
            delayMicroseconds(10);                 
            digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
            
            echoStartTime = micros();             
            ultrasonicInProgress = true;           
            lastUltrasonicTrigger = millis();
            return -1;                            
        }
        return -1;
    }

    // Sprawdź stan ECHO
    if (ultrasonicInProgress) {
        int echoState = digitalRead(PIN_ULTRASONIC_ECHO);
        
        if (echoState == HIGH) {
            // Zmniejszamy timeout - dla 1500mm wystarczy około 9ms
            // (1500mm * 2 / 343m/s = ~8.75ms)
            if ((micros() - echoStartTime) > 10000) { // dajemy 10ms na bezpieczeństwo
                ultrasonicInProgress = false;
                Serial.println("Echo timeout!");
                return -1;
            }
            return -1; // Wciąż czekamy na echo
        }
        
        unsigned long duration = micros() - echoStartTime;
        ultrasonicInProgress = false;

        // Oblicz odległość w mm
        int distance = (duration * 343) / 2000;
        
        // Sprawdź zakres (20mm - 1500mm)
        if (distance >= 20 && distance <= 1500) {  // Zmieniamy górny limit na 1500mm
            measurements[measurementIndex] = distance;
            measurementIndex++;

            // Jeśli mamy komplet pomiarów
            if (measurementIndex >= MEASUREMENTS_COUNT) {
                // Sortuj pomiary (dla mediany)
                for (int i = 0; i < MEASUREMENTS_COUNT - 1; i++) {
                    for (int j = i + 1; j < MEASUREMENTS_COUNT; j++) {
                        if (measurements[i] > measurements[j]) {
                            int temp = measurements[i];
                            measurements[i] = measurements[j];
                            measurements[j] = temp;
                        }
                    }
                }

                // Weź medianę (środkowy pomiar)
                int medianDistance = measurements[MEASUREMENTS_COUNT / 2];
                measurementIndex = 0;
                return medianDistance;
            }
        }
    }
    return -1;
}

void onPumpAlarmCommand(bool state, HASwitch* sender) {
    if (!state) {
        status.pumpSafetyLock = false;
    }
}

// Kontrola pompy
void updatePump() {
    // Zabezpieczenie przed przepełnieniem millis()
    if (millis() < status.pumpStartTime) {
        status.pumpStartTime = millis();
    }
    
    if (millis() < status.pumpDelayStartTime) {
        status.pumpDelayStartTime = millis();
    }
    
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
        sensorAlarm.setValue("ON");               
        Serial.println("Alarm: Krytycznie niski poziom wody!");
    } else if (currentDistance < (DISTANCE_WHEN_EMPTY - HYSTERESIS) && status.waterAlarmActive) {
        status.waterAlarmActive = false;
        sensorAlarm.setValue("OFF");             
        Serial.println("Alarm wody wyłączony");
    }

    // Sprawdzenie stanu rezerwy z histerezą
    if (currentDistance >= DISTANCE_RESERVE && !status.waterReserveActive) {
        status.waterReserveActive = true;
        sensorReserve.setValue("ON");            
        Serial.println("Uwaga: Poziom rezerwy!");
    } else if (currentDistance < (DISTANCE_RESERVE - HYSTERESIS) && status.waterReserveActive) {
        status.waterReserveActive = false;
        sensorReserve.setValue("OFF");           
        Serial.println("Poziom powyżej rezerwy");
    }
}

void updateWaterLevel() {
    currentDistance = measureDistanceNonBlocking();
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
    
    // Debug info tylko gdy wartości się zmieniły (conajmniej 5mm)
    static float lastReportedDistance = 0;
    if (abs(currentDistance - lastReportedDistance) > 5) {
        Serial.printf("Poziom: %.1f mm, Obj: %.1f L\n", currentDistance, volume);
        lastReportedDistance = currentDistance;
    }
}

// Funkcja inicjalizacyjna - wykonywana jednorazowo przy starcie urządzenia
void setup() {
    ESP.wdtEnable(WATCHDOG_TIMEOUT);                // Aktywacja watchdoga
    Serial.begin(115200);                          // Inicjalizacja portu szeregowego
    Serial.println("\nStarting HydroSense...");    // Komunikat startowy
    
    // Inicjalizacja pamięci EEPROM
    EEPROM.begin(EEPROM_SIZE);                     // Start EEPROM z określonym rozmiarem
    delay(100);                                    // Opóźnienie stabilizacyjne
    
    // Odczyt zapisanego stanu dźwięku z EEPROM
    uint8_t savedState = EEPROM.read(EEPROM_SOUND_STATE_ADDR);
    status.soundEnabled = (savedState == 1);
    Serial.printf("Wczytano stan dźwięku z EEPROM (adres %d): %d -> %s\n", 
                 EEPROM_SOUND_STATE_ADDR, 
                 savedState,
                 status.soundEnabled ? "WŁĄCZONY" : "WYŁĄCZONY");
    
    // Konfiguracja kierunków pinów i stanów początkowych
    pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);          // Wyjście - trigger czujnika ultradźwiękowego
    pinMode(PIN_ULTRASONIC_ECHO, INPUT);           // Wejście - echo czujnika ultradźwiękowego
    digitalWrite(PIN_ULTRASONIC_TRIG, LOW);  // Upewnij się że TRIG jest LOW na starcie
    
    pinMode(PIN_WATER_LEVEL, INPUT_PULLUP);        // Wejście z podciąganiem - czujnik poziomu
    pinMode(PIN_BUTTON, INPUT_PULLUP);             // Wejście z podciąganiem - przycisk
    pinMode(PIN_BUZZER, OUTPUT);                   // Wyjście - buzzer
    digitalWrite(PIN_BUZZER, LOW);                 // Wyłączenie buzzera
    pinMode(PIN_PUMP, OUTPUT);                     // Wyjście - pompa
    digitalWrite(PIN_PUMP, LOW);                   // Wyłączenie pompy
    
    // Nawiązanie połączenia WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {        // Oczekiwanie na połączenie
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nPołączono z WiFi");

    // Próba połączenia MQTT
    Serial.println("Rozpoczynam połączenie MQTT...");
    if (connectMQTT()) {
        Serial.println("Połączono z MQTT pomyślnie!");
    } else {
        Serial.println("Nie udało się połączyć z MQTT!");
    }
    
    // Konfiguracja urządzenia dla Home Assistant
    device.setName("HydroSense");                  // Nazwa urządzenia
    device.setModel("HS ESP8266");                 // Model urządzenia
    device.setManufacturer("PMW");                 // Producent
    device.setSoftwareVersion("11.11.24");         // Wersja oprogramowania
       
    // Konfiguracja sensorów pomiarowych w HA
    sensorDistance.setName("Odległość od lustra wody");
    sensorDistance.setIcon("mdi:ruler");           // Ikona linijki
    sensorDistance.setUnitOfMeasurement("mm");     // Jednostka - milimetry
    
    sensorLevel.setName("Poziom wody");
    sensorLevel.setIcon("mdi:water-percent");      // Ikona poziomu wody
    sensorLevel.setUnitOfMeasurement("%");         // Jednostka - procenty
    
    sensorVolume.setName("Objętość wody");
    sensorVolume.setIcon("mdi:water");             // Ikona wody
    sensorVolume.setUnitOfMeasurement("L");        // Jednostka - litry
    
    // Konfiguracja sensorów statusu w HA
    sensorPump.setName("Status pompy");
    sensorPump.setIcon("mdi:water-pump");          // Ikona pompy
    
    sensorWater.setName("Czujnik wody");
    sensorWater.setIcon("mdi:water");              // Ikona wody
    
    // Konfiguracja sensorów alarmowych w HA
    sensorAlarm.setName("Brak wody");
    sensorAlarm.setIcon("mdi:water-alert");        // Ikona alarmu wody
    sensorAlarm.setValue("OFF");                   // Stan początkowy - wyłączony
    
    sensorReserve.setName("Rezerwa wody");
    sensorReserve.setIcon("mdi:alert-outline");    // Ikona ostrzeżenia
    sensorReserve.setValue("OFF");                 // Stan początkowy - wyłączony
    
    // Konfiguracja przełączników w HA
    switchService.setName("Serwis");
    switchService.setIcon("mdi:tools");            // Ikona narzędzi
    switchService.onCommand(onServiceSwitchCommand);// Funkcja obsługi zmiany stanu
    switchService.setState(status.isServiceMode);   // Stan początkowy
    
    switchSound.setName("Dźwięk");
    switchSound.setIcon("mdi:volume-high");        // Ikona głośnika
    switchSound.onCommand(onSoundSwitchCommand);   // Funkcja obsługi zmiany stanu
    switchSound.setState(status.soundEnabled);      // Stan początkowy
    
    switchPump.setName("Alarm pompy");
    switchPump.setIcon("mdi:alert");               // Ikona alarmu
    switchPump.onCommand(onPumpAlarmCommand);      // Funkcja obsługi zmiany stanu

    // Inicjalizacja połączenia MQTT
    mqtt.begin(MQTT_SERVER, 1883, MQTT_USER, MQTT_PASSWORD);
}

void loop() {
    unsigned long currentMillis = millis();
    static unsigned long lastMQTTRetry = 0;
    static unsigned long lastMeasurement = 0;
    static unsigned long serviceStartTime = 0;
    static unsigned long lastServicePrint = 0;  // do logowania trybu serwisowego
    static unsigned long lastDebugPrint = 0;  // Nowe - do debugowania

    // Feed watchdog
    ESP.wdtFeed();

    // Sprawdź WiFi i MQTT - bez zmian
    if (WiFi.status() != WL_CONNECTED) {
        setupWiFi();
        return;
    }
         
    if (!mqtt.isConnected()) {
        if (currentMillis - lastMQTTRetry >= 10000) {
            lastMQTTRetry = currentMillis;
            Serial.println("\nBrak połączenia MQTT - próba reconnect...");
            if (mqtt.begin(MQTT_SERVER, 1883, MQTT_USER, MQTT_PASSWORD)) {
                Serial.println("MQTT połączono ponownie!");
            }
        }
    }
    
    mqtt.loop();
    handleButton();
    
    // Obsługa trybu serwisowego
    if (status.isServiceMode) {
        if (serviceStartTime == 0) {
            serviceStartTime = currentMillis;
            Serial.println("Włączono tryb serwisowy - brak pomiarów przez 60 sekund");
        }
        
        // Wyświetl status co 10 sekund
        if (currentMillis - lastServicePrint >= 10000) {
            lastServicePrint = currentMillis;
            Serial.printf("Tryb serwisowy: pozostało %d sekund\n", 
                (60000 - (currentMillis - serviceStartTime)) / 1000);
        }
        
        if (currentMillis - serviceStartTime >= 60000) {
            status.isServiceMode = false;
            serviceStartTime = 0;
            switchService.setState(false);
            Serial.println("Zakończono tryb serwisowy");
            lastMeasurement = 0;  // Wymusi natychmiastowy pomiar po wyjściu z trybu serwisowego
        }
        return;
    }
       
    // Sprawdź czy nie ma zbyt długiej przerwy w pomiarach
    if (currentMillis - status.lastSuccessfulMeasurement >= 60000) {
        if (!status.isServiceMode) {  // Zabezpieczenie przed wielokrotnym logowaniem
            Serial.println("UWAGA: Brak poprawnych pomiarów przez 60 sekund!");
            status.isServiceMode = true;
            switchService.setState(true);
            serviceStartTime = 0;
        }
    }
    
    // Pomiar co MEASUREMENT_INTERVAL
    if (millis() - lastMeasurement >= MEASUREMENT_INTERVAL) {
        int distance = measureDistanceNonBlocking();
        if (distance > 0) {
            currentDistance = distance;
            
            // Obliczenie objętości
            float waterHeight = DISTANCE_WHEN_EMPTY - currentDistance;    
            waterHeight = constrain(waterHeight, 0, DISTANCE_WHEN_EMPTY - DISTANCE_WHEN_FULL);
            float radius = TANK_DIAMETER / 2.0;
            volume = PI * (radius * radius) * waterHeight / 1000000.0;
            
            // Aktualizacja sensorów HA
            sensorDistance.setValue(String((int)currentDistance).c_str());
            sensorLevel.setValue(String(calculateWaterLevel(currentDistance)).c_str());
            
            char valueStr[10];
            dtostrf(volume, 1, 1, valueStr);
            sensorVolume.setValue(valueStr);
            
            lastMeasurement = millis();
        }
    }

    updatePump();
    yield();
}
