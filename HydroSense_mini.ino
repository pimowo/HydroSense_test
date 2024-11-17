// --- Biblioteki

#include <Arduino.h>  // Podstawowa biblioteka Arduino zawierająca funkcje rdzenia
#include <ArduinoHA.h>  // Integracja z Home Assistant przez protokół MQTT
#include <ArduinoOTA.h>  // Aktualizacja oprogramowania przez sieć WiFi (Over-The-Air)
#include <ESP8266WiFi.h>  // Biblioteka WiFi dedykowana dla układu ESP8266
#include <EEPROM.h>  // Dostęp do pamięci nieulotnej EEPROM

// --- Definicje stałych i zmiennych globalnych

// Konfiguracja WiFi i MQTT
const char* WIFI_SSID = "pimowo";                  // Nazwa sieci WiFi
const char* WIFI_PASSWORD = "ckH59LRZQzCDQFiUgj";  // Hasło do sieci WiFi
const char* MQTT_SERVER = "192.168.1.14";          // Adres IP serwera MQTT (Home Assistant)
const char* MQTT_USER = "hydrosense";              // Użytkownik MQTT
const char* MQTT_PASSWORD = "hydrosense";          // Hasło MQTT

// Konfiguracja pinów ESP8266
#define PIN_ULTRASONIC_TRIG D6  // Pin TRIG czujnika ultradźwiękowego
#define PIN_ULTRASONIC_ECHO D7  // Pin ECHO czujnika ultradźwiękowego

#define PIN_WATER_LEVEL D5  // Pin czujnika poziomu wody w akwarium
#define POMPA_PIN D1       // Pin sterowania pompą
#define BUZZER_PIN D2           // Pin buzzera do alarmów dźwiękowych
#define PRZYCISK_PIN D3           // Pin przycisku do kasowania alarmów

// Stałe czasowe (wszystkie wartości w milisekundach)
const unsigned long ULTRASONIC_TIMEOUT = 50;       // Timeout pomiaru czujnika ultradźwiękowego
const unsigned long MEASUREMENT_INTERVAL = 10000;  // Interwał między pomiarami
const unsigned long WIFI_CHECK_INTERVAL = 5000;    // Interwał sprawdzania połączenia WiFi
const unsigned long WATCHDOG_TIMEOUT = 8000;       // Timeout dla watchdoga
const unsigned long PUMP_MAX_WORK_TIME = 300000;   // Maksymalny czas pracy pompy (5 minut)
const unsigned long PUMP_DELAY_TIME = 60000;       // Opóźnienie ponownego załączenia pompy (1 minuta)
const unsigned long SENSOR_READ_INTERVAL = 5000;   // Częstotliwość odczytu czujnika
const unsigned long MQTT_RETRY_INTERVAL = 5000;    // Interwał prób połączenia MQTT
const unsigned long WIFI_RETRY_INTERVAL = 10000;   // Interwał prób połączenia WiFi
const unsigned long BUTTON_DEBOUNCE_TIME = 50;     // Czas debouncingu przycisku
const unsigned long LONG_PRESS_TIME = 1000;        // Czas długiego naciśnięcia przycisku
const unsigned long SOUND_ALERT_INTERVAL = 60000;  // Interwał między sygnałami dźwiękowymi

// Konfiguracja EEPROM
#define EEPROM_SOUND_STATE_ADDR 0    // Adres przechowywania stanu dźwięku

// Definicja debugowania - ustaw 1 aby włączyć, 0 aby wyłączyć
#define DEBUG 1

#if DEBUG
    #define DEBUG_PRINT(x) Serial.println(x)
    #define DEBUG_PRINTF(format, ...) Serial.printf(format, __VA_ARGS__)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTF(format, ...)
#endif

// Stałe konfiguracyjne zbiornika
// Wszystkie odległości w milimetrach od czujnika do powierzchni wody
// Mniejsza odległość = wyższy poziom wody
const int TANK_FULL = 65;      // Odległość gdy zbiornik jest pełny (mm)
const int TANK_EMPTY = 510;    // Odległość gdy zbiornik jest pusty (mm)
const int RESERVE_LEVEL = 450;  // Poziom rezerwy wody (mm)
const int HYSTERESIS = 10; // Histereza przy zmianach poziomu (mm)
const int TANK_DIAMETER = 150;  // Średnica zbiornika (mm)
const int SENSOR_AVG_SAMPLES = 3;  // Liczba próbek do uśrednienia pomiaru
const int PUMP_DELAY = 5;      // Opóźnienie uruchomienia pompy (sekundy)
const int PUMP_WORK_TIME = 15;  // Czas pracy pompy

float aktualnaOdleglosc = 0;              // Aktualny dystans
unsigned long ostatniCzasDebounce = 0;     // Ostatni czas zmiany stanu przycisku

// Obiekty do komunikacji
WiFiClient client;              // Klient połączenia WiFi
HADevice device("HydroSense");  // Definicja urządzenia dla Home Assistant
HAMqtt mqtt(client, device);    // Klient MQTT dla Home Assistant

// Sensory pomiarowe
HASensor sensorDistance("water_level");                   // Odległość od lustra wody (w mm)
HASensor sensorLevel("water_level_percent");              // Poziom wody w zbiorniku (w procentach)
HASensor sensorVolume("water_volume");                    // Objętość wody (w litrach)

// Sensory statusu
HASensor sensorPump("pump");                              // Status pracy pompy (ON/OFF)
HASensor sensorWater("water");                            // Status czujnika poziomu w akwarium (ON=niski/OFF=ok)

// Sensory alarmowe
HASensor sensorAlarm("water_alarm");                      // Alarm braku wody w zbiorniku dolewki
HASensor sensorReserve("water_reserve");                  // Alarm rezerwy w zbiorniku dolewki

// Przełączniki
HASwitch switchPump("pump_alarm");                        // Przełącznik resetowania blokady pompy
HASwitch switchService("service_mode");                   // Przełącznik trybu serwisowego
HASwitch switchSound("sound_switch");                     // Przełącznik dźwięku alarmu

// --- Deklaracje funkcji i struktury

// Status systemu
struct SystemStatus {
    bool isPumpActive = false; // Status pracy pompy
    unsigned long pumpStartTime = 0; // Czas startu pompy
    float waterLevelBeforePump = 0;       // Poziom wody przed startem pompy
    bool isPumpDelayActive = false; // Status opóźnienia przed startem pompy
    unsigned long pumpDelayStartTime = 0; // Czas rozpoczęcia opóźnienia pompy
    bool pumpSafetyLock = false; // Blokada bezpieczeństwa pompy
    bool waterAlarmActive = false; // Alarm braku wody w zbiorniku dolewki
    bool waterReserveActive = false; // Status rezerwy wody w zbiorniku
    bool soundEnabled;// = false; // Status włączenia dźwięku alarmu
    bool isServiceMode = false; // Status trybu serwisowego
    unsigned long lastSuccessfulMeasurement = 0; // Czas ostatniego udanego pomiaru
    unsigned long lastSoundAlert = 0;  //
}; 
SystemStatus systemStatus;

// eeprom
struct Config {
    uint8_t version;  // Wersja konfiguracji
    bool soundEnabled;  // Status dźwięku (włączony/wyłączony)
    char checksum;  // Suma kontrolna
};
Config config;

// Struktura dla obsługi przycisku
struct ButtonState {
    bool lastState; // Poprzedni stan przycisku
    unsigned long pressedTime = 0; // Czas wciśnięcia przycisku
    unsigned long releasedTime = 0; // Czas puszczenia przycisku
    bool isLongPressHandled = false; // Flaga obsłużonego długiego naciśnięcia
    bool isInitialized = false; 
};
ButtonState buttonState;

// Struktura dla dźwięków alarmowych
struct AlarmTone {
    uint16_t frequency;        // Częstotliwość dźwięku
    uint16_t duration;         // Czas trwania
    uint8_t repeats;          // Liczba powtórzeń
    uint16_t pauseDuration;   // Przerwa między powtórzeniami
};

struct Status {
    bool soundEnabled;
    bool waterAlarmActive;
    bool waterReserveActive;
    bool isPumpActive;
    bool isPumpDelayActive;
    bool pumpSafetyLock;
    bool isServiceMode;
    unsigned long pumpStartTime;
    unsigned long pumpDelayStartTime;
    unsigned long lastSoundAlert;
};
Status status;

// Stałe konfiguracyjne
const uint8_t CONFIG_VERSION = 1;        // Wersja konfiguracji
const int EEPROM_SIZE = sizeof(Config);  // Rozmiar używanej pamięci EEPROM   

float currentDistance = 0;
float volume = 0;
unsigned long pumpStartTime = 0;
float waterLevelBeforePump = 0;

// --- EEPROM

// Ustawienia domyślne
void setDefaultConfig() {
    config.version = CONFIG_VERSION;
    config.soundEnabled = true;  // Domyślnie dźwięk włączony
    config.checksum = calculateChecksum(config);

    saveConfig();
    DEBUG_PRINT(F("Utworzono domyślną konfigurację"));
}

// Wczytywanie konfiguracji
bool loadConfig() {
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.get(0, config);
    
    if (config.version != CONFIG_VERSION) {
        DEBUG_PRINT(F("Niekompatybilna wersja konfiguracji"));
        setDefaultConfig();
        return false;
    }
    
    char calculatedChecksum = calculateChecksum(config);
    if (config.checksum != calculatedChecksum) {
        DEBUG_PRINT(F("Błąd sumy kontrolnej"));
        setDefaultConfig();
        return false;
    }
    
    // Synchronizuj stan po wczytaniu
    // Dzwięk
    status.soundEnabled = config.soundEnabled;
    switchSound.setState(config.soundEnabled, true);  // force update

    Serial.printf("Konfiguracja wczytana. Stan dźwięku: %s\n", 
                 config.soundEnabled ? "WŁĄCZONY" : "WYŁĄCZONY");
    return true;
}

// Zapis do EEPROM
void saveConfig() {
    static Config lastConfig;
    if (memcmp(&lastConfig, &config, sizeof(Config)) != 0) {
        EEPROM.put(0, config);
        EEPROM.commit();
        memcpy(&lastConfig, &config, sizeof(Config));
    }
}

// Obliczanie sumy kontrolnej
char calculateChecksum(const Config& cfg) {
    const byte* p = (const byte*)(&cfg);
    char sum = 0;
    for (int i = 0; i < sizeof(Config) - 1; i++) {
        sum ^= p[i];
    }
    return sum;
}

// --- Deklaracje funkcji alarmów

// Krótki sygnał dźwiękowy - pojedyncze piknięcie
void playShortWarningSound() {
    if (config.soundEnabled) {
        tone(BUZZER_PIN, 2000, 100); // Krótkie piknięcie (2000Hz, 100ms)
    }
}

// Sprawdzenie warunków alarmowych
void checkAlarmConditions() {
    unsigned long currentTime = millis();

    // Sprawdź czy minęła minuta od ostatniego alarmu
    if (currentTime - status.lastSoundAlert >= 60000) { // 60000ms = 1 minuta
        // Sprawdź czy dźwięk jest włączony i czy występuje alarm pompy lub tryb serwisowy
        if (config.soundEnabled && (status.pumpSafetyLock || status.isServiceMode)) {
            playShortWarningSound();
            status.lastSoundAlert = currentTime;
            
            // Debug info
            DEBUG_PRINT(F("Alarm dźwiękowy - przyczyna:"));
            if (status.pumpSafetyLock) DEBUG_PRINT(F("- Alarm pompy"));
            if (status.isServiceMode) DEBUG_PRINT(F("- Tryb serwisowy"));
        }
    }
}

void updateAlarmStates(float currentDistance) {
    // --- Obsługa alarmu krytycznie niskiego poziomu wody ---
    // Włącz alarm jeśli:
    // - odległość jest większa lub równa max (zbiornik pusty)
    // - alarm nie jest jeszcze aktywny
    if (currentDistance >= TANK_EMPTY && !status.waterAlarmActive) {
        status.waterAlarmActive = true;
        sensorAlarm.setValue("ON");               
        DEBUG_PRINT("Brak wody ON");
    } 
    // Wyłącz alarm jeśli:
    // - odległość spadła poniżej progu wyłączenia (z histerezą)
    // - alarm jest aktywny
    else if (currentDistance < (TANK_EMPTY - HYSTERESIS) && status.waterAlarmActive) {
        status.waterAlarmActive = false;
        sensorAlarm.setValue("OFF");
        DEBUG_PRINT("Brak wody OFF");
    }

    // --- Obsługa ostrzeżenia o rezerwie wody ---
    // Włącz ostrzeżenie o rezerwie jeśli:
    // - odległość osiągnęła próg rezerwy
    // - ostrzeżenie nie jest jeszcze aktywne
    if (currentDistance >= RESERVE_LEVEL && !status.waterReserveActive) {
        status.waterReserveActive = true;
        sensorReserve.setValue("ON");
        DEBUG_PRINT("Rezerwa ON");
    } 
    // Wyłącz ostrzeżenie o rezerwie jeśli:
    // - odległość spadła poniżej progu rezerwy (z histerezą)
    // - ostrzeżenie jest aktywne
    else if (currentDistance < (RESERVE_LEVEL - HYSTERESIS) && status.waterReserveActive) {
        status.waterReserveActive = false;
        sensorReserve.setValue("OFF");
        DEBUG_PRINT("Rezerwa OFF");
    }
}

// --- Deklaracje funkcji pompy

// Kontrola pompy - funkcja zarządzająca pracą pompy i jej zabezpieczeniami
void updatePump() {
    // Odczyt stanu czujnika poziomu wody
    // LOW = brak wody - należy uzupełnić
    // HIGH = woda obecna - stan normalny
    bool waterPresent = (digitalRead(PIN_WATER_LEVEL) == LOW);
    sensorWater.setValue(waterPresent ? "ON" : "OFF");

    // Zabezpieczenie przed przepełnieniem licznika millis()
    unsigned long currentMillis = millis(); // Dodane: zapisanie aktualnego czasu

    if (currentMillis < status.pumpStartTime) {
        status.pumpStartTime = currentMillis;
    }
    
    if (currentMillis < status.pumpDelayStartTime) {
        status.pumpDelayStartTime = currentMillis;
    }
      
    // --- ZABEZPIECZENIE 1: Tryb serwisowy ---
    if (status.isServiceMode) {
        if (status.isPumpActive) {
            stopPump();
        }
        return;
    }

    // --- ZABEZPIECZENIE 2: Maksymalny czas pracy ---
    if (status.isPumpActive && (currentMillis - status.pumpStartTime > PUMP_WORK_TIME * 1000)) {
        stopPump();
        status.pumpSafetyLock = true;
        switchPump.setState(true);
        DEBUG_PRINT(F("ALARM: Pompa pracowała za długo - aktywowano blokadę bezpieczeństwa!"));
        return;
    }
    
    // --- ZABEZPIECZENIE 3: Blokady bezpieczeństwa ---
    if (status.pumpSafetyLock || status.waterAlarmActive) {
        if (status.isPumpActive) {
            stopPump();
        }
        return;
    }
    
    // --- ZABEZPIECZENIE 4: Ochrona przed przepełnieniem ---
    if (!waterPresent && status.isPumpActive) {
        stopPump();
        status.isPumpDelayActive = false;
        return;
    }
    
    // --- LOGIKA WŁĄCZANIA POMPY ---
    if (waterPresent && !status.isPumpActive && !status.isPumpDelayActive) {
        status.isPumpDelayActive = true;
        status.pumpDelayStartTime = currentMillis;
        return;
    }
    
    // Po upływie opóźnienia, włącz pompę
    if (status.isPumpDelayActive && !status.isPumpActive) {
        if (currentMillis - status.pumpDelayStartTime >= (PUMP_DELAY * 1000)) {
            startPump();
        }
    }
}

// Ztrzymanie pompy
void stopPump() {
    digitalWrite(POMPA_PIN, LOW);
    status.isPumpActive = false;
    status.pumpStartTime = 0;
    sensorPump.setValue("OFF");
    DEBUG_PRINT(F("Pompa zatrzymana"));
}

// Uruchomienienie pompy
void startPump() {
    digitalWrite(POMPA_PIN, HIGH);
    status.isPumpActive = true;
    status.pumpStartTime = millis();
    status.isPumpDelayActive = false;
    sensorPump.setValue("ON");
    DEBUG_PRINT(F("Pompa uruchomiona"));
}

// Funkcja obsługuje zdarzenie resetu alarmu pompy
// Jest wywoływana gdy użytkownik zmieni stan przełącznika alarmu w interfejsie HA
// 
// Parametry:
// - state: stan przełącznika (true = alarm aktywny, false = reset alarmu)
// - sender: wskaźnik do obiektu przełącznika w HA (niewykorzystywany)
void onPumpAlarmCommand(bool state, HASwitch* sender) {
    // Reset blokady bezpieczeństwa pompy następuje tylko gdy przełącznik 
    // zostanie ustawiony na false (wyłączony)
    if (!state) {
        status.pumpSafetyLock = false;  // Wyłącz blokadę bezpieczeństwa pompy
    }
}

// --- Deklaracje funkcji związanych z siecią

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
    if (!mqtt.begin(MQTT_SERVER, 1883, MQTT_USER, MQTT_PASSWORD)) {
        DEBUG_PRINT("\nBŁĄD POŁĄCZENIA MQTT!");
        return false;
    }
    
    DEBUG_PRINT("MQTT połączono pomyślnie!");
    return true;
}

// Funkcja konfigurująca Home Assistant
void setupHA() {
    // Konfiguracja urządzenia dla Home Assistant
    device.setName("HydroSense");                  // Nazwa urządzenia
    device.setModel("HS ESP8266");                 // Model urządzenia
    device.setManufacturer("PMW");                 // Producent
    device.setSoftwareVersion("17.11.24");         // Wersja oprogramowania

    // Konfiguracja sensorów pomiarowych w HA
    sensorDistance.setName("Pomiar odległości");
    sensorDistance.setIcon("mdi:ruler");           // Ikona linijki
    sensorDistance.setUnitOfMeasurement("mm");     // Jednostka - milimetry
    
    sensorLevel.setName("Poziom wody");
    sensorLevel.setIcon("mdi:cup-water");      // Ikona poziomu wody
    sensorLevel.setUnitOfMeasurement("%");         // Jednostka - procenty
    
    sensorVolume.setName("Objętość wody");
    sensorVolume.setIcon("mdi:cup-water");             // Ikona wody
    sensorVolume.setUnitOfMeasurement("L");        // Jednostka - litry
    
    // Konfiguracja sensorów statusu w HA
    sensorPump.setName("Status pompy");
    sensorPump.setIcon("mdi:water-pump");          // Ikona pompy
    
    sensorWater.setName("Czujnik wody");
    sensorWater.setIcon("mdi:electric-switch");              // Ikona wody
    
    // Konfiguracja sensorów alarmowych w HA
    sensorAlarm.setName("Brak wody");
    sensorAlarm.setIcon("mdi:alarm-light");        // Ikona alarmu wody

    sensorReserve.setName("Rezerwa wody");
    sensorReserve.setIcon("mdi:alarm-light-outline");    // Ikona ostrzeżenia
    
    // Konfiguracja przełączników w HA
    switchService.setName("Serwis");
    switchService.setIcon("mdi:tools");            // Ikona narzędzi
    switchService.onCommand(onServiceSwitchCommand);// Funkcja obsługi zmiany stanu
    switchService.setState(status.isServiceMode);   // Stan początkowy
    // Inicjalizacja stanu - domyślnie wyłączony
    status.isServiceMode = false;
    switchService.setState(false, true); // force update przy starcie

    switchSound.setName("Dźwięk");
    switchSound.setIcon("mdi:volume-high");        // Ikona głośnika
    switchSound.onCommand(onSoundSwitchCommand);   // Funkcja obsługi zmiany stanu
    switchSound.setState(status.soundEnabled);      // Stan początkowy
    
    switchPump.setName("Alarm pompy");
    switchPump.setIcon("mdi:alert");               // Ikona alarmu
    switchPump.onCommand(onPumpAlarmCommand);      // Funkcja obsługi zmiany stanu
}

// --- Deklaracje funkcji ogólnych

// Konfiguracja kierunków pinów i stanów początkowych
void setupPin() {
    pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);          // Wyjście - trigger czujnika ultradźwiękowego
    pinMode(PIN_ULTRASONIC_ECHO, INPUT);           // Wejście - echo czujnika ultradźwiękowego
    digitalWrite(PIN_ULTRASONIC_TRIG, LOW);  // Upewnij się że TRIG jest LOW na starcie
    
    pinMode(PIN_WATER_LEVEL, INPUT_PULLUP);        // Wejście z podciąganiem - czujnik poziomu
    pinMode(PRZYCISK_PIN, INPUT_PULLUP);             // Wejście z podciąganiem - przycisk
    pinMode(BUZZER_PIN, OUTPUT);                   // Wyjście - buzzer
    digitalWrite(BUZZER_PIN, LOW);                 // Wyłączenie buzzera
    pinMode(POMPA_PIN, OUTPUT);                     // Wyjście - pompa
    digitalWrite(POMPA_PIN, LOW);                   // Wyłączenie pompy
}

// Melodia powitalna
void welcomeMelody() {
    tone(BUZZER_PIN, 1397, 100);  // F6
    delay(150);
    tone(BUZZER_PIN, 1568, 100);  // G6
    delay(150);
    tone(BUZZER_PIN, 1760, 150);  // A6
    delay(200);
}

// Wykonaj pierwszy pomiar i ustaw stany
void firstUpdateHA() {
    float initialDistance = measureDistance();
    
    // Ustaw początkowe stany na podstawie pomiaru
    status.waterAlarmActive = (initialDistance >= TANK_EMPTY);
    status.waterReserveActive = (initialDistance >= RESERVE_LEVEL);
    
    // Wymuś stan OFF na początku
    sensorAlarm.setValue("OFF");
    sensorReserve.setValue("OFF");
    switchSound.setState(false);  // Dodane - wymuś stan początkowy
    mqtt.loop();
    
    // Ustawienie końcowych stanów i wysyłka do HA
    sensorAlarm.setValue(status.waterAlarmActive ? "ON" : "OFF");
    sensorReserve.setValue(status.waterReserveActive ? "ON" : "OFF");
    switchSound.setState(status.soundEnabled);  // Dodane - ustaw aktualny stan dźwięku
    mqtt.loop();
}

/**
 * Funkcja obsługująca przełączanie trybu serwisowego z poziomu Home Assistant
 * 
 * @param state - nowy stan przełącznika (true = włączony, false = wyłączony)
 * @param sender - wskaźnik do obiektu przełącznika HA wywołującego funkcję
 * 
 * Działanie:
 * Przy włączaniu (state = true):
 * - Aktualizuje flagę trybu serwisowego
 * - Resetuje stan przycisku fizycznego
 * - Aktualizuje stan w Home Assistant
 * - Jeśli pompa pracowała:
 *   - Wyłącza pompę
 *   - Resetuje liczniki czasu pracy
 *   - Aktualizuje status w HA
 * 
 * Przy wyłączaniu (state = false):
 * - Wyłącza tryb serwisowy
 * - Aktualizuje stan w Home Assistant
 * - Umożliwia normalną pracę pompy według czujnika poziomu
 * - Resetuje stan opóźnienia pompy
 */
void onServiceSwitchCommand(bool state, HASwitch* sender) {
    status.isServiceMode = state;  // Ustawienie flagi trybu serwisowego
    buttonState.lastState = HIGH;  // Reset stanu przycisku
    
    // Aktualizacja stanu w Home Assistant
    switchService.setState(state);  // Synchronizacja stanu przełącznika
    
    if (state) {  // Włączanie trybu serwisowego
        if (status.isPumpActive) {
            digitalWrite(POMPA_PIN, LOW);  // Wyłączenie pompy
            status.isPumpActive = false;  // Reset flagi aktywności
            status.pumpStartTime = 0;  // Reset czasu startu
            sensorPump.setValue("OFF");  // Aktualizacja stanu w HA
        }
    } else {  // Wyłączanie trybu serwisowego
        // Reset stanu opóźnienia pompy aby umożliwić normalne uruchomienie
        status.isPumpDelayActive = false;
        status.pumpDelayStartTime = 0;
        // Normalny tryb pracy - pompa uruchomi się automatycznie 
        // jeśli czujnik poziomu wykryje wodę
    }
    
    Serial.printf("Tryb serwisowy: %s (przez HA)\n", 
                  state ? "WŁĄCZONY" : "WYŁĄCZONY");
}

// Pobranie aktualnego poziomu wody
/**
 * @brief Wykonuje pomiar odległości za pomocą czujnika ultradźwiękowego i oblicza aktualny poziom wody.
 * 
 * Funkcja najpierw wykonuje pomiar odległości za pomocą czujnika ultradźwiękowego HC-SR04,
 * zapisuje wynik pomiaru do zmiennej, a następnie oblicza poziom wody na podstawie tej wartości.
 * 
 * @return Poziom wody w jednostkach używanych przez funkcję calculateWaterLevel.
 *         Zwraca wartość obliczoną przez funkcję calculateWaterLevel, która przelicza odległość na poziom wody.
 */
float getCurrentWaterLevel() {
    int distance = measureDistance();  // Pobranie wyniku pomiaru
    float waterLevel = calculateWaterLevel(distance);  // Obliczenie poziomu wody na podstawie wyniku pomiaru
    return waterLevel;  // Zwrócenie obliczonego poziomu wody
}

// Pomiar odległości
// Funkcja wykonuje pomiar odległości za pomocą czujnika ultradźwiękowego HC-SR04
// Zwraca:
//   - zmierzoną odległość w milimetrach (mediana z kilku pomiarów)
//   - (-1) w przypadku błędu lub przekroczenia czasu odpowiedzi
int measureDistance() {
    int measurements[SENSOR_AVG_SAMPLES];  // Tablica do przechowywania pomiarów

    for (int i = 0; i < SENSOR_AVG_SAMPLES; i++) {
        // Generowanie impulsu wyzwalającego (trigger)
        digitalWrite(PIN_ULTRASONIC_TRIG, LOW);  // Upewnij się że pin jest w stanie niskim
        delayMicroseconds(5);  // Krótka pauza dla stabilizacji
        digitalWrite(PIN_ULTRASONIC_TRIG, HIGH);  // Wysłanie impulsu 10µs
        delayMicroseconds(10);  // Czekaj 10µs
        digitalWrite(PIN_ULTRASONIC_TRIG, LOW);  // Zakończ impuls

        // Oczekiwanie na początek sygnału echo
        // Timeout 100ms chroni przed zawieszeniem jeśli czujnik nie odpowiada
        unsigned long startWaiting = millis();
        while (digitalRead(PIN_ULTRASONIC_ECHO) == LOW) {
            if (millis() - startWaiting > 100) {
                DEBUG_PRINT("Timeout - brak początku echa");
                return -1;  // Błąd - brak odpowiedzi od czujnika
            }
        }

        // Pomiar czasu początku echa (w mikrosekundach)
        unsigned long echoStartTime = micros();

        // Oczekiwanie na koniec sygnału echo
        // Timeout 20ms - teoretyczny max dla 3.4m to około 20ms
        while (digitalRead(PIN_ULTRASONIC_ECHO) == HIGH) {
            if (micros() - echoStartTime > 20000) {
                DEBUG_PRINT("Timeout - zbyt długie echo");
                return -1;  // Błąd - echo trwa zbyt długo
            }
        }

        // Obliczenie czasu trwania echa (w mikrosekundach)
        unsigned long duration = micros() - echoStartTime;

        // Konwersja czasu na odległość
        int distance = (duration * 343) / 2000;

        // Walidacja wyniku
        if (distance >= 20 && distance <= 1500) {
            measurements[i] = distance;  // Zapis poprawnego pomiaru
        } else {
            measurements[i] = -1;  // Zapis błędnego pomiaru
        }

        delay(50);  // Krótka przerwa między pomiarami
    }

    // Sortowanie pomiarów rosnąco
    for (int i = 0; i < SENSOR_AVG_SAMPLES - 1; i++) {
        for (int j = 0; j < SENSOR_AVG_SAMPLES - i - 1; j++) {
            if (measurements[j] > measurements[j + 1]) {
                int temp = measurements[j];
                measurements[j] = measurements[j + 1];
                measurements[j + 1] = temp;
            }
        }
    }

    // Obliczenie mediany z pomiarów
    if (SENSOR_AVG_SAMPLES % 2 == 0) {
        // Jeśli liczba pomiarów jest parzysta, zwróć średnią z dwóch środkowych wartości
        int midIndex = SENSOR_AVG_SAMPLES / 2;
        if (measurements[midIndex - 1] == -1 || measurements[midIndex] == -1) {
            return -1;  // Jeśli środkowe wartości są błędne, zwróć błąd
        }
        return (measurements[midIndex - 1] + measurements[midIndex]) / 2;
    } else {
        // Jeśli liczba pomiarów jest nieparzysta, zwróć środkową wartość
        int midIndex = SENSOR_AVG_SAMPLES / 2;
        if (measurements[midIndex] == -1) {
            return -1;  // Jeśli środkowa wartość jest błędna, zwróć błąd
        }
        return measurements[midIndex];
    }
}

// Funkcja obliczająca poziom wody w zbiorniku dolewki w procentach
//  
// @param distance - zmierzona odległość od czujnika do lustra wody w mm
// @return int - poziom wody w procentach (0-100%)
// Wzór: ((EMPTY - distance) / (EMPTY - FULL)) * 100
int calculateWaterLevel(int distance) {
    // Ograniczenie wartości do zakresu pomiarowego
    if (distance < TANK_FULL) distance = TANK_FULL;  // Nie mniej niż przy pełnym
    if (distance > TANK_EMPTY) distance = TANK_EMPTY;  // Nie więcej niż przy pustym
    
    // Obliczenie procentowe poziomu wody
    float percentage = (float)(TANK_EMPTY - distance) /  // Różnica: pusty - aktualny
                      (float)(TANK_EMPTY - TANK_FULL) *  // Różnica: pusty - pełny
                      100.0;  // Przeliczenie na procenty
    
    return (int)percentage;  // Zwrot wartości całkowitej
}

//
void updateWaterLevel() {
    // Zapisz poprzednią objętość
    float previousVolume = volume;

    currentDistance = measureDistance();
    if (currentDistance < 0) return; // błąd pomiaru
    
    // Aktualizacja stanów alarmowych
    updateAlarmStates(currentDistance);

    // Obliczenie objętości
    float waterHeight = TANK_EMPTY - currentDistance;    
    waterHeight = constrain(waterHeight, 0, TANK_EMPTY - TANK_FULL);
    
    // Obliczenie objętości w litrach (wszystko w mm)
    float radius = TANK_DIAMETER / 2.0;
    volume = PI * (radius * radius) * waterHeight / 1000000.0; // mm³ na litry
    
    // Aktualizacja sensorów pomiarowych
    sensorDistance.setValue(String((int)currentDistance).c_str());
    sensorLevel.setValue(String(calculateWaterLevel(currentDistance)).c_str());
        
    char valueStr[10];
    dtostrf(volume, 1, 1, valueStr);
    sensorVolume.setValue(valueStr);
        
    // Debug info tylko gdy wartości się zmieniły (conajmniej 5mm)
    static float lastReportedDistance = 0;
    if (abs(currentDistance - lastReportedDistance) > 5) {
        Serial.printf("Poziom: %.1f mm, Obj: %.1f L\n", currentDistance, volume);
        lastReportedDistance = currentDistance;
    }
}

// Obsługa przycisku
/**
 * Funkcja obsługująca fizyczny przycisk na urządzeniu
 * 
 * Obsługuje dwa tryby naciśnięcia:
 * - Krótkie (< 1s): przełącza tryb serwisowy
 * - Długie (≥ 1s): kasuje blokadę bezpieczeństwa pompy
 */
// void handleButton() {
void handleButton() {
    static unsigned long lastDebounceTime = 0;
    static bool lastReading = HIGH;
    const unsigned long DEBOUNCE_DELAY = 50;  // 50ms debounce

    bool reading = digitalRead(PRZYCISK_PIN);

    // Jeśli odczyt się zmienił, zresetuj timer debounce
    if (reading != lastReading) {
        lastDebounceTime = millis();
    }
    
    // Kontynuuj tylko jeśli minął czas debounce
    if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
        // Jeśli stan się faktycznie zmienił po debounce
        if (reading != buttonState.lastState) {
            buttonState.lastState = reading;
            
            if (reading == LOW) {  // Przycisk naciśnięty
                buttonState.pressedTime = millis();
                buttonState.isLongPressHandled = false;  // Reset flagi długiego naciśnięcia
            } else {  // Przycisk zwolniony
                buttonState.releasedTime = millis();
                
                // Sprawdzenie czy to było krótkie naciśnięcie
                if (buttonState.releasedTime - buttonState.pressedTime < LONG_PRESS_TIME) {
                    // Przełącz tryb serwisowy
                    status.isServiceMode = !status.isServiceMode;
                    switchService.setState(status.isServiceMode, true);  // force update w HA
                    
                    // Log zmiany stanu
                    Serial.printf("Tryb serwisowy: %s (przez przycisk)\n", 
                                status.isServiceMode ? "WŁĄCZONY" : "WYŁĄCZONY");
                    
                    // Jeśli włączono tryb serwisowy podczas pracy pompy
                    if (status.isServiceMode && status.isPumpActive) {
                        digitalWrite(POMPA_PIN, LOW);  // Wyłącz pompę
                        status.isPumpActive = false;  // Reset flagi aktywności
                        status.pumpStartTime = 0;  // Reset czasu startu
                        sensorPump.setValue("OFF");  // Aktualizacja w HA
                    }
                }
            }
        }
        
        // Obsługa długiego naciśnięcia (reset blokady pompy)
        if (reading == LOW && !buttonState.isLongPressHandled) {
            if (millis() - buttonState.pressedTime >= LONG_PRESS_TIME) {
                ESP.wdtFeed();  // Reset przy długim naciśnięciu
                status.pumpSafetyLock = false;  // Zdjęcie blokady pompy
                switchPump.setState(false, true);  // force update w HA
                buttonState.isLongPressHandled = true;  // Oznacz jako obsłużone
                DEBUG_PRINT("Alarm pompy skasowany");
            }
        }
    }
    
    lastReading = reading;  // Zapisz ostatni odczyt dla następnego porównania
    yield();  // Oddaj sterowanie systemowi
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
    status.soundEnabled = state;  // Aktualizuj status lokalny
    config.soundEnabled = state;  // Aktualizuj konfigurację
    saveConfig();  // Zapisz do EEPROM
    
    // Aktualizuj stan w Home Assistant
    switchSound.setState(state, true);  // force update
    
    Serial.printf("Zmieniono stan dźwięku na: %s\n", state ? "WŁĄCZONY" : "WYŁĄCZONY");
}

// --- Setup
void setup() {
    ESP.wdtEnable(WATCHDOG_TIMEOUT);  // Aktywacja watchdoga
    Serial.begin(115200);  // Inicjalizacja portu szeregowego
    DEBUG_PRINT("\nStarting HydroSense...");  // Komunikat startowy
    
    // Wczytaj konfigurację
    if (!loadConfig()) {
        DEBUG_PRINT(F("Tworzenie nowej konfiguracji..."));
        setDefaultConfig();
    }
    
    // Zastosuj wczytane ustawienia
    //status.soundEnabled = config.soundEnabled;
    
    setupPin();
    
    // Nawiązanie połączenia WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Łączenie z WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        ESP.wdtFeed(); // Reset watchdoga podczas łączenia
    }
    DEBUG_PRINT("\nPołączono z WiFi");

    // Próba połączenia MQTT
    DEBUG_PRINT("Rozpoczynam połączenie MQTT...");
    connectMQTT();

    setupHA();
   
    // Wczytaj konfigurację z EEPROM
    if (loadConfig()) {        
        status.soundEnabled = config.soundEnabled;  // Synchronizuj stan dźwięku z wczytanej konfiguracji
    }
    
    firstUpdateHA();
    status.lastSoundAlert = millis();
    
    // Konfiguracja OTA (Over-The-Air) dla aktualizacji oprogramowania
    ArduinoOTA.setHostname("HydroSense");  // Ustaw nazwę urządzenia
    ArduinoOTA.setPassword("hydrosense");  // Ustaw hasło dla OTA
    ArduinoOTA.begin();  // Uruchom OTA    
    
    DEBUG_PRINT("Setup zakończony pomyślnie!");
    
    if (status.soundEnabled) {
        welcomeMelody();
    }  
}

void loop() {
    unsigned long currentMillis = millis();
    static unsigned long lastMQTTRetry = 0;
    static unsigned long lastMeasurement = 0;
    static unsigned long lastStatsUpdate = 0;

    // KRYTYCZNE OPERACJE SYSTEMOWE
    
    // Zabezpieczenie przed zawieszeniem systemu
    ESP.wdtFeed();  // Resetowanie licznika watchdog
    yield();  // Obsługa krytycznych zadań systemowych ESP8266
    
    // System aktualizacji bezprzewodowej
    ArduinoOTA.handle();  // Nasłuchiwanie żądań aktualizacji OTA

    // ZARZĄDZANIE ŁĄCZNOŚCIĄ
    
    // Sprawdzanie i utrzymanie połączenia WiFi
    if (WiFi.status() != WL_CONNECTED) {
        setupWiFi();    // Próba ponownego połączenia z siecią
        return;         // Powrót do początku pętli po próbie połączenia
    }
    
    // Zarządzanie połączeniem MQTT
    if (!mqtt.isConnected()) {
        if (currentMillis - lastMQTTRetry >= 10000) { // Ponowna próba co 10 sekund
            lastMQTTRetry = currentMillis;
            DEBUG_PRINT("\nBrak połączenia MQTT - próba reconnect...");
            if (mqtt.begin(MQTT_SERVER, 1883, MQTT_USER, MQTT_PASSWORD)) {
                DEBUG_PRINT("MQTT połączono ponownie!");
            }
        }
    }
        
    mqtt.loop();  // Obsługa komunikacji MQTT

    // GŁÓWNE FUNKCJE URZĄDZENIA
    
    // Obsługa interfejsu i sterowania
    handleButton();     // Przetwarzanie sygnałów z przycisków
    updatePump();       // Sterowanie pompą
    checkAlarmConditions(); // System ostrzeżeń dźwiękowych
    
    // Monitoring poziomu wody
    if (currentMillis - lastMeasurement >= MEASUREMENT_INTERVAL) {
        updateWaterLevel();  // Pomiar i aktualizacja stanu wody
        lastMeasurement = currentMillis;
    }
}
