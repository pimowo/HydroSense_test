// --- Biblioteki

#include <Arduino.h>  // Podstawowa biblioteka Arduino zawierająca funkcje rdzenia
#include <ArduinoHA.h>  // Biblioteka do integracji z Home Assistant przez protokół MQTT
#include <ArduinoOTA.h>  // Biblioteka do aktualizacji oprogramowania przez sieć WiFi
#include <ESP8266WiFi.h>  // Biblioteka WiFi dedykowana dla układu ESP8266
#include <EEPROM.h>  // Biblioteka do dostępu do pamięci nieulotnej EEPROM
#include <WiFiManager.h>  // Biblioteka do zarządzania połączeniami WiFi
#include <ESP8266WebServer.h>

// --- Definicje stałych i zmiennych globalnych

ESP8266WebServer server(80);

// Wersja systemu
const char* SOFTWARE_VERSION = "20.11.24";

// Konfiguracja MQTT
const char* MQTT_SERVER = "192.168.1.14";  // Adres IP serwera MQTT (Home Assistant)
const char* MQTT_USER = "hydrosense";  // Użytkownik MQTT
const char* MQTT_PASSWORD = "hydrosense";  // Hasło MQTT

// Konfiguracja pinów ESP8266
const int PIN_ULTRASONIC_TRIG = D6;  // Pin TRIG czujnika ultradźwiękowego
const int PIN_ULTRASONIC_ECHO = D7;  // Pin ECHO czujnika ultradźwiękowego

const int PIN_WATER_LEVEL = D5;  // Pin czujnika poziomu wody w akwarium
const int POMPA_PIN = D1;  // Pin sterowania pompą
const int BUZZER_PIN = D2;  // Pin buzzera do alarmów dźwiękowych
const int PRZYCISK_PIN = D3;  // Pin przycisku do kasowania alarmów

// Stałe czasowe (wszystkie wartości w milisekundach)
const unsigned long ULTRASONIC_TIMEOUT = 50;  // Timeout pomiaru czujnika ultradźwiękowego
const unsigned long MEASUREMENT_INTERVAL = 60000;  // Interwał między pomiarami
const unsigned long WIFI_CHECK_INTERVAL = 5000;  // Interwał sprawdzania połączenia WiFi
const unsigned long WATCHDOG_TIMEOUT = 8000;  // Timeout dla watchdoga
const unsigned long SENSOR_READ_INTERVAL = 5000;  // Częstotliwość odczytu czujnika
const unsigned long WIFI_RETRY_INTERVAL = 10000;  // Interwał prób połączenia WiFi
const unsigned long BUTTON_DEBOUNCE_TIME = 50;  // Czas debouncingu przycisku
const unsigned long LONG_PRESS_TIME = 1000;  // Czas długiego naciśnięcia przycisku
const unsigned long SOUND_ALERT_INTERVAL = 60000;  // Interwał między sygnałami dźwiękowymi
const unsigned long MQTT_LOOP_INTERVAL = 100;  // Obsługa MQTT co 100ms
const unsigned long OTA_CHECK_INTERVAL = 1000;  // Sprawdzanie OTA co 1s
const unsigned long MQTT_RETRY_INTERVAL = 10000;  // Próba połączenia MQTT co 10s

// Konfiguracja EEPROM
#define EEPROM_SOUND_STATE_ADDR 0    // Adres przechowywania stanu dźwięku

// Definicja debugowania
#define DEBUG 0  // 0 wyłącza debug, 1 włącza debug

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
//const int TANK_FULL = 65;  // Odległość gdy zbiornik jest pełny (mm)
//const int TANK_EMPTY = 510;  // Odległość gdy zbiornik jest pusty (mm)
//const int RESERVE_LEVEL = 450;  // Poziom rezerwy wody (mm)
const int HYSTERESIS = 10;  // Histereza przy zmianach poziomu (mm)
//const int TANK_DIAMETER = 150;  // Średnica zbiornika (mm)
//const int VALID_MARGIN = 25;  // Margines błędu dla poprawnych pomiarów (mm)
// Stałe dla fizycznych limitów czujnika
const int SENSOR_MIN_RANGE = 20;   // Minimalny zakres czujnika (mm)
const int SENSOR_MAX_RANGE = 1020; // Maksymalny zakres czujnika (mm)

//const int PUMP_DELAY = 5;  // Opóźnienie uruchomienia pompy (sekundy)
//const int PUMP_WORK_TIME = 60;  // Czas pracy pompy (sekundy)

const float EMA_ALPHA = 0.2f;  // Współczynnik wygładzania dla średniej wykładniczej (0-1)
const int SENSOR_AVG_SAMPLES = 3;  // Liczba próbek do uśrednienia pomiaru

// Zmienne globalne
float lastFilteredDistance = 0;  // Dla filtra EMA (Exponential Moving Average)
float aktualnaOdleglosc = 0;  // Aktualny dystans
float lastReportedDistance = 0;
unsigned long ostatniCzasDebounce = 0;  // Ostatni czas zmiany stanu przycisku
unsigned long lastMeasurement = 0;
const unsigned long MILLIS_OVERFLOW_THRESHOLD = 4294967295U - 60000; // ~49.7 dni

// Stałe konfiguracyjne

// eeprom
struct Config {
    uint8_t version;  // Wersja konfiguracji
    bool soundEnabled;  // Status dźwięku (włączony/wyłączony)

    // Ustawienia MQTT
    char mqtt_server[40];
    uint16_t mqtt_port;
    char mqtt_user[32];
    char mqtt_password[32];
    
    // Ustawienia zbiornika
    int tank_full;
    int tank_empty;
    int reserve_level;
    int tank_diameter;
    
    // Ustawienia pompy
    int pump_delay;
    int pump_work_time;
    
    // Suma kontrolna
    char checksum;
};
Config config;

const uint8_t CONFIG_VERSION = 1;  // Wersja konfiguracji
const int EEPROM_SIZE = sizeof(Config);  // Rozmiar używanej pamięci EEPROM   

float currentDistance = 0;
float volume = 0;
unsigned long pumpStartTime = 0;
float waterLevelBeforePump = 0;

// Obiekty do komunikacji
WiFiClient client;  // Klient połączenia WiFi
HADevice device("HydroSense");  // Definicja urządzenia dla Home Assistant
HAMqtt mqtt(client, device);  // Klient MQTT dla Home Assistant

// Sensory pomiarowe
HASensor sensorDistance("water_level");  // Odległość od lustra wody (w mm)
HASensor sensorLevel("water_level_percent");  // Poziom wody w zbiorniku (w procentach)
HASensor sensorVolume("water_volume");  // Objętość wody (w litrach)

// Sensory statusu
HASensor sensorPump("pump");  // Praca pompy (ON/OFF)
HASensor sensorWater("water");  // Czujnik poziomu w akwarium (ON=niski/OFF=ok)

// Sensory alarmowe
HASensor sensorAlarm("water_alarm");  // Brak wody w zbiorniku dolewki
HASensor sensorReserve("water_reserve");  // Rezerwa w zbiorniku dolewki

// Przełączniki
HASwitch switchPumpAlarm("pump_alarm");  // Resetowania blokady pompy
HASwitch switchService("service_mode");  // Tryb serwisowy
HASwitch switchSound("sound_switch");  // Dźwięki systemu

// --- Deklaracje funkcji i struktury

// Struktura do przechowywania różnych stanów i parametrów systemu
struct Status {
    bool soundEnabled;  // Flaga wskazująca, czy dźwięk jest włączony
    bool waterAlarmActive;  // Flaga wskazująca, czy alarm wodny jest aktywny
    bool waterReserveActive;  // Flaga wskazująca, czy rezerwa wody jest aktywna
    bool isPumpActive;  // Flaga wskazująca, czy pompa jest aktywna
    bool isPumpDelayActive;  // Flaga wskazująca, czy opóźnienie pompy jest aktywne
    bool pumpSafetyLock;  // Flaga wskazująca, czy blokada bezpieczeństwa pompy jest aktywna
    bool isServiceMode;  // Flaga wskazująca, czy tryb serwisowy jest włączony
    float waterLevelBeforePump;  // Poziom wody przed uruchomieniem pompy
    unsigned long pumpStartTime;  // Znacznik czasu uruchomienia pompy
    unsigned long pumpDelayStartTime;  // Znacznik czasu rozpoczęcia opóźnienia pompy
    unsigned long lastSoundAlert;  // Znacznik czasu ostatniego alertu dźwiękowego
    unsigned long lastSuccessfulMeasurement;  // Znacznik czasu ostatniego udanego pomiaru
};

// Instancja struktury Status
Status status;

// Struktura dla obsługi przycisku
struct ButtonState {
    bool lastState; // Poprzedni stan przycisku
    unsigned long pressedTime = 0; // Czas wciśnięcia przycisku
    unsigned long releasedTime = 0; // Czas puszczenia przycisku
    bool isLongPressHandled = false; // Flaga obsłużonego długiego naciśnięcia
    bool isInitialized = false; 
};

// Instancja struktury ButtonState
ButtonState buttonState;

// Struktura dla dźwięków alarmowych
struct AlarmTone {
    uint16_t frequency;  // Częstotliwość dźwięku
    uint16_t duration;  // Czas trwania
    uint8_t repeats;  // Liczba powtórzeń
    uint16_t pauseDuration;  // Przerwa między powtórzeniami
};

// Struktura do przechowywania różnych znaczników czasowych
struct Timers {
    unsigned long lastMQTTRetry;  // Znacznik czasu ostatniej próby połączenia MQTT
    unsigned long lastMeasurement;  // Znacznik czasu ostatniego pomiaru
    unsigned long lastOTACheck;  // Znacznik czasu ostatniego sprawdzenia OTA (Over-The-Air)
    unsigned long lastMQTTLoop;  // Znacznik czasu ostatniego cyklu pętli MQTT
    
    // Konstruktor inicjalizujący wszystkie znaczniki czasowe na 0
    Timers() : lastMQTTRetry(0), lastMeasurement(0), lastOTACheck(0), lastMQTTLoop(0) {}
};

// Instancja struktury Timers
static Timers timers;

// Funkcja do resetowania do ustawień fabrycznych
void factoryReset() {
    setDefaultConfig();  // Wczytaj domyślne ustawienia
    saveConfig();       // Zapisz je do EEPROM
    ESP.restart();      // Zrestartuj ESP
}

// Funkcja do restartu ESP
void rebootDevice() {
    ESP.restart();
}

// Zerowanie liczników
void handleMillisOverflow() {
    unsigned long currentMillis = millis();
    
    // Sprawdź przepełnienie dla wszystkich timerów
    if (currentMillis < status.pumpStartTime) status.pumpStartTime = 0;
    if (currentMillis < status.pumpDelayStartTime) status.pumpDelayStartTime = 0;
    if (currentMillis < status.lastSoundAlert) status.lastSoundAlert = 0;
    if (currentMillis < status.lastSuccessfulMeasurement) status.lastSuccessfulMeasurement = 0;
    if (currentMillis < lastMeasurement) lastMeasurement = 0;
    
    // Jeśli zbliża się przepełnienie, zresetuj wszystkie timery
    if (currentMillis > MILLIS_OVERFLOW_THRESHOLD) {
        status.pumpStartTime = 0;
        status.pumpDelayStartTime = 0;
        status.lastSoundAlert = 0;
        status.lastSuccessfulMeasurement = 0;
        lastMeasurement = 0;
        
        DEBUG_PRINT(F("Reset timerów - zbliża się przepełnienie millis()"));
    }
}

// --- EEPROM

// Ustawienia domyślne
void setDefaultConfig() {
    config.version = CONFIG_VERSION;
    config.soundEnabled = true;  // Domyślnie dźwięk włączony
    config.checksum = calculateChecksum(config);

    // MQTT
    strlcpy(config.mqtt_server, "192.168.1.14", sizeof(config.mqtt_server));
    config.mqtt_port = 1883;
    strlcpy(config.mqtt_user, "hydrosense", sizeof(config.mqtt_user));
    strlcpy(config.mqtt_password, "hydrosense", sizeof(config.mqtt_password));
    
    // Zbiornik
    config.tank_full = 20;        // Domyślna odległość gdy zbiornik jest pełny (mm)
    config.tank_empty = 1000;      // Domyślna odległość gdy zbiornik jest pusty (mm)
    config.reserve_level = 500;   // Domyślny poziom rezerwy wody (mm)
    config.tank_diameter = 100;   // Domyślna średnica zbiornika (mm)
    
    // Pompa
    config.pump_delay = 5;        // Domyślne opóźnienie uruchomienia pompy (sekundy)
    config.pump_work_time = 30;   //Domyślny czas pracy pompy (sekundy)

    saveConfig();
    DEBUG_PRINT(F("Utworzono domyślną konfigurację"));
}

// Wczytywanie konfiguracji
bool loadConfig() {
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.get(0, config);
    EEPROM.end();
    
    // Sprawdź wersję konfiguracji
    if (config.version != CONFIG_VERSION) {
        DEBUG_PRINT(F("Niekompatybilna wersja konfiguracji"));
        setDefaultConfig();
        return false;
    }
    
    // Sprawdź sumę kontrolną
    char calculatedChecksum = calculateChecksum(config);
    if (config.checksum != calculatedChecksum) {
        DEBUG_PRINT(F("Błąd sumy kontrolnej"));
        setDefaultConfig();
        return false;
    }
    
    // Synchronizuj stan po wczytaniu
    status.soundEnabled = config.soundEnabled;
    switchSound.setState(config.soundEnabled, true);  // force update

    DEBUG_PRINTF("Konfiguracja wczytana. Stan dźwięku: %s\n", 
                 config.soundEnabled ? "WŁĄCZONY" : "WYŁĄCZONY");
    return true;
}

// Zapis do EEPROM
void saveConfig() {
    // Oblicz sumę kontrolną przed zapisem
    config.checksum = calculateChecksum(config);
    
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.put(0, config);
    bool success = EEPROM.commit();
    EEPROM.end();

    if (success) {
        DEBUG_PRINT(F("Konfiguracja zapisana pomyślnie"));
    } else {
        DEBUG_PRINT(F("Błąd zapisu konfiguracji!"));
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

// Długi sygnał dźwiękowy - pojedyncze piknięcie
void playConfirmationSound() {
    if (config.soundEnabled) {
        tone(BUZZER_PIN, 2000, 200); // Dłuższe piknięcie (2000Hz, 200ms)
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
    if (currentDistance >= config.tank_empty && !status.waterAlarmActive) {
        status.waterAlarmActive = true;
        sensorAlarm.setValue("ON");               
        DEBUG_PRINT("Brak wody ON");
    } 
    // Wyłącz alarm jeśli:
    // - odległość spadła poniżej progu wyłączenia (z histerezą)
    // - alarm jest aktywny
    else if (currentDistance < (config.tank_empty - HYSTERESIS) && status.waterAlarmActive) {
        status.waterAlarmActive = false;
        sensorAlarm.setValue("OFF");
        DEBUG_PRINT("Brak wody OFF");
    }

    // --- Obsługa ostrzeżenia o rezerwie wody ---
    // Włącz ostrzeżenie o rezerwie jeśli:
    // - odległość osiągnęła próg rezerwy
    // - ostrzeżenie nie jest jeszcze aktywne
    if (currentDistance >= config.reserve_level && !status.waterReserveActive) {
        status.waterReserveActive = true;
        sensorReserve.setValue("ON");
        DEBUG_PRINT("Rezerwa ON");
    } 
    // Wyłącz ostrzeżenie o rezerwie jeśli:
    // - odległość spadła poniżej progu rezerwy (z histerezą)
    // - ostrzeżenie jest aktywne
    else if (currentDistance < (config.reserve_level - HYSTERESIS) && status.waterReserveActive) {
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
    if (status.isPumpActive && (currentMillis - status.pumpStartTime >= config.pump_work_time * 1000)) {
        stopPump();
        status.pumpSafetyLock = true;
        switchPumpAlarm.setState(true);
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
        switchPumpAlarm.setState(true, true);  // Wymuś aktualizację stanu na ON w HA
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
        if (currentMillis - status.pumpDelayStartTime >= (config.pump_delay * 1000)) {
            startPump();
        }
    }
}

// Zatrzymanie pompy
void stopPump() {
    digitalWrite(POMPA_PIN, LOW);
    status.isPumpActive = false;
    status.pumpStartTime = 0;
    sensorPump.setValue("OFF");
    DEBUG_PRINT(F("Pompa zatrzymana"));
}

// Uruchomienie pompy
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
        playConfirmationSound();  // Sygnał potwierdzenia zmiany trybu
        status.pumpSafetyLock = false;  // Wyłącz blokadę bezpieczeństwa pompy
        switchPumpAlarm.setState(false);  // Aktualizuj stan przełącznika w HA na OFF        
    }
}

// --- Deklaracje funkcji związanych z siecią

// Konfiguracja i zarządzanie połączeniem WiFi
void setupWiFi() {
    WiFiManager wifiManager;
    
    // Konfiguracja AP
    // Nazwa AP: "HydroSense-Setup"
    // Hasło do AP: "hydrosense"
    
    wifiManager.setAPCallback([](WiFiManager *myWiFiManager) {
        DEBUG_PRINT("Tryb punktu dostępowego");
        DEBUG_PRINT("SSID: HydroSense");
        DEBUG_PRINT("IP: 192.168.4.1");
        if (status.soundEnabled) {
            tone(BUZZER_PIN, 1000, 1000); // Sygnał dźwiękowy informujący o trybie AP
        }
    });
    
    wifiManager.setConfigPortalTimeout(180); // 3 minuty na konfigurację
    
    // Próba połączenia lub utworzenia AP
    if (!wifiManager.autoConnect("HydroSense", "hydrosense")) {
        DEBUG_PRINT("Nie udało się połączyć i timeout upłynął");
        ESP.restart(); // Restart ESP w przypadku niepowodzenia
    }
    
    DEBUG_PRINT("Połączono z WiFi!");
    
    // Pokaż uzyskane IP
    DEBUG_PRINT("IP: ");
    DEBUG_PRINT(WiFi.localIP().toString().c_str());
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
    device.setSoftwareVersion(SOFTWARE_VERSION);         // Wersja oprogramowania

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
    switchService.setIcon("mdi:account-wrench-outline");            // Ikona narzędzi
    switchService.onCommand(onServiceSwitchCommand);// Funkcja obsługi zmiany stanu
    switchService.setState(status.isServiceMode);   // Stan początkowy
    // Inicjalizacja stanu - domyślnie wyłączony
    status.isServiceMode = false;
    switchService.setState(false, true); // force update przy starcie

    switchSound.setName("Dźwięk");
    switchSound.setIcon("mdi:volume-high");        // Ikona głośnika
    switchSound.onCommand(onSoundSwitchCommand);   // Funkcja obsługi zmiany stanu
    switchSound.setState(status.soundEnabled);      // Stan początkowy
    
    switchPumpAlarm.setName("Alarm pompy");
    switchPumpAlarm.setIcon("mdi:alert");               // Ikona alarmu
    switchPumpAlarm.onCommand(onPumpAlarmCommand);      // Funkcja obsługi zmiany stanu
}

// --- Deklaracje funkcji ogólnych

// Konfiguracja kierunków pinów i stanów początkowych
void setupPin() {
    pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);  // Wyjście - trigger czujnika ultradźwiękowego
    pinMode(PIN_ULTRASONIC_ECHO, INPUT);  // Wejście - echo czujnika ultradźwiękowego
    digitalWrite(PIN_ULTRASONIC_TRIG, LOW);  // Upewnij się że TRIG jest LOW na starcie
    
    pinMode(PIN_WATER_LEVEL, INPUT_PULLUP);  // Wejście z podciąganiem - czujnik poziomu
    pinMode(PRZYCISK_PIN, INPUT_PULLUP);  // Wejście z podciąganiem - przycisk
    pinMode(BUZZER_PIN, OUTPUT);  // Wyjście - buzzer
    digitalWrite(BUZZER_PIN, LOW);  // Wyłączenie buzzera
    pinMode(POMPA_PIN, OUTPUT);  // Wyjście - pompa
    digitalWrite(POMPA_PIN, LOW);  // Wyłączenie pompy
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
    status.waterAlarmActive = (initialDistance >= config.tank_empty);
    status.waterReserveActive = (initialDistance >= config.reserve_level);
    
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
    playConfirmationSound();  // Sygnał potwierdzenia zmiany trybu
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
    
    DEBUG_PRINTF("Tryb serwisowy: %s (przez HA)\n", state ? "WŁĄCZONY" : "WYŁĄCZONY");
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
    int measurements[SENSOR_AVG_SAMPLES];
    int validCount = 0;

    // Zakres akceptowalnych pomiarów o margines
    const int MIN_VALID_DISTANCE = SENSOR_MIN_RANGE;  // 20mm
    const int MAX_VALID_DISTANCE = SENSOR_MAX_RANGE;  // 1020mm

    for (int i = 0; i < SENSOR_AVG_SAMPLES; i++) {
        measurements[i] = -1; // Domyślna wartość błędu
        
        // Reset trigger
        digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
        delayMicroseconds(2);
        
        // Wysłanie impulsu 10µs
        digitalWrite(PIN_ULTRASONIC_TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(PIN_ULTRASONIC_TRIG, LOW);

        // Pomiar czasu odpowiedzi z timeoutem
        unsigned long startTime = micros();
        unsigned long timeout = startTime + 25000; // 25ms timeout
        
        // Czekaj na początek echa
        bool validEcho = true;
        while (digitalRead(PIN_ULTRASONIC_ECHO) == LOW) {
            if (micros() > timeout) {
                DEBUG_PRINT(F("Limit czasu rozpoczęcia echa"));
                validEcho = false;
                break;
            }
        }
        
        if (validEcho) {
            startTime = micros();
            
            // Czekaj na koniec echa
            while (digitalRead(PIN_ULTRASONIC_ECHO) == HIGH) {
                if (micros() > timeout) {
                    DEBUG_PRINT(F("Limit czasu zakończenia echa"));
                    validEcho = false;
                    break;
                }
            }
            
            if (validEcho) {
                unsigned long duration = micros() - startTime;
                int distance = (duration * 343) / 2000; // Prędkość dźwięku 343 m/s

                // Walidacja odległości
                if (distance >= MIN_VALID_DISTANCE && distance <= MAX_VALID_DISTANCE) {
                    measurements[i] = distance;
                    validCount++;
                } else {
                    DEBUG_PRINT(F("Odległość poza zasięgiem: "));
                    DEBUG_PRINT(distance);
                }
            }
        }
        delay(ULTRASONIC_TIMEOUT);
    }

    // Sprawdź czy mamy wystarczająco dużo poprawnych pomiarów
    if (validCount < (SENSOR_AVG_SAMPLES / 2)) {
        DEBUG_PRINT(F("Za mało prawidłowych pomiarów: "));
        DEBUG_PRINT(validCount);
        return -1;
    }

    // Przygotuj tablicę na poprawne pomiary
    int validMeasurements[validCount];
    int validIndex = 0;
    
    // Kopiuj tylko poprawne pomiary
    for (int i = 0; i < SENSOR_AVG_SAMPLES; i++) {
        if (measurements[i] != -1) {
            validMeasurements[validIndex++] = measurements[i];
        }
    }

    // Sortowanie
    for (int i = 0; i < validCount - 1; i++) {
        for (int j = 0; j < validCount - i - 1; j++) {
            if (validMeasurements[j] > validMeasurements[j + 1]) {
                int temp = validMeasurements[j];
                validMeasurements[j] = validMeasurements[j + 1];
                validMeasurements[j + 1] = temp;
            }
        }
    }

    // Oblicz medianę
    float medianValue;
    if (validCount % 2 == 0) {
        medianValue = (validMeasurements[(validCount-1)/2] + validMeasurements[validCount/2]) / 2.0;
    } else {
        medianValue = validMeasurements[validCount/2];
    }

    // Zastosuj filtr EMA
    if (lastFilteredDistance == 0) {
        lastFilteredDistance = medianValue;
    }
    
    // Zastosuj filtr EMA z ograniczeniem maksymalnej zmiany
    float maxChange = 10.0; // maksymalna zmiana między pomiarami w mm
    float currentChange = medianValue - lastFilteredDistance;
    if (abs(currentChange) > maxChange) {
        medianValue = lastFilteredDistance + (currentChange > 0 ? maxChange : -maxChange);
    }
    
    lastFilteredDistance = (EMA_ALPHA * medianValue) + ((1 - EMA_ALPHA) * lastFilteredDistance);

    // Końcowe ograniczenie do rzeczywistego zakresu zbiornika
    if (lastFilteredDistance < config.tank_full) lastFilteredDistance = config.tank_full;
    if (lastFilteredDistance > config.tank_empty) lastFilteredDistance = config.tank_empty;

    return (int)lastFilteredDistance;
}

// Funkcja obliczająca poziom wody w zbiorniku dolewki w procentach
//  
// @param distance - zmierzona odległość od czujnika do lustra wody w mm
// @return int - poziom wody w procentach (0-100%)
// Wzór: ((config.tank_empty - distance) / (config.tank_empty - config.tank_full)) * 100
int calculateWaterLevel(int distance) {
    // Ograniczenie wartości do zakresu pomiarowego
    if (distance < SENSOR_MIN_RANGE) distance = SENSOR_MIN_RANGE;  // Nie mniej niż przy pełnym
    if (distance > SENSOR_MAX_RANGE) distance = SENSOR_MAX_RANGE;  // Nie więcej niż przy pustym
   
    // Obliczenie procentowe poziomu wody
    float percentage = (float)(config.tank_empty - distance) /  // Różnica: pusty - aktualny
                      (float)(config.tank_empty - config.tank_full) *  // Różnica: pusty - pełny
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
    float waterHeight = config.tank_empty - currentDistance;    
    waterHeight = constrain(waterHeight, 0, config.tank_empty - config.tank_full);
    
    // Obliczenie objętości w litrach (wszystko w mm)
    float radius = config.tank_diameter / 2.0;
    volume = PI * (radius * radius) * waterHeight / 1000000.0; // mm³ na litry
    
    // Aktualizacja sensorów pomiarowych
    sensorDistance.setValue(String((int)currentDistance).c_str());
    sensorLevel.setValue(String(calculateWaterLevel(currentDistance)).c_str());
        
    char valueStr[10];
    dtostrf(volume, 1, 1, valueStr);
    sensorVolume.setValue(valueStr);
        
    //Debug info tylko gdy wartości się zmieniły (conajmniej 5mm)
    static float lastReportedDistance = 0;
    if (abs(currentDistance - lastReportedDistance) > 5) {
        DEBUG_PRINTF("Poziom: %.1f mm, Obj: %.1f L\n", currentDistance, volume);
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
                    playConfirmationSound();  // Sygnał potwierdzenia zmiany trybu
                    switchService.setState(status.isServiceMode, true);  // force update w HA
                    
                    // Log zmiany stanu
                    DEBUG_PRINTF("Tryb serwisowy: %s (przez przycisk)\n", status.isServiceMode ? "WŁĄCZONY" : "WYŁĄCZONY");
                    
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
                playConfirmationSound();  // Sygnał potwierdzenia zmiany trybu
                switchPumpAlarm.setState(false, true);  // force update w HA
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
    
    // Zagraj dźwięk potwierdzenia tylko gdy włączamy dźwięk
    if (state) {
        playConfirmationSound();
    }
    
    DEBUG_PRINTF("Zmieniono stan dźwięku na: ", state ? "WŁĄCZONY" : "WYŁĄCZONY");
}

const char CONFIG_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>HydroSense - Konfiguracja</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body { 
            font-family: Arial, sans-serif; 
            line-height: 1.6;
            background: #1a1a1a;
            color: #e0e0e0;
            min-height: 100vh;
        }
        .container { 
            width: 100%;
            max-width: 1200px;
            margin: 0 auto;
            padding: 15px;
        }
        .header {
            background: #2C3E50;
            color: white;
            padding: 1rem;
            text-align: center;
            margin-bottom: 20px;
            border-radius: 4px;
        }
        .header h1 {
            margin: 0;
            font-size: 2em;
        }
        .status-panel, .config-panel {
            background: #2d2d2d;
            padding: 20px;
            border-radius: 4px;
            margin-bottom: 20px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.2);
        }
        .status-panel h3, .config-panel h2 {
            color: #4a9eff;
            margin-bottom: 15px;
            padding-bottom: 10px;
            border-bottom: 2px solid #3d3d3d;
            font-size: 1.5em;  /* Ujednolicona wielkość dla wszystkich nagłówków sekcji */
            font-weight: 500;
        }
        .status-item {
            display: flex;
            justify-content: space-between;
            padding: 8px 0;
            border-bottom: 1px solid #3d3d3d;
            font-size: 0.95em;  /* Ujednolicona wielkość dla opisów */
        }
        .status-item:last-child {
            border-bottom: none;
        }
        .form-group {
            margin-bottom: 15px;
            padding: 10px;
            background: #333333;
            border-radius: 4px;
        }
        label {
            display: block;
            margin-bottom: 8px;
            color: #e0e0e0;
            font-size: 0.95em;  /* Ujednolicona wielkość dla opisów */
            font-weight: normal;
        }
        input {
            width: 100%;
            padding: 10px;
            border: 1px solid #444;
            border-radius: 4px;
            font-size: 0.95em;  /* Ujednolicona wielkość dla pól input */
            background: #404040;
            color: #e0e0e0;
        }
        input[type="number"] {
            width: 200px;
        }
        button {
            background: #4a9eff;
            color: white;
            padding: 12px 24px;
            border: none;
            border-radius: 4px;
            cursor: pointer;
            font-size: 1em;
            width: 100%;
            margin-top: 20px;
        }
        button:hover {
            background: #3a8eef;
        }
        .alert {
            padding: 10px;
            margin: 10px 0;
            border-radius: 4px;
            text-align: center;
        }
        .alert.success {
            background-color: #d4edda;
            color: #155724;
            border: 1px solid #c3e6cb;
        }
        .success {
            background-color: #d4edda;
            color: #155724;
            border: 1px solid #c3e6cb;
        }
        .alert.error {
            background-color: #f8d7da;
            color: #721c24;
            border: 1px solid #f5c6cb;
        }
        .error {
            background-color: #f8d7da;
            color: #721c24;
            border: 1px solid #f5c6cb;
        }
        input:focus {
            outline: none;
            border-color: #4a9eff;
        }
        @media (max-width: 768px) {
            .container {
                padding: 10px;
            }
            input[type="number"] {
                width: 100%;
            }
        }
                /* Style dla statusów */
        .status-item span:last-child {
            padding: 2px 8px;
            border-radius: 3px;
            font-weight: bold;
        }
        .status-item .success {
            color: #3c763d;
            background-color: #dff0d8;
            border: 1px solid #d6e9c6;
        }
        .status-item .error {
            color: #a94442;
            background-color: #f2dede;
            border: 1px solid #ebccd1;
        }
    </style>
</head>
<body>
    %MESSAGE%
    <div class="container">
        <div class="header">
            <h1>HydroSense</h1>
        </div>
        
        <div class="status-panel">
            <h3>Status systemu</h3>
            <div class="status-item">
                <span>Status MQTT:</span>
                <span class="%MQTT_STATUS_CLASS%">%MQTT_STATUS%</span>
            </div>
            <div class="status-item">
                <span>Status dźwięku:</span>
                <span class="%SOUND_STATUS_CLASS%">%SOUND_STATUS%</span>
            </div>
            <div class="status-item">
                <span>Wersja systemu:</span>
                <span>%SOFTWARE_VERSION%</span>
            </div>
        </div>

        <form method="POST" action="/save">
            <div class="config-panel">
                <h2>Ustawienia MQTT</h2>
                <div class="form-group">
                    <label>Adres serwera MQTT:</label>
                    <input type="text" name="mqtt_server" value="%MQTT_SERVER%" required>
                </div>
                <div class="form-group">
                    <label>Port MQTT:</label>
                    <input type="number" name="mqtt_port" value="%MQTT_PORT%" required>
                </div>
                <div class="form-group">
                    <label>Użytkownik MQTT:</label>
                    <input type="text" name="mqtt_user" value="%MQTT_USER%">
                </div>
                <div class="form-group">
                    <label>Hasło MQTT:</label>
                    <input type="password" name="mqtt_password" value="%MQTT_PASSWORD%">
                </div>
            </div>

            <div class="config-panel">
                <h2>Ustawienia zbiornika</h2>
                <div class="form-group">
                    <label>Poziom pełnego zbiornika (mm):</label>
                    <input type="number" name="tank_full" value="%TANK_FULL%" required>
                </div>
                <div class="form-group">
                    <label>Poziom pustego zbiornika (mm):</label>
                    <input type="number" name="tank_empty" value="%TANK_EMPTY%" required>
                </div>
                <div class="form-group">
                    <label>Poziom rezerwy (mm):</label>
                    <input type="number" name="reserve_level" value="%RESERVE_LEVEL%" required>
                </div>
                <div class="form-group">
                    <label>Średnica zbiornika (mm):</label>
                    <input type="number" name="tank_diameter" value="%TANK_DIAMETER%" required>
                </div>
            </div>

            <div class="config-panel">
                <h2>Ustawienia pompy</h2>
                <div class="form-group">
                    <label>Opóźnienie pompy (s):</label>
                    <input type="number" name="pump_delay" value="%PUMP_DELAY%" required>
                </div>
                <div class="form-group">
                    <label>Czas pracy pompy (s):</label>
                    <input type="number" name="pump_work_time" value="%PUMP_WORK_TIME%" required>
                </div>
            </div>

            <button type="submit">Zapisz ustawienia</button>
        </form>
    </div>
</body>
</html>
)rawliteral";

String getConfigPage() {
    String page = F("<!DOCTYPE html>"
                   "<html><head>"
                   "<title>HydroSense - Konfiguracja</title>"
                   "<meta charset='UTF-8'>"
                   "<style>"
                   "body { font-family: Arial; margin: 20px; }"
                   ".container { max-width: 800px; margin: 0 auto; }"
                   "input[type='number'] { width: 100px; }"
                   ".section { background-color: #f0f0f0; padding: 10px; margin: 10px 0; border-radius: 5px; }"
                   ".btn { padding: 10px 20px; border: none; border-radius: 5px; cursor: pointer; margin: 5px; }"
                   ".btn-blue { background-color: #007bff; color: white; }"
                   ".btn-red { background-color: #dc3545; color: white; }"
                   "table { width: 100%; border-collapse: collapse; }"
                   "td { padding: 5px; }"
                   "input[type='submit'] { background-color: #4CAF50; color: white; padding: 10px 20px; border: none; border-radius: 5px; cursor: pointer; }"
                   "</style>"
                   "<script>"
                   "function confirmReset() {"
                   "  return confirm('Czy na pewno chcesz przywrócić ustawienia fabryczne? Spowoduje to utratę wszystkich ustawień.');"
                   "}"
                   "function rebootDevice() {"
                   "  if(confirm('Czy na pewno chcesz zrestartować urządzenie?')) {"
                   "    fetch('/reboot', {method: 'POST'});"
                   "  }"
                   "}"
                   "function factoryReset() {"
                   "  if(confirmReset()) {"
                   "    fetch('/factory-reset', {method: 'POST'});"
                   "  }"
                   "}"
                   "</script>"
                   "</head><body><div class='container'>"
                   "<h1>HydroSense - Konfiguracja</h1>");

    // Sekcja statusu systemu
    page += F("<div class='section'><h2>Status systemu</h2>");
    page += F("<table>");
    page += F("<tr><td>Poziom wody:</td><td>") + String(currentWaterLevel) + F("%</td></tr>");
    page += F("<tr><td>Objętość wody:</td><td>") + String(currentWaterVolume) + F(" L</td></tr>");
    page += F("<tr><td>Ostatni pomiar:</td><td>") + String(lastMeasurementDistance) + F(" mm</td></tr>");
    page += F("<tr><td>Stan pompy:</td><td>") + String(pumpState ? "Włączona" : "Wyłączona") + F("</td></tr>");
    page += F("<tr><td>Stan alarmu:</td><td>") + String(alarmState ? "Aktywny" : "Nieaktywny") + F("</td></tr>");
    page += F("</table></div>");

    // Sekcja z przyciskami
    page += F("<div class='section'>"
             "<button class='btn btn-blue' onclick='rebootDevice()'>Reboot</button>"
             "<button class='btn btn-red' onclick='factoryReset()'>Ustawienia fabryczne</button>"
             "</div>");

    // Formularz z ustawieniami
    page += F("<form method='POST' action='/save'>");

    // Sekcja ustawień MQTT
    page += F("<div class='section'><h2>Ustawienia MQTT</h2>");
    page += F("<table>");
    page += F("<tr><td>MQTT Host:</td><td><input type='text' name='mqtt_host' value='") + String(config.mqtt_host) + F("'></td></tr>");
    page += F("<tr><td>MQTT Port:</td><td><input type='number' name='mqtt_port' value='") + String(config.mqtt_port) + F("'></td></tr>");
    page += F("<tr><td>MQTT User:</td><td><input type='text' name='mqtt_user' value='") + String(config.mqtt_user) + F("'></td></tr>");
    page += F("<tr><td>MQTT Password:</td><td><input type='password' name='mqtt_password' value='") + String(config.mqtt_password) + F("'></td></tr>");
    page += F("</table></div>");

    // Sekcja ustawień zbiornika
    page += F("<div class='section'><h2>Wymiary zbiornika</h2>");
    page += F("<table>");
    page += F("<tr><td>Szerokość [mm]:</td><td><input type='number' name='tank_width' value='") + String(config.tank_width) + F("'></td></tr>");
    page += F("<tr><td>Długość [mm]:</td><td><input type='number' name='tank_length' value='") + String(config.tank_length) + F("'></td></tr>");
    page += F("<tr><td>Wysokość [mm]:</td><td><input type='number' name='tank_height' value='") + String(config.tank_height) + F("'></td></tr>");
    page += F("</table></div>");

    // Sekcja ustawień czujnika
    page += F("<div class='section'><h2>Kalibracja czujnika</h2>");
    page += F("<table>");
    page += F("<tr><td>Odległość przy pustym [mm]:</td><td><input type='number' name='tank_empty' value='") + String(config.tank_empty) + F("'></td></tr>");
    page += F("<tr><td>Odległość przy pełnym [mm]:</td><td><input type='number' name='tank_full' value='") + String(config.tank_full) + F("'></td></tr>");
    page += F("<tr><td>Poziom rezerwy [%]:</td><td><input type='number' name='reserve_level' value='") + String(config.reserve_level) + F("'></td></tr>");
    page += F("</table></div>");

    // Sekcja ustawień pompy
    page += F("<div class='section'><h2>Ustawienia pompy</h2>");
    page += F("<table>");
    page += F("<tr><td>Opóźnienie pompy [s]:</td><td><input type='number' name='pump_delay' value='") + String(config.pump_delay) + F("'></td></tr>");
    page += F("<tr><td>Czas pracy pompy [s]:</td><td><input type='number' name='pump_time' value='") + String(config.pump_time) + F("'></td></tr>");
    page += F("</table></div>");

    // Przycisk zapisu
    page += F("<div class='section'>");
    page += F("<input type='submit' value='Zapisz ustawienia'>");
    page += F("</div>");
    
    page += F("</form></div></body></html>");

    return page;
}

void handleRoot() {
    server.send(200, "text/html", getConfigPage());
}

// Dodaj tę funkcję przed handleSave()
bool validateConfigValues() {
    // Sprawdź sensowność wartości
    if (server.arg("tank_full").toInt() >= server.arg("tank_empty").toInt() ||
        server.arg("reserve_level").toInt() >= server.arg("tank_empty").toInt() ||
        server.arg("tank_diameter").toInt() <= 0 ||
        server.arg("pump_delay").toInt() < 0 ||
        server.arg("pump_work_time").toInt() <= 0) {
        return false;
    }
    return true;
}

void handleSave() {
    if (server.method() != HTTP_POST) {
        server.send(405, "text/plain", "Method Not Allowed");
        return;
    }

    // Przed zapisem sprawdź poprawność
    if (!validateConfigValues()) {
        server.send(400, "text/plain", "Nieprawidłowe wartości! Sprawdź wprowadzone dane.");
        return;
    }

    bool needMqttReconnect = false;

    // Sprawdź czy dane MQTT się zmieniły
    if (strcmp(config.mqtt_server, server.arg("mqtt_server").c_str()) != 0 ||
        config.mqtt_port != server.arg("mqtt_port").toInt() ||
        strcmp(config.mqtt_user, server.arg("mqtt_user").c_str()) != 0 ||
        strcmp(config.mqtt_password, server.arg("mqtt_password").c_str()) != 0) {
        needMqttReconnect = true;
    }

    // Zapisz ustawienia MQTT
    strlcpy(config.mqtt_server, server.arg("mqtt_server").c_str(), sizeof(config.mqtt_server));
    config.mqtt_port = server.arg("mqtt_port").toInt();
    strlcpy(config.mqtt_user, server.arg("mqtt_user").c_str(), sizeof(config.mqtt_user));
    strlcpy(config.mqtt_password, server.arg("mqtt_password").c_str(), sizeof(config.mqtt_password));

    // Zapisz ustawienia zbiornika
    config.tank_full = server.arg("tank_full").toInt();
    config.tank_empty = server.arg("tank_empty").toInt();
    config.reserve_level = server.arg("reserve_level").toInt();
    config.tank_diameter = server.arg("tank_diameter").toInt();

    // Zapisz ustawienia pompy
    config.pump_delay = server.arg("pump_delay").toInt();
    config.pump_work_time = server.arg("pump_work_time").toInt();

    // Zapisz konfigurację
    saveConfig();

    // Jeśli dane MQTT się zmieniły, zrestartuj połączenie
    if (needMqttReconnect) {
        if (mqtt.isConnected()) {
            mqtt.disconnect();
        }
        connectMQTT();
    }

    // Przekieruj z powrotem na stronę główną z komunikatem sukcesu
    server.sendHeader("Location", "/?status=saved");
    server.send(303);
}

void setupWebServer() {
    server.on("/", handleRoot);
    server.on("/save", handleSave);
    
    // Dodaj nowe endpointy
    server.on("/reboot", HTTP_POST, []() {
        server.send(200, "text/plain", "Restarting...");
        delay(1000);
        rebootDevice();
    });
    
    server.on("/factory-reset", HTTP_POST, []() {
        server.send(200, "text/plain", "Resetting to factory defaults...");
        delay(1000);
        factoryReset();
    });
    
    server.begin();
}

// --- SETUP ---
void setup() {
    ESP.wdtEnable(WATCHDOG_TIMEOUT);  // Aktywacja watchdoga
    Serial.begin(115200);  // Inicjalizacja portu szeregowego
    DEBUG_PRINT("\nHydroSense start...");  // Komunikat startowy
    
    // Wczytaj konfigurację
    if (!loadConfig()) {
        DEBUG_PRINT(F("Tworzenie nowej konfiguracji..."));
        setDefaultConfig();
    }
    
    setupPin();
    
    // Nawiązanie połączenia WiFi
    setupWiFi();

    setupWebServer();

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
    
    // Konfiguracja OTA
    ArduinoOTA.setHostname("HydroSense");  // Ustaw nazwę urządzenia
    ArduinoOTA.setPassword("hydrosense");  // Ustaw hasło dla OTA
    ArduinoOTA.begin();  // Uruchom OTA    
    
    DEBUG_PRINT("Setup zakończony pomyślnie!");

    // Powitanie
    if (status.soundEnabled) {  // Gdy jest włączony dzwięk
        welcomeMelody();  //  to odegraj muzyczkę, że program poprawnie wystartował
    }  
}

// --- LOOP ---
void loop() {
    unsigned long currentMillis = millis();  // Pobierz bieżącą wartość millis()

    // KRYTYCZNE OPERACJE CZASOWE
    handleMillisOverflow();  // Obsługa przepełnienia millis()
    updatePump();  // Aktualizacja stanu pompy
    ESP.wdtFeed();  // Reset watchdog timer ESP
    yield();  // Umożliwienie przetwarzania innych zadań

    // BEZPOŚREDNIA INTERAKCJA
    handleButton();  // Obsługa naciśnięcia przycisku
    checkAlarmConditions();  // Sprawdzenie warunków alarmowych
    server.handleClient();  // Obsługa serwera WWW

    // POMIARY I AKTUALIZACJE
    if (currentMillis - timers.lastMeasurement >= MEASUREMENT_INTERVAL) {
        updateWaterLevel();  // Aktualizacja poziomu wody
        timers.lastMeasurement = currentMillis;  // Aktualizacja znacznika czasu ostatniego pomiaru
    }

    // KOMUNIKACJA
    if (currentMillis - timers.lastMQTTLoop >= MQTT_LOOP_INTERVAL) {
        mqtt.loop();  // Obsługa pętli MQTT
        timers.lastMQTTLoop = currentMillis;  // Aktualizacja znacznika czasu ostatniej pętli MQTT
    }

    if (currentMillis - timers.lastOTACheck >= OTA_CHECK_INTERVAL) {
        ArduinoOTA.handle();  // Obsługa aktualizacji OTA
        timers.lastOTACheck = currentMillis;  // Aktualizacja znacznika czasu ostatniego sprawdzenia OTA
    }

    // ZARZĄDZANIE POŁĄCZENIEM
    if (!mqtt.isConnected() && 
        (currentMillis - timers.lastMQTTRetry >= MQTT_RETRY_INTERVAL)) {
        timers.lastMQTTRetry = currentMillis;  // Aktualizacja znacznika czasu ostatniej próby połączenia MQTT
        DEBUG_PRINT(F("Brak połączenia MQTT - próba połączenia..."));  // Wydrukuj komunikat debugowania
        if (mqtt.begin(MQTT_SERVER, 1883, MQTT_USER, MQTT_PASSWORD)) {
            DEBUG_PRINT(F("MQTT połączono ponownie!"));  // Wydrukuj komunikat debugowania
        }
    }
}
