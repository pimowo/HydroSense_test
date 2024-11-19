// --- Biblioteki

#include <Arduino.h>  // Podstawowa biblioteka Arduino zawierająca funkcje rdzenia
#include <ArduinoHA.h>  // Biblioteka do integracji z Home Assistant przez protokół MQTT
#include <ArduinoOTA.h>  // Biblioteka do aktualizacji oprogramowania przez sieć WiFi
#include <ESP8266WiFi.h>  // Biblioteka WiFi dedykowana dla układu ESP8266
#include <EEPROM.h>  // Biblioteka do dostępu do pamięci nieulotnej EEPROM
#include <WiFiManager.h>  // Biblioteka do zarządzania połączeniami WiFi

// --- Definicje stałych i zmiennych globalnych

// Wersja systemu
const char* SOFTWARE_VERSION = "19.11.24";

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
const int TANK_FULL = 65;  // Odległość gdy zbiornik jest pełny (mm)
const int TANK_EMPTY = 510;  // Odległość gdy zbiornik jest pusty (mm)
const int RESERVE_LEVEL = 450;  // Poziom rezerwy wody (mm)
const int HYSTERESIS = 10;  // Histereza przy zmianach poziomu (mm)
const int TANK_DIAMETER = 150;  // Średnica zbiornika (mm)
const int VALID_MARGIN = 25;  // Margines błędu dla poprawnych pomiarów (mm)

const int PUMP_DELAY = 5;  // Opóźnienie uruchomienia pompy (sekundy)
const int PUMP_WORK_TIME = 60;  // Czas pracy pompy (sekundy)

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
    char checksum;  // Suma kontrolna
};
Config config;

const uint8_t CONFIG_VERSION = 1;        // Wersja konfiguracji
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

// Zerowanie liczników
void handleMillisOverflow() {
    unsigned long currentMillis = millis();  // Pobierz bieżącą wartość millis()
    
    // Sprawdź, czy zbliża się przepełnienie wartości millis()
    if (currentMillis > MILLIS_OVERFLOW_THRESHOLD) {
        // Reset wszystkich liczników czasu w strukturze status
        status.pumpStartTime = 0;
        status.pumpDelayStartTime = 0;
        status.lastSoundAlert = 0;
        status.lastSuccessfulMeasurement = 0;
        
        DEBUG_PRINT(F("Millis overflow - reset timerów"));  // Wydrukuj komunikat debugowania
    }
    
    // Sprawdź, czy wystąpiło przepełnienie wartości millis()
    if (currentMillis < lastMeasurement) {
        lastMeasurement = currentMillis;  // Zaktualizuj wartość lastMeasurement
        DEBUG_PRINT(F("Wykryto przepełnienie millis()"));  // Wydrukuj komunikat debugowania
    }
}

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

    DEBUG_PRINTF("Konfiguracja wczytana. Stan dźwięku: %s\n", 
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
    if (status.isPumpActive && (currentMillis - status.pumpStartTime >= PUMP_WORK_TIME * 1000)) {
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
        if (currentMillis - status.pumpDelayStartTime >= (PUMP_DELAY * 1000)) {
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

    // Rozszerzony zakres akceptowalnych pomiarów o margines
    const int MIN_VALID_DISTANCE = TANK_FULL - VALID_MARGIN;      // 45mm
    const int MAX_VALID_DISTANCE = TANK_EMPTY + VALID_MARGIN;     // 530mm

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

                // Walidacja odległości z uwzględnieniem marginesu
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
    if (lastFilteredDistance < TANK_FULL) lastFilteredDistance = TANK_FULL;
    if (lastFilteredDistance > TANK_EMPTY) lastFilteredDistance = TANK_EMPTY;

    return (int)lastFilteredDistance;
}

// Funkcja obliczająca poziom wody w zbiorniku dolewki w procentach
//  
// @param distance - zmierzona odległość od czujnika do lustra wody w mm
// @return int - poziom wody w procentach (0-100%)
// Wzór: ((TANK_EMPTY - distance) / (TANK_EMPTY - TANK_FULL)) * 100
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
