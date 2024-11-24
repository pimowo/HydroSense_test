/*
 * HydroSense - System monitorowania poziomu wody
 * Wersja: pierwsza ;)
 * Autor: PMW
 * Data: 
 * 
 * Opis: System do monitorowania i kontroli poziomu wody
 * z integracją z Home Assistant poprzez MQTT
 */

// --- Biblioteki

#include <Arduino.h>  // Podstawowa biblioteka Arduino zawierająca funkcje rdzenia
#include <ArduinoHA.h>  // Biblioteka do integracji z Home Assistant przez protokół MQTT
#include <ArduinoOTA.h>  // Biblioteka do aktualizacji oprogramowania przez sieć WiFi
#include <ESP8266WiFi.h>  // Biblioteka WiFi dedykowana dla układu ESP8266
#include <EEPROM.h>  // Biblioteka do dostępu do pamięci nieulotnej EEPROM
#include <WiFiManager.h>  // Biblioteka do zarządzania połączeniami WiFi
#include <ESP8266WebServer.h>  // Biblioteka do obsługi serwera HTTP na ESP8266
#include <WebSocketsServer.h>  // Biblioteka do obsługi serwera WebSockets na ESP8266
#include <ESP8266HTTPUpdateServer.h>

// --- Definicje stałych i zmiennych globalnych

ESP8266WebServer server(80);  // Tworzenie instancji serwera HTTP na porcie 80
WebSocketsServer webSocket = WebSocketsServer(81);  // Tworzenie instancji serwera WebSockets na porcie 81

// Wersja systemu
const char* SOFTWARE_VERSION = "23.11.24";  // Definiowanie wersji oprogramowania

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
const unsigned long WATCHDOG_TIMEOUT = 8000;  // Timeout dla watchdoga
const unsigned long LONG_PRESS_TIME = 1000;  // Czas długiego naciśnięcia przycisku
const unsigned long MQTT_LOOP_INTERVAL = 100;  // Obsługa MQTT co 100ms
const unsigned long OTA_CHECK_INTERVAL = 1000;  // Sprawdzanie OTA co 1s
const unsigned long MQTT_RETRY_INTERVAL = 10000;  // Próba połączenia MQTT co 10s

const char UPDATE_FORM[] PROGMEM = R"rawliteral(
<div class='section'>
    <h2>Aktualizacja firmware</h2>
    <form method='POST' action='/update' enctype='multipart/form-data'>
        <table class='config-table' style='margin-bottom: 15px;'>
            <tr><td colspan='2'><input type='file' name='update' accept='.bin'></td></tr>
        </table>
        <input type='submit' value='Aktualizuj firmware' class='btn btn-orange'>
    </form>
    <div id='update-progress' style='display:none'>
        <div class='progress'>
            <div id='progress-bar' class='progress-bar' role='progressbar' style='width: 0%'>0%</div>
        </div>
    </div>
</div>
)rawliteral";

const char PAGE_FOOTER[] PROGMEM = R"rawliteral(
<div class='footer'>
    <a href='https://github.com/pimowo/HydroSense' target='_blank'>Project by PMW</a>
</div>
)rawliteral";

// Definicja debugowania
#define DEBUG 1  // 0 wyłącza debug, 1 włącza debug

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
const int HYSTERESIS = 10;  // Histereza przy zmianach poziomu (mm)
const int SENSOR_MIN_RANGE = 20;   // Minimalny zakres czujnika (mm)
const int SENSOR_MAX_RANGE = 1020; // Maksymalny zakres czujnika (mm)
const float EMA_ALPHA = 0.2f;  // Współczynnik wygładzania dla średniej wykładniczej (0-1)
const int SENSOR_AVG_SAMPLES = 3;  // Liczba próbek do uśrednienia pomiaru

// Zmienne globalne
float lastFilteredDistance = 0;  // Dla filtra EMA (Exponential Moving Average)
float lastReportedDistance = 0;
unsigned long lastMeasurement = 0;
const unsigned long MILLIS_OVERFLOW_THRESHOLD = 4294967295U - 60000; // ~49.7 dni

// Stałe konfiguracyjne

// 
struct Config {
    // Wersja konfiguracji - używana do kontroli kompatybilności przy aktualizacjach
    uint8_t version;  
    
    // Globalne ustawienie dźwięku (true = włączony, false = wyłączony)
    bool soundEnabled;  

    // Ustawienia MQTT
    char mqtt_server[40];    // Adres brokera MQTT (IP lub nazwa domeny)
    uint16_t mqtt_port;      // Port serwera MQTT (domyślnie 1883)
    char mqtt_user[32];      // Nazwa użytkownika do autoryzacji MQTT
    char mqtt_password[32];  // Hasło do autoryzacji MQTT

    // Ustawienia
    char webUser[32];  // nazwa użytkownika
    char webPassword[32];  // hasło
    
    // Ustawienia zbiornika
    int tank_full;      // Poziom w cm gdy zbiornik jest pełny (odległość od czujnika)
    int tank_empty;     // Poziom w cm gdy zbiornik jest pusty (odległość od czujnika)
    int reserve_level;  // Poziom rezerwy w cm (ostrzeżenie o niskim poziomie)
    int tank_diameter;  // Średnica zbiornika w cm (do obliczania objętości)
    
    // Ustawienia pompy
    int pump_delay;      // Opóźnienie przed startem pompy w sekundach
    int pump_work_time;  // Maksymalny czas pracy pompy w sekundach
    
    // Suma kontrolna - używana do weryfikacji integralności danych w EEPROM
    char checksum;       // XOR wszystkich poprzednich pól
};

Config config;  // Globalna instancja struktury konfiguracyjnej

const uint8_t CONFIG_VERSION = 2;  // Wersja konfiguracji
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
HASensor sensorPumpWorkTime("pump_work_time");  // Czas pracy pompy

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
    bool lastState;  // Poprzedni stan przycisku
    bool isInitialized = false; 
    bool isLongPressHandled = false;  // Flaga obsłużonego długiego naciśnięcia
    unsigned long pressedTime = 0;  // Czas wciśnięcia przycisku
    unsigned long releasedTime = 0;  // Czas puszczenia przycisku
};

ButtonState buttonState;  // Instancja struktury ButtonState

// Struktura do przechowywania różnych znaczników czasowych
struct Timers {
    unsigned long lastMQTTRetry;  // Znacznik czasu ostatniej próby połączenia MQTT
    unsigned long lastMeasurement;  // Znacznik czasu ostatniego pomiaru
    unsigned long lastOTACheck;  // Znacznik czasu ostatniego sprawdzenia OTA (Over-The-Air)
    unsigned long lastMQTTLoop;  // Znacznik czasu ostatniego cyklu pętli MQTT
    
    // Konstruktor inicjalizujący wszystkie znaczniki czasowe na 0
    Timers() : lastMQTTRetry(0), lastMeasurement(0), lastOTACheck(0), lastMQTTLoop(0) {}
};

static Timers timers;  // Instancja struktury Timers

// Funkcja do resetowania do ustawień fabrycznych
void factoryReset() {    
    WiFi.disconnect(true);  // true = kasuj zapisane ustawienia
    WiFi.mode(WIFI_OFF);   
    delay(100);
    
    WiFiManager wm;
    wm.resetSettings();
    ESP.eraseConfig();
    
    setDefaultConfig();
    saveConfig();
    
    delay(100);
    ESP.reset();
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
// Ustawia domyślną konfigurację urządzenia
// Wywoływana przy pierwszym uruchomieniu lub po resecie do ustawień fabrycznych
void setDefaultConfig() {
    // Podstawowa konfiguracja
    config.version = CONFIG_VERSION;        // Ustawienie wersji konfiguracji
    config.soundEnabled = true;             // Włączenie powiadomień dźwiękowych
    
    // MQTT
    strlcpy(config.mqtt_server, "", sizeof(config.mqtt_server));
    config.mqtt_port = 1883;
    strlcpy(config.mqtt_user, "", sizeof(config.mqtt_user));
    strlcpy(config.mqtt_password, "", sizeof(config.mqtt_password));

    // WWW
    strlcpy(config.webUser, "admin", sizeof(config.webUser));
    strlcpy(config.webPassword, "hydrosense", sizeof(config.webPassword));
    
    // Konfiguracja zbiornika
    config.tank_full = 50;  // czujnik od powierzchni przy pełnym zbiorniku
    config.tank_empty = 1050;  // czujnik od dna przy pustym zbiorniku
    config.reserve_level = 550;  // czujnik od poziomu rezerwy
    config.tank_diameter = 100;  // średnica zbiornika
    
    // Konfiguracja pompy
    config.pump_delay = 5;  // opóźnienie przed startem pompy
    config.pump_work_time = 30;  // Maksymalny czas ciągłej pracy pompy
    
    // Finalizacja
    config.checksum = calculateChecksum(config);  // Obliczenie sumy kontrolnej
    saveConfig();  // Zapis do EEPROM
    
    DEBUG_PRINT(F("Utworzono domyślną konfigurację"));
}

// Wczytywanie konfiguracji
bool loadConfig() {
    EEPROM.begin(sizeof(Config) + 1);  // +1 dla sumy kontrolnej
    
    // Tymczasowa struktura do wczytania danych
    Config tempConfig;
    
    // Wczytaj dane z EEPROM do tymczasowej struktury
    uint8_t *p = (uint8_t*)&tempConfig;
    for (size_t i = 0; i < sizeof(Config); i++) {
        p[i] = EEPROM.read(i);
    }
    
    EEPROM.end();
    
    // Sprawdź sumę kontrolną
    char calculatedChecksum = calculateChecksum(tempConfig);
    if (calculatedChecksum == tempConfig.checksum) {
        // Jeśli suma kontrolna się zgadza, skopiuj dane do głównej struktury config
        memcpy(&config, &tempConfig, sizeof(Config));
        DEBUG_PRINT("Konfiguracja wczytana pomyślnie");
        return true;
    } else {
        DEBUG_PRINT("Błąd sumy kontrolnej - ładowanie ustawień domyślnych");
        setDefaultConfig();
        return false;
    }
}

// Zapis do EEPROM
void saveConfig() {
    EEPROM.begin(sizeof(Config) + 1);  // +1 dla sumy kontrolnej
    
    // Oblicz sumę kontrolną przed zapisem
    config.checksum = calculateChecksum(config);
    
    // Zapisz strukturę do EEPROM
    uint8_t *p = (uint8_t*)&config;
    for (size_t i = 0; i < sizeof(Config); i++) {
        EEPROM.write(i, p[i]);
    }
    
    // Wykonaj faktyczny zapis do EEPROM
    bool success = EEPROM.commit();
    EEPROM.end();
    
    if (success) {
        DEBUG_PRINT("Konfiguracja zapisana pomyślnie");
    } else {
        DEBUG_PRINT("Błąd zapisu konfiguracji!");
    }
}

// Obliczanie sumy kontrolnej
char calculateChecksum(const Config& cfg) {
    const uint8_t* p = (const uint8_t*)&cfg;
    char checksum = 0;
    // Oblicz sumę kontrolną pomijając pole checksum
    for (size_t i = 0; i < offsetof(Config, checksum); i++) {
        checksum ^= p[i];
    }
    return checksum;
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

void updatePump() {
    // Funkcja pomocnicza do wysłania całkowitego czasu pracy
    auto sendPumpWorkTime = [&]() {
        if (status.pumpStartTime > 0) {
            unsigned long totalWorkTime = (millis() - status.pumpStartTime) / 1000; // Konwersja na sekundy
            char timeStr[16];
            itoa(totalWorkTime, timeStr, 10);  // Konwersja liczby na string
            sensorPumpWorkTime.setValue(timeStr);  // Wysyłamy całkowity czas do HA
        }
    };

    // Zabezpieczenie przed przepełnieniem licznika millis()
    if (millis() < status.pumpStartTime) {
        status.pumpStartTime = millis();
    }
    
    if (millis() < status.pumpDelayStartTime) {
        status.pumpDelayStartTime = millis();
    }
    
    // Odczyt stanu czujnika poziomu wody
    // LOW = brak wody - należy uzupełnić
    // HIGH = woda obecna - stan normalny
    bool waterPresent = (digitalRead(PIN_WATER_LEVEL) == LOW);
    sensorWater.setValue(waterPresent ? "ON" : "OFF");
    
    // --- ZABEZPIECZENIE 1: Tryb serwisowy ---
    if (status.isServiceMode) {
        if (status.isPumpActive) {
            digitalWrite(POMPA_PIN, LOW);
            status.isPumpActive = false;
            sendPumpWorkTime();  // Wyślij czas przy wyłączeniu
            status.pumpStartTime = 0;
            sensorPump.setValue("OFF");
        }
        return;
    }

    // --- ZABEZPIECZENIE 2: Maksymalny czas pracy ---
    if (status.isPumpActive && (millis() - status.pumpStartTime > config.pump_work_time * 1000)) {
        digitalWrite(POMPA_PIN, LOW);
        status.isPumpActive = false;
        sendPumpWorkTime();  // Wyślij czas przy wyłączeniu
        status.pumpStartTime = 0;
        sensorPump.setValue("OFF");
        status.pumpSafetyLock = true;
        switchPumpAlarm.setState(true);
        DEBUG_PRINT("ALARM: Pompa pracowała za długo - aktywowano blokadę bezpieczeństwa!");
        return;
    }
    
    // --- ZABEZPIECZENIE 3: Blokady bezpieczeństwa ---
    if (status.pumpSafetyLock || status.waterAlarmActive) {
        if (status.isPumpActive) {
            digitalWrite(POMPA_PIN, LOW);
            status.isPumpActive = false;
            sendPumpWorkTime();  // Wyślij czas przy wyłączeniu
            status.pumpStartTime = 0;
            sensorPump.setValue("OFF");
        }
        return;
    }
    
    // Kontrola poziomu wody w zbiorniku
    float currentDistance = measureDistance();
    if (status.isPumpActive && currentDistance >= config.tank_empty) {
        digitalWrite(POMPA_PIN, LOW);
        status.isPumpActive = false;
        sendPumpWorkTime();  // Wyślij czas przy wyłączeniu
        status.pumpStartTime = 0;
        status.isPumpDelayActive = false;
        sensorPump.setValue("OFF");
        switchPumpAlarm.setState(true);  // Włącz alarm pompy
        DEBUG_PRINT(F("ALARM: Zatrzymano pompę - brak wody w zbiorniku!"));
        return;
    }

    // --- ZABEZPIECZENIE 4: Ochrona przed przepełnieniem ---
    if (!waterPresent && status.isPumpActive) {
        digitalWrite(POMPA_PIN, LOW);
        status.isPumpActive = false;
        sendPumpWorkTime();  // Wyślij czas przy wyłączeniu
        status.pumpStartTime = 0;
        status.isPumpDelayActive = false;
        sensorPump.setValue("OFF");
        return;
    }
    
    // --- LOGIKA WŁĄCZANIA POMPY ---
    if (waterPresent && !status.isPumpActive && !status.isPumpDelayActive) {
        status.isPumpDelayActive = true;
        status.pumpDelayStartTime = millis();
        return;
    }
    
    // Po upływie opóźnienia, włącz pompę
    if (status.isPumpDelayActive && !status.isPumpActive) {
        if (millis() - status.pumpDelayStartTime >= (config.pump_delay * 1000)) {
            digitalWrite(POMPA_PIN, HIGH);
            status.isPumpActive = true;
            status.pumpStartTime = millis();
            status.isPumpDelayActive = false;
            sensorPump.setValue("ON");
        }
    }
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

// Funkcja do kasowania ustawień WiFiManager
void resetWiFiSettings() {
    DEBUG_PRINT(F("Rozpoczynam kasowanie ustawień WiFi..."));
    
    // Najpierw rozłącz WiFi i wyczyść wszystkie zapisane ustawienia
    WiFi.disconnect(false, true);  // false = nie wyłączaj WiFi, true = kasuj zapisane ustawienia
    
    // Upewnij się, że WiFi jest w trybie stacji
    WiFi.mode(WIFI_STA);
    
    // Reset przez WiFiManager
    WiFiManager wm;
    wm.resetSettings();
    
    DEBUG_PRINT(F("Ustawienia WiFi zostały skasowane"));
    delay(100);
}

// Konfiguracja i zarządzanie połączeniem WiFi
void setupWiFi() {
    WiFiManager wifiManager;
       
    wifiManager.setAPCallback([](WiFiManager *myWiFiManager) {
        DEBUG_PRINT("Tryb punktu dostępowego");
        DEBUG_PRINT("SSID: HydroSense");
        DEBUG_PRINT("IP: 192.168.4.1");
        // if (status.soundEnabled) {
        //     tone(BUZZER_PIN, 1000, 1000); // Sygnał dźwiękowy informujący o trybie AP
        // }
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
    if (!mqtt.begin(config.mqtt_server, 1883, config.mqtt_user, config.mqtt_password)) {
        DEBUG_PRINT("\nBŁĄD POŁĄCZENIA MQTT!");
        return false;
    }
    
    DEBUG_PRINT("MQTT połączono pomyślnie!");
    return true;
}

// Funkcja konfigurująca Home Assistant
void setupHA() {
    // Konfiguracja podstawowych informacji o urządzeniu
    device.setName("HydroSense");
    device.setModel("HS ESP8266");
    device.setManufacturer("PMW");
    device.setSoftwareVersion(SOFTWARE_VERSION);

    // Konfiguracja sensorów pomiarowych
    
    // Pomiar odległości
    sensorDistance.setName("Pomiar odległości");
    sensorDistance.setIcon("mdi:ruler");
    sensorDistance.setUnitOfMeasurement("mm");
    
    // Pomiar poziomu wody
    sensorLevel.setName("Poziom wody");
    sensorLevel.setIcon("mdi:cup-water");
    sensorLevel.setUnitOfMeasurement("%");
    
    // Pomiar objętości wody
    sensorVolume.setName("Objętość wody");
    sensorVolume.setIcon("mdi:cup-water");
    sensorVolume.setUnitOfMeasurement("L");

    // Pomiar czasu pracy pompy
    sensorPumpWorkTime.setName("Czas pracy pompy");
    sensorPumpWorkTime.setIcon("mdi:timer-outline");
    sensorPumpWorkTime.setUnitOfMeasurement("s");
    
    // Konfiguracja sensorów statusu
    
    // Status pompy
    sensorPump.setName("Status pompy");
    sensorPump.setIcon("mdi:water-pump");
    
    // Status czujnika wody
    sensorWater.setName("Czujnik wody");
    sensorWater.setIcon("mdi:electric-switch");
    
    // Konfiguracja sensorów alarmowych
    
    // Alarm braku wody
    sensorAlarm.setName("Brak wody");
    sensorAlarm.setIcon("mdi:alarm-light");

    // Alarm rezerwy wody
    sensorReserve.setName("Rezerwa wody");
    sensorReserve.setIcon("mdi:alarm-light-outline");
    
    // Konfiguracja przełączników
    
    // Przełącznik trybu serwisowego
    switchService.setName("Serwis");
    switchService.setIcon("mdi:account-wrench-outline");
    switchService.onCommand(onServiceSwitchCommand);
    status.isServiceMode = false;                  // Domyślnie wyłączony
    switchService.setState(false, true);           // Wymuszenie aktualizacji przy starcie
    
    // Przełącznik dźwięku
    switchSound.setName("Dźwięk");
    switchSound.setIcon("mdi:volume-high");
    switchSound.onCommand(onSoundSwitchCommand);
    switchSound.setState(status.soundEnabled);
    
    // Przełącznik alarmu pompy
    switchPumpAlarm.setName("Alarm pompy");
    switchPumpAlarm.setIcon("mdi:alert");
    switchPumpAlarm.onCommand(onPumpAlarmCommand);
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

    sensorPumpWorkTime.setValue("0");
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
    const int MIN_VALID_DISTANCE = SENSOR_MIN_RANGE;
    const int MAX_VALID_DISTANCE = SENSOR_MAX_RANGE;

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

    // Końcowe ograniczenie do ustawionego zakresu czujnika
    if (lastFilteredDistance < MIN_VALID_DISTANCE) lastFilteredDistance = MIN_VALID_DISTANCE;
    if (lastFilteredDistance > MAX_VALID_DISTANCE) lastFilteredDistance = MAX_VALID_DISTANCE;

    return (int)lastFilteredDistance;
}

// Funkcja obliczająca poziom wody w zbiorniku dolewki w procentach
//  
// @param distance - zmierzona odległość od czujnika do lustra wody w mm
// @return int - poziom wody w procentach (0-100%)
// Wzór: ((config.tank_empty - distance) / (config.tank_empty - config.tank_full)) * 100
int calculateWaterLevel(int distance) {
    // Ograniczenie wartości do zakresu pomiarowego
    //if (distance < SENSOR_MIN_RANGE) distance = SENSOR_MIN_RANGE;  // Nie mniej niż przy pełnym
    //if (distance > SENSOR_MAX_RANGE) distance = SENSOR_MAX_RANGE;  // Nie więcej niż przy pustym
   
    // Ograniczenie do zakresu zbiornika dla obliczenia procentów
    if (distance < config.tank_full) distance = config.tank_full;
    if (distance > config.tank_empty) distance = config.tank_empty;

    // Obliczenie procentowe poziomu wody
    float percentage = (float)(config.tank_empty - distance) /  // Różnica: pusty - aktualny
                      (float)(config.tank_empty - config.tank_full) *  // Różnica: pusty - pełny
                      100.0;  // Przeliczenie na procenty
    
    return (int)percentage;  // Zwrot wartości całkowitej
}

// Aktualizacja poziomu wody i powiązanych czujników
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

// Strona www   
const char CONFIG_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
    <head>
        <meta charset='UTF-8'>
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>HydroSense</title>
        
        <!-- Style CSS -->
        <style>
            /* Podstawowe style strony */
            body { 
                font-family: Arial, sans-serif; 
                margin: 0; 
                padding: 20px; 
                background-color: #1a1a1a;
                color: #ffffff;
            }
            
            /* Kontenery i sekcje */
            .container { 
                max-width: 800px; 
                margin: 0 auto; 
                padding: 0 15px;
            }
            
            .section {
                background-color: #2a2a2a;
                padding: 20px;
                margin-bottom: 20px;
                border-radius: 8px;
                width: 100%;
                box-sizing: border-box;
            }
            
            .buttons-container {
                display: flex;
                justify-content: space-between;
                margin: -5px;
            }
            
            /* Nagłówki */
            h1 { 
                color: #ffffff; 
                text-align: center;
                margin-bottom: 30px;
                font-size: 2.5em;
                background-color: #2d2d2d;
                padding: 20px;
                border-radius: 8px;
                box-shadow: 0 2px 4px rgba(0,0,0,0.2);
            }
            
            h2 { 
                color: #2196F3;
                margin-top: 0;
                font-size: 1.5em;
            }
            
            /* Tabele */
            table { 
                width: 100%; 
                border-collapse: collapse;
            }
            
            td { 
                padding: 12px 8px;
                border-bottom: 1px solid #3d3d3d;
            }
            
            .config-table {
                width: 100%;
                border-collapse: collapse;
                table-layout: fixed;
            }
            
            .config-table td {
                padding: 8px;
            }
            
            .config-table td:first-child {
                width: 65%;
            }
            
            .config-table td:last-child {
                width: 35%;
            }
            /* Style formularzy i inputów */
            .config-table input[type="text"],
            .config-table input[type="password"],
            .config-table input[type="number"] {
                width: 100%;
                padding: 8px;
                border: 1px solid #3d3d3d;
                border-radius: 4px;
                background-color: #1a1a1a;
                color: #ffffff;
                box-sizing: border-box;
            }
            
            input[type="text"],
            input[type="password"],
            input[type="number"] {
                width: 100%;
                padding: 8px;
                border: 1px solid #3d3d3d;
                border-radius: 4px;
                background-color: #1a1a1a;
                color: #ffffff;
                box-sizing: border-box;
                text-align: left;
            }
            
            input[type="submit"] { 
                background-color: #4CAF50; 
                color: white; 
                padding: 12px 24px; 
                border: none; 
                border-radius: 4px; 
                cursor: pointer;
                width: 100%;
                font-size: 16px;
            }
            
            input[type="submit"]:hover { 
                background-color: #45a049; 
            }
            
            /* Style dla przycisków wyboru pliku */
            input[type="file"] {
                background-color: #1a1a1a;
                color: #ffffff;
                padding: 8px;
                border: 1px solid #3d3d3d;
                border-radius: 4px;
                width: 100%;
                cursor: pointer;
            }
            
            input[type="file"]::-webkit-file-upload-button {
                background-color: #2196F3;
                color: white;
                padding: 8px 16px;
                border: none;
                border-radius: 4px;
                cursor: pointer;
                margin-right: 10px;
            }
            
            input[type="file"]::-webkit-file-upload-button:hover {
                background-color: #1976D2;
            }
            
            /* Style komunikatów */
            .success { 
                color: #4CAF50; 
            }
            
            .error { 
                color: #F44336;
            }
            
            .alert { 
                padding: 15px; 
                margin-bottom: 20px; 
                border-radius: 0;
                position: fixed;
                top: 0;
                left: 0;
                right: 0;
                width: 100%;
                z-index: 1000;
                text-align: center;
                animation: fadeOut 0.5s ease-in-out 5s forwards;
            }
            
            .alert.success { 
                background-color: #4CAF50;
                color: white;
                border: none;
                box-shadow: 0 2px 5px rgba(0,0,0,0.2);
            }
            
            /* Style przycisków */
            .btn {
                padding: 12px 24px;
                border: none;
                border-radius: 4px;
                cursor: pointer;
                font-size: 14px;
                width: calc(50% - 10px);
                display: inline-block;
                margin: 5px;
                text-align: center;
            }
            
            .btn-blue { 
                background-color: #2196F3;
                color: white; 
            }
            
            .btn-red { 
                background-color: #F44336;
                color: white; 
            }
            
            .btn-orange {
                background-color: #FF9800 !important;
                color: white !important;
            }
            
            .btn-orange:hover {
                background-color: #F57C00 !important;
            }
            /* Style konsoli i komunikatów */
            .console {
                background-color: #1a1a1a;
                color: #ffffff;
                padding: 15px;
                border-radius: 4px;
                font-family: monospace;
                height: 200px;
                overflow-y: auto;
                margin-top: 10px;
                border: 1px solid #3d3d3d;
            }
            
            .footer {
                background-color: #2d2d2d;
                padding: 20px;
                margin-top: 20px;
                border-radius: 8px;
                text-align: center;
                box-shadow: 0 2px 4px rgba(0,0,0,0.2);
            }
            
            .footer a {
                display: inline-block;
                background-color: #2196F3;
                color: white;
                text-decoration: underline;
                padding: 12px 24px;
                border-radius: 4px;
                font-weight: normal;
                transition: background-color 0.3s;
                width: 100%;
                box-sizing: border-box;
            }
            
            .footer a:hover {
                background-color: #1976D2;
            }
            
            /* Style paska postępu */
            .progress {
                width: 100%;
                background-color: #1a1a1a;
                border: 1px solid #3d3d3d;
                border-radius: 4px;
                padding: 3px;
                margin-top: 15px;
            }
            
            .progress-bar {
                width: 0%;
                height: 20px;
                background-color: #4CAF50;
                border-radius: 2px;
                transition: width 0.3s ease-in-out;
                text-align: center;
                color: white;
                line-height: 20px;
            }
            
            /* Style komunikatów wyskakujących */
            .message {
                position: fixed;
                top: 20px;
                left: 50%;
                transform: translateX(-50%);
                padding: 15px 30px;
                border-radius: 5px;
                color: white;
                opacity: 0;
                transition: opacity 0.3s ease-in-out;
                z-index: 1000;
            }
            
            .message.success {
                background-color: #4CAF50;
                box-shadow: 0 2px 5px rgba(0,0,0,0.2);
            }
            
            .message.error {
                background-color: #f44336;
                box-shadow: 0 2px 5px rgba(0,0,0,0.2);
            }
            
            /* Style responsywne */
            @media (max-width: 600px) {
                body {
                    padding: 10px;
                }
                .container {
                    padding: 0;
                }
                .section {
                    padding: 15px;
                    margin-bottom: 15px;
                }
                .console {
                    height: 150px;
                }
            }
        </style>

        <!-- Skrypty JavaScript -->
        <script>
            // Funkcje pomocnicze
            function confirmReset() {
                return confirm('Czy na pewno chcesz przywrócić ustawienia fabryczne? Spowoduje to utratę wszystkich ustawień!');
            }
            
            // Funkcja wyświetlająca komunikaty
            function showMessage(text, type) {
                // Usuń istniejące komunikaty
                var oldMessages = document.querySelectorAll('.message');
                oldMessages.forEach(function(msg) {
                    msg.remove();
                });
                
                // Stwórz i dodaj nowy komunikat
                var messageBox = document.createElement('div');
                messageBox.className = 'message ' + type;
                messageBox.innerHTML = text;
                document.body.appendChild(messageBox);
                
                // Animacja pokazywania
                setTimeout(function() {
                    messageBox.style.opacity = '1';
                }, 10);
                
                // Animacja ukrywania
                setTimeout(function() {
                    messageBox.style.opacity = '0';
                    setTimeout(function() {
                        messageBox.remove();
                    }, 300);
                }, 3000);
            }
            // Funkcje zarządzania urządzeniem
            function rebootDevice() {
                if(confirm('Czy na pewno chcesz zrestartować urządzenie?')) {
                    fetch('/reboot', {method: 'POST'})
                        .then(() => {
                            showMessage('Urządzenie zostanie zrestartowane...', 'success');
                            setTimeout(() => { 
                                window.location.reload(); 
                            }, 3000);
                        });
                }
            }
            
            function factoryReset() {
                if(confirmReset()) {
                    fetch('/factory-reset', {method: 'POST'})
                        .then(() => {
                            showMessage('Przywracanie ustawień fabrycznych...', 'success');
                            setTimeout(() => { 
                                window.location.reload(); 
                            }, 3000);
                        });
                }
            }
            
            // Inicjalizacja i obsługa WebSocket
            var socket;
            window.onload = function() {
                // Obsługa formularza
                document.querySelector('form[action="/save"]').addEventListener('submit', function(e) {
                    e.preventDefault();
                    
                    var formData = new FormData(this);
                    fetch('/save', {
                        method: 'POST',
                        body: formData
                    })
                    .then(response => {
                        if (!response.ok) {
                            throw new Error('Network response was not ok');
                        }
                    })
                    .catch(error => {
                        showMessage('Błąd podczas zapisywania!', 'error');
                    });
                });
            
                // Inicjalizacja WebSocket
                socket = new WebSocket('ws://' + window.location.hostname + ':81/');
                socket.onmessage = handleWebSocketMessage;
                
                // Dodanie obsługi błędów WebSocket
                socket.onerror = function(error) {
                    console.error('Błąd WebSocket:', error);
                    showMessage('Błąd połączenia WebSocket', 'error');
                };
                
                socket.onclose = function() {
                    console.log('Połączenie WebSocket zostało zamknięte');
                    showMessage('Połączenie zostało zamknięte', 'error');
                };
            };
            
            // Obsługa wiadomości WebSocket
            function handleWebSocketMessage(event) {
                var message = event.data;
                
                if (message.startsWith('update:')) {
                    handleUpdateMessage(message);
                } 
                else if (message.startsWith('save:')) {
                    handleSaveMessage(message);
                }
                else {
                    updateConsole(message);
                }
            }
            
            // Obsługa wiadomości aktualizacji
            function handleUpdateMessage(message) {
                if (message.startsWith('update:error:')) {
                    document.getElementById('update-progress').style.display = 'none';
                    showMessage(message.split(':')[2], 'error');
                    return;
                }
                
                var percentage = message.split(':')[1];
                var progressBar = document.getElementById('progress-bar');
                document.getElementById('update-progress').style.display = 'block';
                progressBar.style.width = percentage + '%';
                progressBar.textContent = percentage + '%';
                
                if (percentage == '100') {
                    document.getElementById('update-progress').style.display = 'none';
                    showMessage('Aktualizacja zakończona pomyślnie! Trwa restart urządzenia...', 'success');
                    setTimeout(function() {
                        window.location.reload();
                    }, 3000);
                }
            }
            
            // Obsługa wiadomości zapisywania
            function handleSaveMessage(message) {
                var parts = message.split(':');
                showMessage(parts[2], parts[1]);
            }
            
            // Aktualizacja konsoli
            function updateConsole(message) {
                var console = document.getElementById('console');
                console.innerHTML += message + '<br>';
                console.scrollTop = console.scrollHeight;
            }
        </script>
    </head>
    <body>
        <h1>HydroSense</h1>
        
        <div class='section'>
            <h2>Status systemu</h2>
            <table class='config-table'>
                <tr>
                    <td>Status MQTT</td>
                    <td><span class='status %MQTT_STATUS_CLASS%'>%MQTT_STATUS%</span></td>
                </tr>
                <tr>
                    <td>Status dźwięku</td>
                    <td><span class='status %SOUND_STATUS_CLASS%'>%SOUND_STATUS%</span></td>
                </tr>
                <tr>
                    <td>Wersja oprogramowania</td>
                    <td>%SOFTWARE_VERSION%</td>
                </tr>
            </table>
        </div>
    
        <!-- Sekcje dynamiczne -->
        %BUTTONS%
        %CONFIG_FORMS%
        %UPDATE_FORM%
        %FOOTER%
    </body>
</html>
)rawliteral";

//
String getConfigPage() {
    String html = FPSTR(CONFIG_PAGE);
    
    // Przygotuj wszystkie wartości przed zastąpieniem
    bool mqttConnected = client.connected();
    String mqttStatus = mqttConnected ? "Połączony" : "Rozłączony";
    String mqttStatusClass = mqttConnected ? "success" : "error";
    String soundStatus = config.soundEnabled ? "Włączony" : "Wyłączony";
    String soundStatusClass = config.soundEnabled ? "success" : "error";
    
    // Sekcja przycisków - zmieniona na String zamiast PROGMEM
    String buttons = F(
        "<div class='section'>"
        "<div class='buttons-container'>"
        "<button class='btn btn-blue' onclick='rebootDevice()'>Restart urządzenia</button>"
        "<button class='btn btn-red' onclick='factoryReset()'>Przywróć ustawienia fabryczne</button>"
        "</div>"
        "</div>"
    );

    // Zastąp wszystkie placeholdery
    html.replace("%MQTT_SERVER%", config.mqtt_server);
    html.replace("%MQTT_PORT%", String(config.mqtt_port));
    html.replace("%MQTT_USER%", config.mqtt_user);
    html.replace("%MQTT_PASSWORD%", config.mqtt_password);
    html.replace("%MQTT_STATUS%", mqttStatus);
    html.replace("%MQTT_STATUS_CLASS%", mqttStatusClass);
    html.replace("%SOUND_STATUS%", soundStatus);
    html.replace("%SOUND_STATUS_CLASS%", soundStatusClass);
    html.replace("%SOFTWARE_VERSION%", SOFTWARE_VERSION);
    html.replace("%TANK_EMPTY%", String(config.tank_empty));
    html.replace("%TANK_FULL%", String(config.tank_full));
    html.replace("%RESERVE_LEVEL%", String(config.reserve_level));
    html.replace("%TANK_DIAMETER%", String(config.tank_diameter));
    html.replace("%PUMP_DELAY%", String(config.pump_delay));
    html.replace("%PUMP_WORK_TIME%", String(config.pump_work_time));
    html.replace("%BUTTONS%", buttons);
    html.replace("%UPDATE_FORM%", FPSTR(UPDATE_FORM));
    html.replace("%FOOTER%", FPSTR(PAGE_FOOTER));
    html.replace("%MESSAGE%", "");
    html.replace("%WEB_USER%", config.webUser);
    html.replace("%WEB_PASSWORD%", config.webPassword);

    // Przygotuj formularze konfiguracyjne
    String configForms = F("<form method='POST' action='/save'>");
    
    // Ustawienia dostępu
    configForms += F("<div class='section'>"
                     "<h2>Ustawienia dostępu</h2>"
                     "<table class='config-table'>"
                     "<tr><td>Nazwa użytkownika WWW</td><td><input type='text' name='webUser' value='");
    configForms += config.webUser;
    configForms += F("'></td></tr>"
                     "</table>"
                     "<button type='button' class='btn btn-blue' onclick='showChangePasswordModal()'>Zmień hasło</button>"
                     "</div>");
    configForms += config.webPassword;
    configForms += F("'></td></tr>"
                     "</table></div>");
    
    // MQTT
    configForms += F("<div class='section'>"
                     "<h2>Konfiguracja MQTT</h2>"
                     "<table class='config-table'>"
                     "<tr><td>Serwer</td><td><input type='text' name='mqtt_server' value='");
    configForms += config.mqtt_server;
    configForms += F("'></td></tr>"
                     "<tr><td>Port</td><td><input type='number' name='mqtt_port' value='");
    configForms += String(config.mqtt_port);
    configForms += F("'></td></tr>"
                     "<tr><td>Użytkownik</td><td><input type='text' name='mqtt_user' value='");
    configForms += config.mqtt_user;
    configForms += F("'></td></tr>"
                     "<tr><td>Hasło</td><td><input type='password' name='mqtt_password' value='");
    configForms += config.mqtt_password;
    configForms += F("'></td></tr>"
                     "</table></div>");
    
    // Zbiornik
    configForms += F("<div class='section'>"
                     "<h2>Ustawienia zbiornika</h2>"
                     "<table class='config-table'>");
    configForms += "<tr><td>Odległość przy pustym [mm]</td><td><input type='number' name='tank_empty' value='" + String(config.tank_empty) + "'></td></tr>";
    configForms += "<tr><td>Odległość przy pełnym [mm]</td><td><input type='number' name='tank_full' value='" + String(config.tank_full) + "'></td></tr>";
    configForms += "<tr><td>Odległość przy rezerwie [mm]</td><td><input type='number' name='reserve_level' value='" + String(config.reserve_level) + "'></td></tr>";
    configForms += "<tr><td>Średnica zbiornika [mm]</td><td><input type='number' name='tank_diameter' value='" + String(config.tank_diameter) + "'></td></tr>";
    configForms += F("</table></div>");
    
    // Pompa
    configForms += F("<div class='section'>"
                     "<h2>Ustawienia pompy</h2>"
                     "<table class='config-table'>");
    configForms += "<tr><td>Opóźnienie załączenia pompy [s]</td><td><input type='number' name='pump_delay' value='" + String(config.pump_delay) + "'></td></tr>";
    configForms += "<tr><td>Czas pracy pompy [s]</td><td><input type='number' name='pump_work_time' value='" + String(config.pump_work_time) + "'></td></tr>";
    configForms += F("</table></div>");
    
    configForms += F("<div class='section'>"
                     "<input type='submit' value='Zapisz ustawienia' class='btn btn-blue'>"
                     "</div></form>");
    
    html.replace("%CONFIG_FORMS%", configForms);
    
    // Sprawdź, czy wszystkie znaczniki zostały zastąpione
    if (html.indexOf('%') != -1) {
        DEBUG_PRINT("Uwaga: Niektóre znaczniki nie zostały zastąpione!");
        int pos = html.indexOf('%');
        DEBUG_PRINT(html.substring(pos - 20, pos + 20));
    }
    
    return html;
}

void handleRoot() {
    String content = getConfigPage();

    server.send(200, "text/html", content);
}

// Sprawdź sensowność wartości
bool validateConfigValues() {
    // Sprawdzenie czy dane logowania nie są puste
    if (strlen(config.webUser) == 0 || strlen(config.webPassword) == 0) {
        return false;
    }

    // Sprawdzenie długości danych logowania
    if (strlen(config.webUser) >= 32 || strlen(config.webPassword) >= 32) {
        return false;
    }

    // Wlidacje
    if (server.arg("tank_full").toInt() >= server.arg("tank_empty").toInt() ||  // Sprawdź, czy poziom pełnego zbiornika jest większy lub równy poziomowi pustego zbiornika
        server.arg("reserve_level").toInt() >= server.arg("tank_empty").toInt() ||  // Sprawdź, czy poziom rezerwy jest większy lub równy poziomowi pustego zbiornika
        server.arg("tank_diameter").toInt() <= 0 ||  // Sprawdź, czy średnica zbiornika jest większa od 0
        server.arg("pump_delay").toInt() < 0 ||  // Sprawdź, czy opóźnienie pompy jest większe lub równe 0
        server.arg("pump_work_time").toInt() <= 0) {  // Sprawdź, czy czas pracy pompy jest większy od 0
        return false;
    }
    return true;
}

//
void handleSave() {
    if (server.method() != HTTP_POST) {
        server.send(405, "text/plain", "Method Not Allowed");
        return;
    }

    if (!server.authenticate(config.webUser, config.webPassword)) {
        return server.requestAuthentication(BASIC_AUTH, "HydroSense", "Unauthorized");
    }

    // Zapisz poprzednie wartości na wypadek błędów
    Config oldConfig = config;
    bool needMqttReconnect = false;
    bool needRestart = false;
    String errorMessage;

    // Sprawdź dane logowania WWW
    String newWebUser = server.arg("webUser");
    String newWebPassword = server.arg("webPassword");
    String newWebPasswordConfirm = server.arg("webPasswordConfirm");

    // Walidacja danych logowania WWW
    if (newWebUser.length() > 0) {
        if (newWebUser.length() >= sizeof(config.webUser)) {
            errorMessage = "Nazwa użytkownika jest za długa";
            goto handleError;
        }
        needRestart = true;
    }

    // Sprawdź czy hasło ma być zmienione
    if (newWebPassword.length() > 0 || newWebPasswordConfirm.length() > 0) {
        if (newWebPassword.length() == 0 || newWebPasswordConfirm.length() == 0) {
            errorMessage = "Oba pola hasła muszą być wypełnione";
            goto handleError;
        }
        if (newWebPassword != newWebPasswordConfirm) {
            errorMessage = "Podane hasła nie są identyczne";
            goto handleError;
        }
        if (newWebPassword.length() >= sizeof(config.webPassword)) {
            errorMessage = "Hasło jest za długie";
            goto handleError;
        }
        needRestart = true;
    }

    // Zapisz poprzednie wartości MQTT do porównania
    String oldServer = config.mqtt_server;
    int oldPort = config.mqtt_port;
    String oldUser = config.mqtt_user;
    String oldPassword = config.mqtt_password;

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

    // Sprawdź poprawność wartości
    if (!validateConfigValues()) {
        errorMessage = "Nieprawidłowe wartości! Sprawdź wprowadzone dane.";
        goto handleError;
    }

    // Zapisz zatwierdzone dane logowania WWW
    if (newWebUser.length() > 0) {
        strlcpy(config.webUser, newWebUser.c_str(), sizeof(config.webUser));
    }
    if (newWebPassword.length() > 0) {
        strlcpy(config.webPassword, newWebPassword.c_str(), sizeof(config.webPassword));
    }

    // Sprawdź czy dane MQTT się zmieniły
    if (oldServer != config.mqtt_server ||
        oldPort != config.mqtt_port ||
        oldUser != config.mqtt_user ||
        oldPassword != config.mqtt_password) {
        needMqttReconnect = true;
    }

    // Zapisz konfigurację do EEPROM
    saveConfig();

    // Jeśli dane MQTT się zmieniły, zrestartuj połączenie
    if (needMqttReconnect) {
        if (mqtt.isConnected()) {
            mqtt.disconnect();
        }
        connectMQTT();
    }

    // Jeśli zmieniono dane logowania, wyślij informację o restarcie
    if (needRestart) {
        webSocket.broadcastTXT("save:success:Zapisano ustawienia! Urządzenie zostanie zrestartowane...");
        server.send(204);
        delay(1000);
        ESP.restart();
    } else {
        webSocket.broadcastTXT("save:success:Zapisano ustawienia!");
        server.send(204);
    }
    return;

handleError:
    config = oldConfig; // Przywróć poprzednie wartości
    webSocket.broadcastTXT("save:error:" + errorMessage);
    server.send(204);
}

//
void sendSerialMessage(String message) {
    Serial.println(message);
}

//
void handleDoUpdate() {
    HTTPUpload& upload = server.upload();
    
    if (upload.status == UPLOAD_FILE_START) {
        // Sprawdź czy plik został wybrany
        if (upload.filename == "") {
            webSocket.broadcastTXT("update:error:Nie wybrano pliku!");
            server.send(204); // Bez przekierowania
            return;
        }
        
        if (!Update.begin(upload.contentLength)) {
            Update.printError(Serial);
            webSocket.broadcastTXT("update:error:Błąd inicjalizacji aktualizacji!");
            server.send(204);
            return;
        }
        webSocket.broadcastTXT("update:0");
    } 
    else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            Update.printError(Serial);
            webSocket.broadcastTXT("update:error:Błąd zapisu pliku!");
            return;
        }
        int progress = (upload.totalSize * 100) / upload.contentLength;
        String progressMsg = "update:" + String(progress);
        webSocket.broadcastTXT(progressMsg);
    } 
    else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
            webSocket.broadcastTXT("update:100");
            server.send(204);
            delay(1000);
            ESP.restart();
        } else {
            Update.printError(Serial);
            webSocket.broadcastTXT("update:error:Błąd zakończenia aktualizacji!");
            server.send(204);
        }
    }
}

void handleUpdateResult() {
    if (Update.hasError()) {
        server.send(200, "text/html", 
            "<h1>Aktualizacja nie powiodła się</h1>"
            "<a href='/'>Powrót</a>");
    } else {
        server.send(200, "text/html", 
            "<h1>Aktualizacja zakończona powodzeniem</h1>"
            "Urządzenie zostanie zrestartowane...");
        delay(1000);
        ESP.restart();
    }
}

void handleChangePassword() {
    if (!server.authenticate(config.webUser, config.webPassword)) {
        return server.requestAuthentication(BASIC_AUTH, "HydroSense", "Unauthorized");
    }

    if (server.method() != HTTP_POST) {
        server.send(405, "text/plain", "Method Not Allowed");
        return;
    }

    String newPassword = server.arg("newPassword");
    
    // Walidacja hasła
    if (newPassword.length() == 0) {
        server.send(400, "text/plain", "Hasło nie może być puste");
        return;
    }
    
    if (newPassword.length() >= sizeof(config.webPassword)) {
        server.send(400, "text/plain", "Hasło jest za długie");
        return;
    }

    // Zapisz nowe hasło
    strlcpy(config.webPassword, newPassword.c_str(), sizeof(config.webPassword));
    
    // Zapisz konfigurację
    if (saveConfig()) {
        server.send(200, "text/plain", "Hasło zostało zmienione");
        // Opóźniony restart
        delay(500);
        ESP.restart();
    } else {
        server.send(500, "text/plain", "Błąd zapisu konfiguracji");
    }
}

void setupWebServer() {
    // Główna strona
    server.on("/", HTTP_GET, []() {
        if (!server.authenticate(config.webUser, config.webPassword)) {
            return server.requestAuthentication(BASIC_AUTH, "HydroSense", "Unauthorized");
        }
        handleRoot();
    });

    // Obsługa aktualizacji
    server.on("/update", HTTP_POST, []() {
        if (!server.authenticate(config.webUser, config.webPassword)) {
            return server.requestAuthentication(BASIC_AUTH, "HydroSense", "Unauthorized");
        }
        handleUpdateResult();
    }, []() {
        if (!server.authenticate(config.webUser, config.webPassword)) {
            return server.requestAuthentication(BASIC_AUTH, "HydroSense", "Unauthorized");
        }
        handleDoUpdate();
    });

    // Obsługa zapisywania konfiguracji
    server.on("/save", HTTP_POST, []() {
        if (!server.authenticate(config.webUser, config.webPassword)) {
            return server.requestAuthentication(BASIC_AUTH, "HydroSense", "Unauthorized");
        }
        handleSave();
    });

    // Obsługa zmiany hasła (NOWY ENDPOINT)
    server.on("/change-password", HTTP_POST, []() {
        if (!server.authenticate(config.webUser, config.webPassword)) {
            return server.requestAuthentication(BASIC_AUTH, "HydroSense", "Unauthorized");
        }
        handleChangePassword();
    });
    
    // Obsługa restartu
    server.on("/reboot", HTTP_POST, []() {
        if (!server.authenticate(config.webUser, config.webPassword)) {
            return server.requestAuthentication(BASIC_AUTH, "HydroSense", "Unauthorized");
        }
        server.send(200, "text/plain", "Restarting...");
        delay(1000);
        ESP.restart();
    });

    // Obsługa resetu do ustawień fabrycznych
    server.on("/factory-reset", HTTP_POST, []() {
        if (!server.authenticate(config.webUser, config.webPassword)) {
            return server.requestAuthentication(BASIC_AUTH, "HydroSense", "Unauthorized");
        }
        server.send(200, "text/plain", "Resetting to factory defaults...");
        delay(200);
        factoryReset();
    });
    
    // Dodanie obsługi błędu 404
    server.onNotFound([]() {
        if (!server.authenticate(config.webUser, config.webPassword)) {
            return server.requestAuthentication(BASIC_AUTH, "HydroSense", "Unauthorized");
        }
        server.send(404, "text/plain", "404: Not found");
    });

    server.begin();
}

// --- SETUP ---
void setup() {
    ESP.wdtEnable(WATCHDOG_TIMEOUT);  // Aktywacja watchdoga
    Serial.begin(115200);  // Inicjalizacja portu szeregowego
    DEBUG_PRINT("\nHydroSense start...");  // Komunikat startowy
    
    // Wczytaj konfigurację na początku
    if (!loadConfig()) {
        DEBUG_PRINT("Błąd wczytywania konfiguracji - używam ustawień domyślnych");
        setDefaultConfig();
        saveConfig();  // Zapisz domyślną konfigurację do EEPROM
    }
    
    setupPin();  // Ustawienia GPIO
    setupWiFi();  // Nawiązanie połączenia WiFi
    setupWebServer();  // Serwer www

    // Próba połączenia MQTT
    DEBUG_PRINT("Rozpoczynam połączenie MQTT...");
    connectMQTT();

    setupHA();  // Konfiguracja Home Assistant
   
    // Wczytaj konfigurację z EEPROM
    if (loadConfig()) {        
        status.soundEnabled = config.soundEnabled;  // Synchronizuj stan dźwięku z wczytanej konfiguracji
    }
    
    firstUpdateHA();  // Wyślij pierwsze odczyty do Home Assistant
    status.lastSoundAlert = millis();
    
    // Konfiguracja OTA
    ArduinoOTA.setHostname("HydroSense");  // Ustaw nazwę urządzenia
    ArduinoOTA.setPassword("hydrosense");  // Ustaw hasło dla OTA
    ArduinoOTA.begin();  // Uruchom OTA    
    
    DEBUG_PRINT("Setup zakończony pomyślnie!");

    // Powitanie
    if (status.soundEnabled) {  // Gdy jest włączony dzwięk
        //welcomeMelody();  //  to odegraj muzyczkę, że program poprawnie wystartował
    }  
        
    // Ustawienia fabryczne    
    // Czekaj 2 sekundy na wciśnięcie przycisku
    unsigned long startTime = millis();
    while(millis() - startTime < 2000) {
        if(digitalRead(PRZYCISK_PIN) == LOW) {
            playConfirmationSound();
            factoryReset();
        }
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
    webSocket.loop();

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
        if (!mqtt.begin(config.mqtt_server, 1883, config.mqtt_user, config.mqtt_password)) {
            DEBUG_PRINT(F("MQTT połączono ponownie!"));  // Wydrukuj komunikat debugowania
        }
    }
}
