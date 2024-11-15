// Biblioteki związane z obsługą komunikacji
#include <ESP8266WiFi.h>  // Obsługa WiFi dla ESP8266
#include <PubSubClient.h>  // MQTT - protokół komunikacji z serwerem MQTT
#include <ArduinoHA.h>  // Biblioteka do integracji z Home Assistant
#include <ArduinoOTA.h>  // Obsługa aktualizacji OTA (Over-The-Air)
#include <EEPROM.h>  // Obsługa pamięci EEPROM, wykorzystywana do przechowywania danych na stałe
#include <CRC32.h>  // Biblioteka CRC32 - używana do weryfikacji integralności danych
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Time.h>
#include <TimeLib.h>

// Definicje dla NTP
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

// Definicje interwałów czasowych
#define TIME_UPDATE_INTERVAL 3600000  // 1 godzina w milisekundach
#define STATS_UPDATE_INTERVAL 60000   // 1 minuta w milisekundach

// Zmienne do śledzenia czasu
unsigned long lastTimeUpdate = 0;
unsigned long lastStatsUpdate = 0;

// Strefa czasowa dla Polski (UTC+1 lub UTC+2 w czasie letnim)
const long utcOffsetInSeconds = 3600; // 3600 dla UTC+1, 7200 dla UTC+2

// Struktura konfiguracji
struct Config {
    uint8_t version;  // Wersja konfiguracji
    bool soundEnabled;  // Status dźwięku (włączony/wyłączony)
    char checksum;  // Suma kontrolna
};

// Stałe konfiguracyjne
const uint8_t CONFIG_VERSION = 1;
const int EEPROM_SIZE = sizeof(Config);

// Globalna instancja konfiguracji
Config config;

// Konfiguracja WiFi i MQTT
const char* WIFI_SSID = "pimowo";  // Nazwa sieci WiFi
const char* WIFI_PASSWORD = "ckH59LRZQzCDQFiUgj";  // Hasło do sieci WiFi
const char* MQTT_SERVER = "192.168.1.14";  // Adres IP serwera MQTT (Home Assistant)
const char* MQTT_USER = "hydrosense";  // Użytkownik MQTT
const char* MQTT_PASSWORD = "hydrosense";  // Hasło MQTT

// Konfiguracja pinów ESP8266
#define PIN_ULTRASONIC_TRIG D6 // Pin TRIG czujnika ultradźwiękowego
#define PIN_ULTRASONIC_ECHO D7 // Pin ECHO czujnika ultradźwiękowego

#define PIN_WATER_LEVEL D5 // Pin czujnika poziomu wody w akwarium
#define PIN_PUMP D1 // Pin sterowania pompą
#define PIN_BUZZER D2 // Pin buzzera do alarmów dźwiękowych
#define PIN_BUTTON D3 // Pin przycisku do kasowania alarmów

// Stałe czasowe (wszystkie wartości w milisekundach)
const unsigned long ULTRASONIC_TIMEOUT = 50;  // Timeout pomiaru w ms
const unsigned long MEASUREMENT_INTERVAL = 10000;  // Interwał między pomiarami w ms
const unsigned long WIFI_CHECK_INTERVAL = 5000;  // Czas między sprawdzeniami WiFi w ms
const unsigned long WATCHDOG_TIMEOUT = 8000;  // Timeout dla watchdoga w ms
const unsigned long PUMP_MAX_WORK_TIME = 300000;  // Maksymalny czas pracy pompy (5 minut)
const unsigned long PUMP_DELAY_TIME = 60000;  // Opóźnienie ponownego załączenia pompy (1 minuta)
const unsigned long SENSOR_READ_INTERVAL = 5000;  // Częstotliwość odczytu czujnika (5 sekund)
const unsigned long MQTT_RETRY_INTERVAL = 5000;  // Czas między próbami połączenia MQTT (5 sekund)
const unsigned long WIFI_RETRY_INTERVAL = 10000;  // Czas między próbami połączenia WiFi (10 sekund)
const unsigned long BUTTON_DEBOUNCE_TIME = 50;  // Czas debouncingu przycisku (50ms)
const unsigned long LONG_PRESS_TIME = 1000;  // Czas długiego naciśnięcia przycisku (1 sekunda)
const unsigned long SOUND_ALERT_INTERVAL = 60000; // 1 minuta w milisekundach

// Konfiguracja EEPROM
#define EEPROM_SIZE 512  // Rozmiar używanej pamięci EEPROM w bajtach

// Adresy w pamięci EEPROM
#define EEPROM_SOUND_STATE_ADDR 0  // Adres przechowywania stanu dźwięku (1 bajt)

// Konfiguracja zbiornika i pomiarów
const int DISTANCE_WHEN_FULL = 65.0;  // pełny zbiornik - mm
const int DISTANCE_WHEN_EMPTY = 510.0;  // pusty zbiornik - mm
const int DISTANCE_RESERVE = 450.0;  // dystans dla rezerwy - mm
const int HYSTERESIS = 10.0;  // histereza - mm
const int TANK_DIAMETER = 150;  // Średnica zbiornika - mm
const int MEASUREMENTS_COUNT = 3;  // liczba pomiarów do uśrednienia
const int PUMP_DELAY = 5;  // opóźnienie włączenia pompy - sekundy
const int PUMP_WORK_TIME = 60;  // czas pracy pompy - sekundy

float currentDistance = 0;
float volume = 0;  // Dodajemy też zmienną volume jako globalną
unsigned long lastDebounceTime = 0;  // ostatni czas zmiany stanu przycisku

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

HASensor sensorDailyPumpRuns("daily_pump_runs", true);
HASensor sensorDailyPumpTime("daily_pump_time", true);
HASensor sensorDailyWaterUsed("daily_water_used", true);

// Status systemu
struct {
bool isPumpActive = false; // Status pracy pompy
    unsigned long pumpStartTime = 0; // Czas startu pompy
    bool isPumpDelayActive = false; // Status opóźnienia przed startem pompy
    unsigned long pumpDelayStartTime = 0; // Czas rozpoczęcia opóźnienia pompy
    bool pumpSafetyLock = false; // Blokada bezpieczeństwa pompy
    bool waterAlarmActive = false; // Alarm braku wody w zbiorniku dolewki
    bool waterReserveActive = false; // Status rezerwy wody w zbiorniku
    bool soundEnabled;// = false; // Status włączenia dźwięku alarmu
    bool isServiceMode = false; // Status trybu serwisowego
    unsigned long lastSuccessfulMeasurement = 0; // Czas ostatniego udanego pomiaru
    unsigned long lastSoundAlert = 0;  //
} status;

// Struktura dla obsługi przycisku
struct ButtonState {
    bool lastState; // Poprzedni stan przycisku
    unsigned long pressedTime = 0; // Czas wciśnięcia przycisku
    unsigned long releasedTime = 0; // Czas puszczenia przycisku
    bool isLongPressHandled = false; // Flaga obsłużonego długiego naciśnięcia
    bool isInitialized = false; 
} buttonState;

// Obliczanie sumy kontrolnej
char calculateChecksum(const Config& cfg) {
    const byte* p = (const byte*)(&cfg);
    char sum = 0;
    for (int i = 0; i < sizeof(Config) - 1; i++) {
        sum ^= p[i];
    }
    return sum;
}

// Definicje dla różnych typów alarmów
enum AlarmType {
    ALARM_PUMP_START,
    ALARM_PUMP_STOP,
    ALARM_WATER_LOW,
    ALARM_WATER_HIGH
};

// Deklaracja funkcji (dodaj to po enum AlarmType)
void playAlarm(AlarmType type);  // Deklaracja prototypu funkcji

// Struktura dla dźwięków alarmowych
struct AlarmTone {
    uint16_t frequency;        // Częstotliwość dźwięku
    uint16_t duration;         // Czas trwania
    uint8_t repeats;          // Liczba powtórzeń
    uint16_t pauseDuration;   // Przerwa między powtórzeniami
};

// Konfiguracja dźwięków dla różnych alarmów
const AlarmTone alarmTones[] = {
    {2000, 1000, 3, 500},     // ALARM_WATER_LOW
    {3000, 500, 5, 200},      // ALARM_PUMP_SAFETY
    {1500, 200, 2, 300},      // ALARM_WATER_RESERVE
    {800, 100, 1, 0},         // ALARM_PUMP_START
    {600, 200, 1, 0}          // ALARM_PUMP_STOP
};

// Struktura dla statystyk
struct PumpStatistics {
    // Dzienne statystyki
    uint16_t dailyPumpRuns;
    uint32_t dailyPumpWorkTime;    // w sekundach
    float dailyWaterUsed;          // w litrach
    
    // Tygodniowe statystyki
    uint16_t weeklyPumpRuns;
    uint32_t weeklyPumpWorkTime;
    float weeklyWaterUsed;
    
    // Miesięczne statystyki
    uint16_t monthlyPumpRuns;
    uint32_t monthlyPumpWorkTime;
    float monthlyWaterUsed;
    
    // Całkowite statystyki
    uint32_t totalPumpRuns;
    uint32_t totalPumpWorkTime;
    float totalWaterUsed;
    
    // Znaczniki czasowe ostatnich resetów
    time_t lastDailyReset;
    time_t lastWeeklyReset;
    time_t lastMonthlyReset;
};

// Globalna instancja statystyk
PumpStatistics pumpStats = {0};

// Zmienne do śledzenia stanu pompy
unsigned long pumpStartTime = 0;      // Czas startu pompy
float waterLevelBeforePump = 0;       // Poziom wody przed startem pompy

// Zapis konfiguracji
void saveConfig() {
    config.version = CONFIG_VERSION;
    config.checksum = calculateChecksum(config);
    
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.put(0, config);
    bool success = EEPROM.commit();
    
    if (success) {
        Serial.printf("Konfiguracja zapisana. Stan dźwięku: %s\n",
                     config.soundEnabled ? "WŁĄCZONY" : "WYŁĄCZONY");
    } else {
        Serial.println(F("Błąd zapisu konfiguracji!"));
    }
}

// Wczytywanie konfiguracji
bool loadConfig() {
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.get(0, config);
    
    if (config.version != CONFIG_VERSION) {
        Serial.println(F("Niekompatybilna wersja konfiguracji"));
        setDefaultConfig();
        return false;
    }
    
    char calculatedChecksum = calculateChecksum(config);
    if (config.checksum != calculatedChecksum) {
        Serial.println(F("Błąd sumy kontrolnej"));
        setDefaultConfig();
        return false;
    }
    
    // Synchronizuj stan po wczytaniu
    status.soundEnabled = config.soundEnabled;
    switchSound.setState(config.soundEnabled, true);  // force update
    
    Serial.printf("Konfiguracja wczytana. Stan dźwięku: %s\n", 
                 config.soundEnabled ? "WŁĄCZONY" : "WYŁĄCZONY");
    return true;
}

// Ustawienia domyślne
void setDefaultConfig() {
    config.version = CONFIG_VERSION;
    config.soundEnabled = true;  // Domyślnie dźwięk włączony
    
    saveConfig();
    Serial.println(F("Utworzono domyślną konfigurację"));
}

void resetDailyStatistics() {
    pumpStats.dailyPumpRuns = 0;
    pumpStats.dailyPumpWorkTime = 0;
    pumpStats.dailyWaterUsed = 0;
    pumpStats.lastDailyReset = timeClient.getEpochTime();
}

void resetWeeklyStatistics() {
    pumpStats.weeklyPumpRuns = 0;
    pumpStats.weeklyPumpWorkTime = 0;
    pumpStats.weeklyWaterUsed = 0;
    pumpStats.lastWeeklyReset = timeClient.getEpochTime();
}

void resetMonthlyStatistics() {
    pumpStats.monthlyPumpRuns = 0;
    pumpStats.monthlyPumpWorkTime = 0;
    pumpStats.monthlyWaterUsed = 0;
    pumpStats.lastMonthlyReset = timeClient.getEpochTime();
}

void updateHAStatistics() {
    char value[16];
    
    // Konwersja liczby uruchomień pompy na string
    itoa(pumpStats.dailyPumpRuns, value, 10);
    sensorDailyPumpRuns.setValue(value);
    
    // Konwersja czasu pracy pompy na string (w minutach)
    itoa(pumpStats.dailyPumpWorkTime / 60, value, 10);
    sensorDailyPumpTime.setValue(value);
    
    // Konwersja zużycia wody na string
    dtostrf(pumpStats.dailyWaterUsed, 4, 2, value);
    sensorDailyWaterUsed.setValue(value);
}

// Deklaracja getCurrentWaterLevel
float getCurrentWaterLevel() {
    int distance = measureDistance();
    return calculateWaterLevel(distance);
}

// Funkcja sprawdzająca resety statystyk
void checkStatisticsReset() {
    time_t now = timeClient.getEpochTime();
    struct tm* timeinfo = localtime(&now);
    
    // Reset dzienny o północy
    if (timeinfo->tm_hour == 0 && timeinfo->tm_min == 0) {
        if (now - pumpStats.lastDailyReset >= 86400) { // 24 godziny
            resetDailyStatistics();
        }
    }
    
    // Reset tygodniowy w niedzielę o północy
    if (timeinfo->tm_wday == 0 && timeinfo->tm_hour == 0 && timeinfo->tm_min == 0) {
        if (now - pumpStats.lastWeeklyReset >= 604800) { // 7 dni
            resetWeeklyStatistics();
        }
    }
    
    // Reset miesięczny pierwszego dnia miesiąca o północy
    if (timeinfo->tm_mday == 1 && timeinfo->tm_hour == 0 && timeinfo->tm_min == 0) {
        if (now - pumpStats.lastMonthlyReset >= 2592000) { // 30 dni
            resetMonthlyStatistics();
        }
    }
}

// Funkcja do aktualizacji statystyk przy włączeniu pompy
void onPumpStart() {
    pumpStats.dailyPumpRuns++;
    pumpStats.weeklyPumpRuns++;
    pumpStats.monthlyPumpRuns++;
    pumpStats.totalPumpRuns++;
    
    pumpStartTime = millis();
    waterLevelBeforePump = getCurrentWaterLevel();
    
    // Sygnał dźwiękowy startu pompy
    playAlarm(ALARM_PUMP_START);
}

// Funkcja do aktualizacji statystyk przy wyłączeniu pompy
void onPumpStop() {
    unsigned long pumpWorkTime = (millis() - pumpStartTime) / 1000; // czas w sekundach
    float waterUsed = calculateWaterUsed(waterLevelBeforePump, getCurrentWaterLevel());
    
    pumpStats.dailyPumpWorkTime += pumpWorkTime;
    pumpStats.weeklyPumpWorkTime += pumpWorkTime;
    pumpStats.monthlyPumpWorkTime += pumpWorkTime;
    pumpStats.totalPumpWorkTime += pumpWorkTime;
    
    pumpStats.dailyWaterUsed += waterUsed;
    pumpStats.weeklyWaterUsed += waterUsed;
    pumpStats.monthlyWaterUsed += waterUsed;
    pumpStats.totalWaterUsed += waterUsed;
    
    // Sygnał dźwiękowy zatrzymania pompy
    playAlarm(ALARM_PUMP_STOP);
}

// Funkcja do obliczania zużytej wody
float calculateWaterUsed(float beforeLevel, float afterLevel) {
    // Zakładając, że poziomy są w procentach i znamy objętość zbiornika
    const float CONTAINER_VOLUME = 20.0; // Objętość zbiornika w litrach - do dostosowania
    return (beforeLevel - afterLevel) * CONTAINER_VOLUME / 100.0;
}

void playAlarm(AlarmType type) {
    // Sprawdź czy dźwięk jest włączony
    if (!config.soundEnabled) {
        return; // Jeśli dźwięk jest wyłączony, zakończ funkcję
    }

    // Parametry dźwięku dla różnych typów alarmów
    switch (type) {
        case ALARM_PUMP_START:
            tone(PIN_BUZZER, 2000, 100);  // Wysoki dźwięk, krótki
            break;
            
        case ALARM_PUMP_STOP:
            tone(PIN_BUZZER, 1000, 200);  // Niższy dźwięk, dłuższy
            break;
            
        case ALARM_WATER_LOW:
            // Dwa krótkie sygnały
            tone(PIN_BUZZER, 2500, 100);
            delay(150);
            tone(PIN_BUZZER, 2500, 100);
            break;
            
        case ALARM_WATER_HIGH:
            // Jeden długi sygnał
            tone(PIN_BUZZER, 3000, 500);
            break;
    }
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
    if (!mqtt.begin(MQTT_SERVER, 1883, MQTT_USER, MQTT_PASSWORD)) {
        Serial.println("\nBŁĄD POŁĄCZENIA MQTT!");
        return false;
    }
    
    Serial.println("MQTT połączono pomyślnie!");
    return true;
}

// Funkcja konfigurująca Home Assistant
void setupHA() {
 // Konfiguracja urządzenia dla Home Assistant
    device.setName("HydroSense");                  // Nazwa urządzenia
    device.setModel("HS ESP8266");                 // Model urządzenia
    device.setManufacturer("PMW");                 // Producent
    device.setSoftwareVersion("15.11.24");         // Wersja oprogramowania

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

    // Dodanie nowych sensorów
    sensorDailyPumpRuns.setName("Dzisiejsze uruchomienia pompy");
    sensorDailyPumpRuns.setIcon("mdi:water-pump");
    sensorDailyPumpRuns.setUnitOfMeasurement("razy");
    
    sensorDailyPumpTime.setName("Dzisiejszy czas pracy pompy");
    sensorDailyPumpTime.setIcon("mdi:timer");
    sensorDailyPumpTime.setUnitOfMeasurement("min");
    
    sensorDailyWaterUsed.setName("Dzisiejsze zużycie wody");
    sensorDailyWaterUsed.setIcon("mdi:water");
    sensorDailyWaterUsed.setUnitOfMeasurement("L");
}

void setupPin() {
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
}

void welcomeMelody() {
    // Prosta melodia powitalna
    tone(PIN_BUZZER, 1397, 100);  // F6
    delay(150);
    tone(PIN_BUZZER, 1568, 100);  // G6
    delay(150);
    tone(PIN_BUZZER, 1760, 150);  // A6
    delay(200);
}

void firstUpdateHA() {
    // Wykonaj pierwszy pomiar i ustaw stany
    float initialDistance = measureDistance();
    
    // Ustaw początkowe stany na podstawie pomiaru
    status.waterAlarmActive = (initialDistance >= DISTANCE_WHEN_EMPTY);
    status.waterReserveActive = (initialDistance >= DISTANCE_RESERVE);
    
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

// Funkcja obliczająca poziom wody w zbiorniku dolewki w procentach
//  
// @param distance - zmierzona odległość od czujnika do lustra wody w mm
// @return int - poziom wody w procentach (0-100%)
// Wzór: ((EMPTY - distance) / (EMPTY - FULL)) * 100
int calculateWaterLevel(int distance) {
    // Ograniczenie wartości do zakresu pomiarowego
    if (distance < DISTANCE_WHEN_FULL) distance = DISTANCE_WHEN_FULL;  // Nie mniej niż przy pełnym
    if (distance > DISTANCE_WHEN_EMPTY) distance = DISTANCE_WHEN_EMPTY;  // Nie więcej niż przy pustym
    
    // Obliczenie procentowe poziomu wody
    float percentage = (float)(DISTANCE_WHEN_EMPTY - distance) /  // Różnica: pusty - aktualny
                      (float)(DISTANCE_WHEN_EMPTY - DISTANCE_WHEN_FULL) *  // Różnica: pusty - pełny
                      100.0;  // Przeliczenie na procenty
    
    return (int)percentage;  // Zwrot wartości całkowitej
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
            digitalWrite(PIN_PUMP, LOW);  // Wyłączenie pompy
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
// void handleButton() {
void handleButton() {
    static unsigned long lastDebounceTime = 0;
    static bool lastReading = HIGH;
    const unsigned long DEBOUNCE_DELAY = 50;  // 50ms debounce

    bool reading = digitalRead(PIN_BUTTON);

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
                        digitalWrite(PIN_PUMP, LOW);  // Wyłącz pompę
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
                Serial.println("Alarm pompy skasowany");
            }
        }
    }
    
    lastReading = reading;  // Zapisz ostatni odczyt dla następnego porównania
    yield();  // Oddaj sterowanie systemowi
}

void shortAlertBeep() {
    tone(PIN_BUZZER, 2000, 100);  // Częstotliwość 2000Hz, czas 100ms
}

void checkAlarmConditions() {
    // Sprawdź czy minęła minuta od ostatniego alarmu
    if (millis() - status.lastSoundAlert >= SOUND_ALERT_INTERVAL) {
        // Sprawdź czy dźwięk jest włączony i czy występuje któryś z alarmów
        if (status.soundEnabled && (status.waterAlarmActive || status.pumpSafetyLock)) {
            shortAlertBeep();
            status.lastSoundAlert = millis();
            
            // Debug info
            Serial.println(F("Alarm dźwiękowy - przyczyna:"));
            if (status.waterAlarmActive) Serial.println(F("- Brak wody"));
            if (status.pumpSafetyLock) Serial.println(F("- Alarm pompy"));
        }
    }
}

// Funkcja wykonuje pomiar odległości za pomocą czujnika ultradźwiękowego HC-SR04
// Zwraca:
// - zmierzoną odległość w milimetrach
// - (-1) w przypadku błędu lub przekroczenia czasu odpowiedzi
int measureDistance() {
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
            Serial.println("Timeout - brak początku echa");
            return -1;  // Błąd - brak odpowiedzi od czujnika
        }
    }

    // Pomiar czasu początku echa (w mikrosekundach)
    // micros() zapewnia dokładniejszy pomiar niż millis()
    unsigned long echoStartTime = micros();

    // Oczekiwanie na koniec sygnału echo
    // Timeout 20ms - teoretyczny max dla 3.4m to około 20ms
    while (digitalRead(PIN_ULTRASONIC_ECHO) == HIGH) {
        if (micros() - echoStartTime > 20000) {
            Serial.println("Timeout - zbyt długie echo");
            return -1;  // Błąd - echo trwa zbyt długo
        }
    }

    // Obliczenie czasu trwania echa (w mikrosekundach)
    unsigned long duration = micros() - echoStartTime;

    // Konwersja czasu na odległość
    // Wzór: distance = (czas * prędkość dźwięku) / 2
    // - czas w mikrosekundach
    // - prędkość dźwięku = 343 m/s = 0.343 mm/µs
    // - dzielimy przez 2 bo dźwięk pokonuje drogę w obie strony
    int distance = (duration * 343) / 2000;

    // Walidacja wyniku
    // - min 20mm - minimalna dokładna odległość dla HC-SR04
    // - max 1500mm - maksymalna użyteczna odległość dla zbiornika
    if (distance >= 20 && distance <= 1500) {
        return distance;
    }

    return -1;  // Wynik poza zakresem pomiarowym
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

// Kontrola pompy - funkcja zarządzająca pracą pompy i jej zabezpieczeniami
void updatePump() {
    // Zabezpieczenie przed przepełnieniem licznika millis() (po około 50 dniach)
    // Jeśli millis() się przepełni, aktualizujemy czasy startowe
    if (millis() < status.pumpStartTime) {
        status.pumpStartTime = millis();
    }
    
    if (millis() < status.pumpDelayStartTime) {
        status.pumpDelayStartTime = millis();
    }
    
    // Odczyt stanu czujnika poziomu wody w akwarium
    // LOW = brak wody - należy uzupełnić
    // HIGH = woda obecna - stan normalny
    bool waterPresent = (digitalRead(PIN_WATER_LEVEL) == LOW);
    sensorWater.setValue(waterPresent ? "ON" : "OFF");
    
    // --- ZABEZPIECZENIE 1: Tryb serwisowy ---
    // Jeśli aktywny jest tryb serwisowy, wyłącz pompę i zakończ
    if (status.isServiceMode) {
        if (status.isPumpActive) {
            digitalWrite(PIN_PUMP, LOW);  // Wyłącz pompę
            status.isPumpActive = false;  // Oznacz jako nieaktywną
            status.pumpStartTime = 0;  // Zeruj czas startu
            sensorPump.setValue("OFF");  // Aktualizuj status w HA
        }
        return;
    }

    // --- ZABEZPIECZENIE 2: Maksymalny czas pracy ---
    // Sprawdź czy pompa nie przekroczyła maksymalnego czasu pracy
    if (status.isPumpActive && (millis() - status.pumpStartTime > PUMP_WORK_TIME * 1000)) {
        digitalWrite(PIN_PUMP, LOW);  // Wyłącz pompę
        status.isPumpActive = false;  // Oznacz jako nieaktywną
        status.pumpStartTime = 0;  // Zeruj czas startu
        sensorPump.setValue("OFF");  // Aktualizuj status w HA
        status.pumpSafetyLock = true;  // Aktywuj blokadę bezpieczeństwa
        switchPump.setState(true);  // Aktywuj przełącznik alarmu w HA
        Serial.println("ALARM: Pompa pracowała za długo - aktywowano blokadę bezpieczeństwa!");
        return;
    }
    
    // --- ZABEZPIECZENIE 3: Blokady bezpieczeństwa ---
    // Sprawdź czy nie jest aktywna blokada bezpieczeństwa lub alarm braku wody
    if (status.pumpSafetyLock || status.waterAlarmActive) {
        if (status.isPumpActive) {
            digitalWrite(PIN_PUMP, LOW);  // Wyłącz pompę
            status.isPumpActive = false;  // Oznacz jako nieaktywną
            status.pumpStartTime = 0;  // Zeruj czas startu
            sensorPump.setValue("OFF");  // Aktualizuj status w HA
        }
        return;
    }
    
     // --- ZABEZPIECZENIE 4: Ochrona przed przepełnieniem ---
    // Jeśli woda osiągnęła poziom, a pompa pracuje, 
    // wyłącz ją aby zapobiec przelaniu
    if (!waterPresent && status.isPumpActive) {
        digitalWrite(PIN_PUMP, LOW);  // Wyłącz pompę
        status.isPumpActive = false;  // Oznacz jako nieaktywną
        status.pumpStartTime = 0;  // Zeruj czas startu
        status.isPumpDelayActive = false;  // Wyłącz opóźnienie
        sensorPump.setValue("OFF");  // Aktualizuj status w HA
        return;
    }
    
    // --- LOGIKA WŁĄCZANIA POMPY ---
    // Jeśli brakuje wody w akwarium (czujnik niezanurzony - LOW) 
    // i pompa nie pracuje oraz nie trwa odliczanie opóźnienia,
    // rozpocznij procedurę opóźnionego startu pompy
    if (waterPresent && !status.isPumpActive && !status.isPumpDelayActive) {
        status.isPumpDelayActive = true;  // Aktywuj opóźnienie
        status.pumpDelayStartTime = millis();  // Zapisz czas rozpoczęcia opóźnienia
        return;
    }
    
    // Po upływie opóźnienia, włącz pompę
    if (status.isPumpDelayActive && !status.isPumpActive) {
        if (millis() - status.pumpDelayStartTime >= (PUMP_DELAY * 1000)) {
            digitalWrite(PIN_PUMP, HIGH);  // Włącz pompę
            status.isPumpActive = true;  // Oznacz jako aktywną
            status.pumpStartTime = millis();  // Zapisz czas startu
            status.isPumpDelayActive = false;  // Wyłącz opóźnienie
            sensorPump.setValue("ON");  // Aktualizuj status w HA
        }
    }
}

void updateAlarmStates(float currentDistance) {
    // --- Obsługa alarmu krytycznie niskiego poziomu wody ---
    // Włącz alarm jeśli:
    // - odległość jest większa lub równa max (zbiornik pusty)
    // - alarm nie jest jeszcze aktywny
    if (currentDistance >= DISTANCE_WHEN_EMPTY && !status.waterAlarmActive) {
        status.waterAlarmActive = true;
        sensorAlarm.setValue("ON");               
        Serial.println("Brak wody ON");
    } 
    // Wyłącz alarm jeśli:
    // - odległość spadła poniżej progu wyłączenia (z histerezą)
    // - alarm jest aktywny
    else if (currentDistance < (DISTANCE_WHEN_EMPTY - HYSTERESIS) && status.waterAlarmActive) {
        status.waterAlarmActive = false;
        sensorAlarm.setValue("OFF");
        Serial.println("Brak wody OFF");
    }

    // --- Obsługa ostrzeżenia o rezerwie wody ---
    // Włącz ostrzeżenie o rezerwie jeśli:
    // - odległość osiągnęła próg rezerwy
    // - ostrzeżenie nie jest jeszcze aktywne
    if (currentDistance >= DISTANCE_RESERVE && !status.waterReserveActive) {
        status.waterReserveActive = true;
        sensorReserve.setValue("ON");
        Serial.println("Rezerwa ON");
    } 
    // Wyłącz ostrzeżenie o rezerwie jeśli:
    // - odległość spadła poniżej progu rezerwy (z histerezą)
    // - ostrzeżenie jest aktywne
    else if (currentDistance < (DISTANCE_RESERVE - HYSTERESIS) && status.waterReserveActive) {
        status.waterReserveActive = false;
        sensorReserve.setValue("OFF");
        Serial.println("Rezerwa OFF");
    }
}

void updateWaterLevel() {
    currentDistance = measureDistance();
    if (currentDistance < 0) return; // błąd pomiaru
    
    // Aktualizacja stanów alarmowych
    updateAlarmStates(currentDistance);

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
        
    // Debug info tylko gdy wartości się zmieniły (conajmniej 5mm)
    static float lastReportedDistance = 0;
    if (abs(currentDistance - lastReportedDistance) > 5) {
        Serial.printf("Poziom: %.1f mm, Obj: %.1f L\n", currentDistance, volume);
        lastReportedDistance = currentDistance;
    }
}

// Funkcja inicjalizacyjna - wykonywana jednorazowo przy starcie urządzenia
void setup() {
    ESP.wdtEnable(WATCHDOG_TIMEOUT);  // Aktywacja watchdoga
    Serial.begin(115200);  // Inicjalizacja portu szeregowego
    Serial.println("\nStarting HydroSense...");  // Komunikat startowy
    
    // Wczytaj konfigurację
    if (!loadConfig()) {
        Serial.println(F("Tworzenie nowej konfiguracji..."));
        setDefaultConfig();
    }
    
    // Zastosuj wczytane ustawienia
    status.soundEnabled = config.soundEnabled;
    
    setupPin();
    
    // Nawiązanie połączenia WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {        // Oczekiwanie na połączenie
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nPołączono z WiFi");

    // Próba połączenia MQTT
    Serial.println("Rozpoczynam połączenie MQTT...");
    connectMQTT();

    setupHA();
   
    // Wczytaj konfigurację z EEPROM
    if (loadConfig()) {        
        status.soundEnabled = config.soundEnabled;  // Synchronizuj stan dźwięku z wczytanej konfiguracji
        switchSound.setState(config.soundEnabled);  // Aktualizuj stan w HA
    }
    
    firstUpdateHA();
    status.lastSoundAlert = millis();
    
    // Konfiguracja OTA (Over-The-Air) dla aktualizacji oprogramowania
    ArduinoOTA.setHostname("HydroSense");  // Ustaw nazwę urządzenia
    ArduinoOTA.setPassword("hydrosense");  // Ustaw hasło dla OTA
    ArduinoOTA.begin();  // Uruchom OTA

    // Inicjalizacja NTP
    timeClient.begin();
    timeClient.setTimeOffset(utcOffsetInSeconds);
    
    // Pierwsze pobranie czasu
    while(!timeClient.update()) {
        timeClient.forceUpdate();
        delay(500);
    }
    
    // Reset statystyk
    resetDailyStatistics();
    resetWeeklyStatistics();
    resetMonthlyStatistics();

    Serial.println("Setup zakończony pomyślnie!");
    
    if (status.soundEnabled) {
        //welcomeMelody();
    }
}

void loop() {
    unsigned long currentMillis = millis();
    static unsigned long lastMQTTRetry = 0;
    static unsigned long lastMeasurement = 0;

    // KRYTYCZNE OPERACJE SYSTEMOWE
    
    // Zabezpieczenie przed zawieszeniem systemu
    ESP.wdtFeed();      // Resetowanie licznika watchdog
    yield();            // Obsługa krytycznych zadań systemowych ESP8266
    
    // System aktualizacji bezprzewodowej
    ArduinoOTA.handle(); // Nasłuchiwanie żądań aktualizacji OTA

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
            Serial.println("\nBrak połączenia MQTT - próba reconnect...");
            if (mqtt.begin(MQTT_SERVER, 1883, MQTT_USER, MQTT_PASSWORD)) {
                Serial.println("MQTT połączono ponownie!");
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
    if (millis() - lastMeasurement >= MEASUREMENT_INTERVAL) {
        updateWaterLevel();  // Pomiar i aktualizacja stanu wody
        lastMeasurement = millis();
    }

    // STATYSTYKI I ZARZĄDZANIE CZASEM
    
    // Aktualizacja czasu i resetowanie statystyk (co godzinę)
    if (currentMillis - lastTimeUpdate >= 3600000) {
        if (timeClient.update()) {
            lastTimeUpdate = currentMillis;
            checkStatisticsReset();
        }
    }
    
    // Aktualizacja statystyk Home Assistant (co minutę)
    if (currentMillis - lastStatsUpdate >= 60000) {
        updateHAStatistics();
        lastStatsUpdate = currentMillis;
    }
}
