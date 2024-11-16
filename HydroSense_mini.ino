// --- Biblioteki

#include <Arduino.h>
#include <ArduinoHA.h>     // Biblioteka do integracji z Home Assistant
#include <ArduinoOTA.h>    // Obsługa aktualizacji OTA (Over-The-Air)
#include <ESP8266WiFi.h>   // Obsługa WiFi dla ESP8266
#include <PubSubClient.h>  // MQTT - protokół komunikacji z serwerem MQTT
#include <EEPROM.h>        // Obsługa pamięci EEPROM, używana do przechowywania danych na stałe
#include <CRC32.h>         // Biblioteka CRC32 - używana do weryfikacji integralności danych
//#include <NTPClient.h>     // Obsługa klienta NTP (Network Time Protocol)
#include <WiFiUdp.h>       // Obsługa komunikacji UDP
#include <Time.h>          // Obsługa funkcji związanych z czasem
#include <TimeLib.h>       // Dodatkowe funkcje związane z czasem
#include <sys/time.h>      // Dla funkcji czasowych

// --- Definicje stałych i zmiennych globalnych

// Definicje dla NTP (Network Time Protocol)
//WiFiUDP ntpUDP;
//NTPClient timeClient(ntpUDP, "pool.ntp.org");
// Zasady dla czasu środkowoeuropejskiego (CET/CEST)
// Czas letni (CEST) = UTC + 2
// Czas zimowy (CET) = UTC + 1
//TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 2, 120};  // Czas letni (UTC + 2)
//TimeChangeRule CET = {"CET", Last, Sun, Oct, 3, 60};     // Czas zimowy (UTC + 1)
//Timezone CE(CEST, CET);

// Definicje interwałów czasowych
#define TIME_UPDATE_INTERVAL 3600000  // Interwał aktualizacji czasu (1 godzina w milisekundach)
#define STATS_UPDATE_INTERVAL 60000   // Interwał aktualizacji statystyk (1 minuta w milisekundach)

// Zmienne do śledzenia czasu
unsigned long lastTimeUpdate = 0;   // Ostatnia aktualizacja czasu
unsigned long lastStatsUpdate = 0;  // Ostatnia aktualizacja statystyk

// Strefa czasowa dla Polski (UTC+1 lub UTC+2 w czasie letnim)
const long utcOffsetInSeconds = 3600; // Przesunięcie czasowe w sekundach (3600 dla UTC+1)                  

// Konfiguracja WiFi i MQTT
const char* WIFI_SSID = "pimowo";                  // Nazwa sieci WiFi
const char* WIFI_PASSWORD = "ckH59LRZQzCDQFiUgj";  // Hasło do sieci WiFi
const char* MQTT_SERVER = "192.168.1.14";          // Adres IP serwera MQTT (Home Assistant)
const char* MQTT_USER = "hydrosense";              // Użytkownik MQTT
const char* MQTT_PASSWORD = "hydrosense";          // Hasło MQTT

// Konfiguracja pinów ESP8266
#define PIN_ULTRASONIC_TRIG D6  // Pin TRIG czujnika ultradźwiękowego
#define PIN_ULTRASONIC_ECHO D7  // Pin ECHO czujnika ultradźwiękowego

#define PIN_WATER_LEVEL D5      // Pin czujnika poziomu wody w akwarium
#define POMPA_PIN D1             // Pin sterowania pompą
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

// Konfiguracja zbiornika i pomiarów
const int ODLEGLOSC_PELNY = 65.0;    // Dystans dla pełnego zbiornika (w mm)
const int ODLEGLOSC_PUSTY = 510.0;  // Dystans dla pustego zbiornika (w mm)
const int REZERWA_ODLEGLOSC = 450.0;     // Dystans dla rezerwy (w mm)
const int HYSTERESIS = 10.0;            // Histereza (w mm)
const int SREDNICA_ZBIORNIKA = 150;          // Średnica zbiornika (w mm)
const int MEASUREMENTS_COUNT = 3;       // Liczba pomiarów do uśrednienia
const int PUMP_DELAY = 5;               // Opóźnienie włączenia pompy (w sekundach)
const int PUMP_WORK_TIME = 60;          // Czas pracy pompy (w sekundach)

float aktualnaOdleglosc = 0;              // Aktualny dystans
//float objetosc = 0;                       // Objętość wody
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

// Dzienne statystyki
HASensor sensorDailyPumpRuns("daily_pump_runs", true);    // Dzienne uruchomienia pompy
HASensor sensorDailyPumpTime("daily_pump_time", true);    // Dzienne czas pracy pompy
HASensor sensorDailyWaterUsed("daily_water_used", true);  // Dzienne zużycie wody

// Tygodniowe statystyki
HASensor sensorWeeklyPumpRuns("weekly_pump_runs", true);    // Tygodniowe uruchomienia pompy
HASensor sensorWeeklyPumpTime("weekly_pump_time", true);    // Tygodniowy czas pracy pompy
HASensor sensorWeeklyWaterUsed("weekly_water_used", true);  // Tygodniowe zużycie wody

// Miesięczne statystyki
HASensor sensorMonthlyPumpRuns("monthly_pump_runs", true);    // Miesięczne uruchomienia pompy
HASensor sensorMonthlyPumpTime("monthly_pump_time", true);    // Miesięczny czas pracy pompy
HASensor sensorMonthlyWaterUsed("monthly_water_used", true);  // Miesięczne zużycie wody

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
    uint8_t version;        // Wersja konfiguracji
    bool soundEnabled;      // Status dźwięku (włączony/wyłączony)
    char checksum;          // Suma kontrolna
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

// Konfiguracja dźwięków dla różnych alarmów
const AlarmTone alarmTones[] = {
    {2000, 1000, 3, 500},     // ALARM_WATER_LOW
    {3000, 500, 5, 200},      // ALARM_PUMP_SAFETY
    {1500, 200, 2, 300},      // ALARM_WATER_RESERVE
    {800, 100, 1, 0},         // ALARM_PUMP_START
    {600, 200, 1, 0}          // ALARM_PUMP_STOP
};

// Definicje dla różnych typów alarmów
enum AlarmType {
    ALARM_PUMP_START,
    ALARM_PUMP_STOP,
    ALARM_WATER_LOW,
    ALARM_WATER_HIGH
};

// Struktura dla statystyk
struct PumpStatistics {
    // Dzienne statystyki
    uint8_t dailyPumpRuns;      
    uint16_t dailyPumpWorkTime; 
    uint8_t dailyWaterUsed;     
    
    // Tygodniowe statystyki
    uint8_t weeklyPumpRuns;     
    uint16_t weeklyPumpWorkTime;
    uint8_t weeklyWaterUsed;    
    
    // Miesięczne statystyki
    uint8_t monthlyPumpRuns;    
    uint16_t monthlyPumpWorkTime;
    uint8_t monthlyWaterUsed;   
    
    // Znaczniki czasu resetów
    time_t lastDailyReset;
    time_t lastWeeklyReset;
    time_t lastMonthlyReset;
};
PumpStatistics pumpStats = {0};

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
    
    saveConfig();
    Serial.println(F("Utworzono domyślną konfigurację"));
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

// Zapis konfiguracji
// void saveConfig() {
//     config.version = CONFIG_VERSION;
//     config.checksum = calculateChecksum(config);
    
//     EEPROM.begin(EEPROM_SIZE);
//     EEPROM.put(0, config);
//     bool success = EEPROM.commit();
    
//     if (success) {
//         Serial.printf("Konfiguracja zapisana. Stan dźwięku: %s\n",
//                      config.soundEnabled ? "WŁĄCZONY" : "WYŁĄCZONY");
//     } else {
//         Serial.println(F("Błąd zapisu konfiguracji!"));
//     }
// }

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

// Uruchomienie alarmu
void playAlarm(AlarmType type) {
    // Sprawdź czy dźwięk jest włączony
    if (!config.soundEnabled) {
        return; // Jeśli dźwięk jest wyłączony, zakończ funkcję
    }

    // Parametry dźwięku dla różnych typów alarmów
    switch (type) {
        case ALARM_PUMP_START:
            tone(BUZZER_PIN, 2000, 100);  // Wysoki dźwięk, krótki
            break;
            
        case ALARM_PUMP_STOP:
            tone(BUZZER_PIN, 1000, 200);  // Niższy dźwięk, dłuższy
            break;
            
        case ALARM_WATER_LOW:
            // Dwa krótkie sygnały
            tone(BUZZER_PIN, 2500, 100);
            delay(150);
            tone(BUZZER_PIN, 2500, 100);
            break;
            
        case ALARM_WATER_HIGH:
            // Jeden długi sygnał
            tone(BUZZER_PIN, 3000, 500);
            break;
    }
}

// Krótki sygnał dźwiękowy
void playShortWarningSound() {
    tone(BUZZER_PIN, 2000, 100); // Krótki sygnał dźwiękowy (częstotliwość 2000Hz, czas 100ms)
}

// Sprawdzenie warunków alarmowych
void checkAlarmConditions() {
    static unsigned long lastAlarmTime = 0;  // Zmienna statyczna do przechowywania czasu ostatniego alarmu
    unsigned long currentTime = millis();  // Aktualny czas

    // Sprawdź czy minęła minuta od ostatniego alarmu
    if (currentTime - status.lastSoundAlert >= SOUND_ALERT_INTERVAL) {
        // Sprawdź czy dźwięk jest włączony i czy występuje któryś z alarmów
        if (status.soundEnabled && (status.waterAlarmActive || status.pumpSafetyLock)) {
            
            // Sprawdź czy minęło wystarczająco dużo czasu od ostatniego alarmu
            if (currentTime - lastAlarmTime >= 1000) {  // 1000 ms = 1 sekunda
                playShortWarningSound();
                status.lastSoundAlert = currentTime;
                lastAlarmTime = currentTime;  // Zaktualizuj czas ostatniego alarmu
                
                // Debug info
                Serial.println(F("Alarm dźwiękowy - przyczyna:"));
                if (status.waterAlarmActive) Serial.println(F("- Brak wody"));
                if (status.pumpSafetyLock) Serial.println(F("- Alarm pompy"));
            }
        }
    }
}

void updateAlarmStates(float currentDistance) {
    // --- Obsługa alarmu krytycznie niskiego poziomu wody ---
    // Włącz alarm jeśli:
    // - odległość jest większa lub równa max (zbiornik pusty)
    // - alarm nie jest jeszcze aktywny
    if (currentDistance >= ODLEGLOSC_PUSTY && !status.waterAlarmActive) {
        status.waterAlarmActive = true;
        sensorAlarm.setValue("ON");               
        Serial.println("Brak wody ON");
    } 
    // Wyłącz alarm jeśli:
    // - odległość spadła poniżej progu wyłączenia (z histerezą)
    // - alarm jest aktywny
    else if (currentDistance < (ODLEGLOSC_PUSTY - HYSTERESIS) && status.waterAlarmActive) {
        status.waterAlarmActive = false;
        sensorAlarm.setValue("OFF");
        Serial.println("Brak wody OFF");
    }

    // --- Obsługa ostrzeżenia o rezerwie wody ---
    // Włącz ostrzeżenie o rezerwie jeśli:
    // - odległość osiągnęła próg rezerwy
    // - ostrzeżenie nie jest jeszcze aktywne
    if (currentDistance >= REZERWA_ODLEGLOSC && !status.waterReserveActive) {
        status.waterReserveActive = true;
        sensorReserve.setValue("ON");
        Serial.println("Rezerwa ON");
    } 
    // Wyłącz ostrzeżenie o rezerwie jeśli:
    // - odległość spadła poniżej progu rezerwy (z histerezą)
    // - ostrzeżenie jest aktywne
    else if (currentDistance < (REZERWA_ODLEGLOSC - HYSTERESIS) && status.waterReserveActive) {
        status.waterReserveActive = false;
        sensorReserve.setValue("OFF");
        Serial.println("Rezerwa OFF");
    }
}

// --- Deklaracje funkcji pompy

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
            digitalWrite(POMPA_PIN, LOW);  // Wyłącz pompę
            status.isPumpActive = false;  // Oznacz jako nieaktywną
            status.pumpStartTime = 0;  // Zeruj czas startu
            sensorPump.setValue("OFF");  // Aktualizuj status w HA
        }
        return;
    }

    // --- ZABEZPIECZENIE 2: Maksymalny czas pracy ---
    // Sprawdź czy pompa nie przekroczyła maksymalnego czasu pracy
    if (status.isPumpActive && (millis() - status.pumpStartTime > PUMP_WORK_TIME * 1000)) {
        digitalWrite(POMPA_PIN, LOW);  // Wyłącz pompę
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
            digitalWrite(POMPA_PIN, LOW);  // Wyłącz pompę
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
        digitalWrite(POMPA_PIN, LOW);  // Wyłącz pompę
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
            digitalWrite(POMPA_PIN, HIGH);  // Włącz pompę
            status.isPumpActive = true;  // Oznacz jako aktywną
            status.pumpStartTime = millis();  // Zapisz czas startu
            status.isPumpDelayActive = false;  // Wyłącz opóźnienie
            sensorPump.setValue("ON");  // Aktualizuj status w HA
        }
    }
}

// Wywoływana przy uruchomieniu pompy
void onPumpStart() {
    pumpStats.dailyPumpRuns++;
    pumpStats.weeklyPumpRuns++;
    pumpStats.monthlyPumpRuns++;
    
    pumpStartTime = millis();
    waterLevelBeforePump = getCurrentWaterLevel();
}

// Wywoływana przy zatrzymaniu pompy
void onPumpStop() {
    unsigned long workTime = (millis() - pumpStartTime) / 1000;  // czas w sekundach
    float currentLevel = getCurrentWaterLevel();
    
    // Oblicz faktyczne zużycie wody
    float waterUsed = calculateWaterUsed(waterLevelBeforePump, currentLevel);
    
    // Dodaj do statystyk tylko jeśli faktycznie zużyto wodę
    if (waterUsed > 0) {
        safeIncrementStats(workTime, waterUsed);
    }
    
    updateHAStatistics();
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
    device.setSoftwareVersion("16.11.24");         // Wersja oprogramowania

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

    // Statystyki
    sensorDailyPumpRuns.setName("Uruchomienia pompy - dzień");
    sensorDailyPumpRuns.setIcon("mdi:water-pump");
    
    sensorDailyPumpTime.setName("Czas pracy pompy - dzień");
    sensorDailyPumpTime.setIcon("mdi:timer");
    
    sensorDailyWaterUsed.setName("Zużycie wody - dzień");
    sensorDailyWaterUsed.setIcon("mdi:water");
    sensorDailyWaterUsed.setUnitOfMeasurement("L");

    // Tygodniowe statystyki
    sensorWeeklyPumpRuns.setName("Uruchomienia pompy - tydzień");
    sensorWeeklyPumpRuns.setIcon("mdi:water-pump");
    
    sensorWeeklyPumpTime.setName("Czas pracy pompy - tydzień");
    sensorWeeklyPumpTime.setIcon("mdi:timer");
    
    sensorWeeklyWaterUsed.setName("Zużycie wody - tydzień");
    sensorWeeklyWaterUsed.setIcon("mdi:water");
    sensorWeeklyWaterUsed.setUnitOfMeasurement("L");
    
    // Miesięczne statystyki
    sensorMonthlyPumpRuns.setName("Uruchomienia pompy - miesiąc");
    sensorMonthlyPumpRuns.setIcon("mdi:water-pump");
    
    sensorMonthlyPumpTime.setName("Czas pracy pompy - miesiąc");
    sensorMonthlyPumpTime.setIcon("mdi:timer");
    
    sensorMonthlyWaterUsed.setName("Zużycie wody - miesiąc");
    sensorMonthlyWaterUsed.setIcon("mdi:water");
    sensorMonthlyWaterUsed.setUnitOfMeasurement("L");
}

// Konfiguracja OTA (Over-The-Air) dla aktualizacji oprogramowania
void configOTA() {
    ArduinoOTA.setHostname("HydroSense");
    ArduinoOTA.setPassword("hydrosense");
    
    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else {
            type = "filesystem";
        }
        Serial.println("Start updating " + type);
    });
    
    ArduinoOTA.onEnd([]() {
        Serial.println("\nKoniec OTA");
        WiFi.disconnect(true);
        mqtt.disconnect();
        delay(100);
        ESP.eraseConfig();
    });
    
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Postęp: %u%%\r", (progress / (total / 100)));
    });
    
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    
    ArduinoOTA.begin();
    Serial.println("OTA Gotowy");
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
    status.waterAlarmActive = (initialDistance >= ODLEGLOSC_PUSTY);
    status.waterReserveActive = (initialDistance >= REZERWA_ODLEGLOSC);
    
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
    int measurements[MEASUREMENTS_COUNT];  // Tablica do przechowywania pomiarów

    for (int i = 0; i < MEASUREMENTS_COUNT; i++) {
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
    for (int i = 0; i < MEASUREMENTS_COUNT - 1; i++) {
        for (int j = 0; j < MEASUREMENTS_COUNT - i - 1; j++) {
            if (measurements[j] > measurements[j + 1]) {
                int temp = measurements[j];
                measurements[j] = measurements[j + 1];
                measurements[j + 1] = temp;
            }
        }
    }

    // Obliczenie mediany z pomiarów
    if (MEASUREMENTS_COUNT % 2 == 0) {
        // Jeśli liczba pomiarów jest parzysta, zwróć średnią z dwóch środkowych wartości
        int midIndex = MEASUREMENTS_COUNT / 2;
        if (measurements[midIndex - 1] == -1 || measurements[midIndex] == -1) {
            return -1;  // Jeśli środkowe wartości są błędne, zwróć błąd
        }
        return (measurements[midIndex - 1] + measurements[midIndex]) / 2;
    } else {
        // Jeśli liczba pomiarów jest nieparzysta, zwróć środkową wartość
        int midIndex = MEASUREMENTS_COUNT / 2;
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
    if (distance < ODLEGLOSC_PELNY) distance = ODLEGLOSC_PELNY;  // Nie mniej niż przy pełnym
    if (distance > ODLEGLOSC_PUSTY) distance = ODLEGLOSC_PUSTY;  // Nie więcej niż przy pustym
    
    // Obliczenie procentowe poziomu wody
    float percentage = (float)(ODLEGLOSC_PUSTY - distance) /  // Różnica: pusty - aktualny
                      (float)(ODLEGLOSC_PUSTY - ODLEGLOSC_PELNY) *  // Różnica: pusty - pełny
                      100.0;  // Przeliczenie na procenty
    
    return (int)percentage;  // Zwrot wartości całkowitej
}

//
float calculateWaterUsed(float beforeVolume, float afterVolume) {
    float difference = beforeVolume - afterVolume;
    return difference > 0 ? difference : 0;  // zwracamy tylko dodatnie zużycie
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
    float waterHeight = ODLEGLOSC_PUSTY - currentDistance;    
    waterHeight = constrain(waterHeight, 0, ODLEGLOSC_PUSTY - ODLEGLOSC_PELNY);
    
    // Obliczenie objętości w litrach (wszystko w mm)
    float radius = SREDNICA_ZBIORNIKA / 2.0;
    volume = PI * (radius * radius) * waterHeight / 1000000.0; // mm³ na litry
    
    // Jeśli objętość się zmieniła, aktualizujemy statystyki
    if (volume != previousVolume) {
        float waterUsed = calculateWaterUsed(previousVolume, volume);
        if (waterUsed > 0) {
            safeIncrementStats(0, waterUsed);
        }
    }

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
                Serial.println("Alarm pompy skasowany");
            }
        }
    }
    
    lastReading = reading;  // Zapisz ostatni odczyt dla następnego porównania
    yield();  // Oddaj sterowanie systemowi
}

// 
void resetStatistics(const char* period) {
    time_t local = getLocalTime();
    
    if (strcmp(period, "daily") == 0) {
        pumpStats.dailyPumpRuns = 0;
        pumpStats.dailyPumpWorkTime = 0;
        pumpStats.dailyWaterUsed = 0;
        pumpStats.lastDailyReset = local;
        
        Serial.println(F("Resetowanie statystyk dziennych"));
    } 
    else if (strcmp(period, "weekly") == 0) {
        pumpStats.weeklyPumpRuns = 0;
        pumpStats.weeklyPumpWorkTime = 0;
        pumpStats.weeklyWaterUsed = 0;
        pumpStats.lastWeeklyReset = local;
        
        Serial.println(F("Resetowanie statystyk tygodniowych"));
    } 
    else if (strcmp(period, "monthly") == 0) {
        pumpStats.monthlyPumpRuns = 0;
        pumpStats.monthlyPumpWorkTime = 0;
        pumpStats.monthlyWaterUsed = 0;
        pumpStats.lastMonthlyReset = local;
        
        Serial.println(F("Resetowanie statystyk miesięcznych"));
    }
}

// Sprawdzenie i reset statystyk
void safeIncrementStats(unsigned long workTime, float waterUsed) {
    // Dzienne statystyki
    if (pumpStats.dailyPumpRuns < UINT8_MAX) {
        pumpStats.dailyPumpRuns++;
    }
    if (pumpStats.dailyPumpWorkTime + workTime <= UINT16_MAX) {
        pumpStats.dailyPumpWorkTime += workTime;
    }
    if (waterUsed > 0 && pumpStats.dailyWaterUsed + waterUsed <= UINT8_MAX) {
        pumpStats.dailyWaterUsed += (uint8_t)waterUsed;
    }

    // Tygodniowe statystyki
    if (pumpStats.weeklyPumpRuns < UINT8_MAX) {
        pumpStats.weeklyPumpRuns++;
    }
    if (pumpStats.weeklyPumpWorkTime + workTime <= UINT16_MAX) {
        pumpStats.weeklyPumpWorkTime += workTime;
    }
    if (waterUsed > 0 && pumpStats.weeklyWaterUsed + waterUsed <= UINT8_MAX) {
        pumpStats.weeklyWaterUsed += (uint8_t)waterUsed;
    }

    // Miesięczne statystyki
    if (pumpStats.monthlyPumpRuns < UINT8_MAX) {
        pumpStats.monthlyPumpRuns++;
    }
    if (pumpStats.monthlyPumpWorkTime + workTime <= UINT16_MAX) {
        pumpStats.monthlyPumpWorkTime += workTime;
    }
    if (waterUsed > 0 && pumpStats.monthlyWaterUsed + waterUsed <= UINT8_MAX) {
        pumpStats.monthlyWaterUsed += (uint8_t)waterUsed;
    }
}

// Sprawdzenie i reset statystyk
void checkStatisticsReset() {
    time_t now = getLocalTime();
    struct tm *timeinfo = localtime(&now);
    struct tm *lastDailyReset = localtime(&pumpStats.lastDailyReset);
    struct tm *lastWeeklyReset = localtime(&pumpStats.lastWeeklyReset);
    struct tm *lastMonthlyReset = localtime(&pumpStats.lastMonthlyReset);

    // Debug info
    #ifdef DEBUG
    Serial.println(F("Checking statistics reset:"));
    Serial.print(F("Current time: "));
    printDateTime(now);
    Serial.print(F("Last daily reset: "));
    printDateTime(pumpStats.lastDailyReset);
    #endif
    
    // Reset dzienny - sprawdź czy zmienił się dzień
    if (timeinfo->tm_mday != lastDailyReset->tm_mday || 
        timeinfo->tm_mon != lastDailyReset->tm_mon ||
        timeinfo->tm_year != lastDailyReset->tm_year) {
        
        #ifdef DEBUG
        Serial.println(F("Performing daily reset"));
        #endif
        
        resetStatistics("daily");
    }
    
    // Reset tygodniowy - sprawdź czy jest niedziela i czy ostatni reset nie był dziś
    if (timeinfo->tm_wday == 0 && // 0 = niedziela
        (lastWeeklyReset->tm_mday != timeinfo->tm_mday || 
         lastWeeklyReset->tm_mon != timeinfo->tm_mon ||
         lastWeeklyReset->tm_year != timeinfo->tm_year)) {
        
        #ifdef DEBUG
        Serial.println(F("Performing weekly reset"));
        #endif
        
        resetStatistics("weekly");
    }
    
    // Reset miesięczny - sprawdź czy zmienił się miesiąc
    if (timeinfo->tm_mon != lastMonthlyReset->tm_mon ||
        timeinfo->tm_year != lastMonthlyReset->tm_year) {
        
        #ifdef DEBUG
        Serial.println(F("Performing monthly reset"));
        #endif
        
        resetStatistics("monthly");
    }
}

// Aktualizacja statystyk Home Assistant
void updateHAStatistics() {
    char value[16];
    
    // Konwersja liczby uruchomień pompy
    itoa(pumpStats.dailyPumpRuns, value, 10);
    sensorDailyPumpRuns.setValue(value);
    
    // Formatowanie czasu pracy pompy
    formatTimeForHA(value, sizeof(value), pumpStats.dailyPumpWorkTime);
    sensorDailyPumpTime.setValue(value);
    
    // Konwersja zużycia wody
    dtostrf(pumpStats.dailyWaterUsed, 4, 2, value);
    sensorDailyWaterUsed.setValue(value);
    
    // Formatowanie czasu dla statystyk tygodniowych
    formatTimeForHA(value, sizeof(value), pumpStats.weeklyPumpWorkTime);
    sensorWeeklyPumpTime.setValue(value);
    
    itoa(pumpStats.weeklyPumpRuns, value, 10);
    sensorWeeklyPumpRuns.setValue(value);
    
    dtostrf(pumpStats.weeklyWaterUsed, 4, 2, value);
    sensorWeeklyWaterUsed.setValue(value);
    
    // Formatowanie czasu dla statystyk miesięcznych
    formatTimeForHA(value, sizeof(value), pumpStats.monthlyPumpWorkTime);
    sensorMonthlyPumpTime.setValue(value);
    
    itoa(pumpStats.monthlyPumpRuns, value, 10);
    sensorMonthlyPumpRuns.setValue(value);
    
    dtostrf(pumpStats.monthlyWaterUsed, 4, 2, value);
    sensorMonthlyWaterUsed.setValue(value);
}

// Czas
time_t getLocalTime() {
    time_t now;
    time(&now);
    return now;  // Czas jest już lokalny dzięki konfiguracji strefy czasowej
}

// Funkcja pomocnicza do wyświetlania czasu (przydatna przy debugowaniu)
void printDateTime(time_t t) {
    char buf[32];
    sprintf(buf, "%.2d-%.2d-%.2d %.2d:%.2d:%.2d",
        year(t), month(t), day(t),
        hour(t), minute(t), second(t));
    Serial.println(buf);
}

//
void formatTimeForHA(char* buffer, size_t size, uint32_t seconds) {
    // Konwersja sekund na godziny i minuty
    uint16_t hours = seconds / 3600;
    uint16_t minutes = (seconds % 3600) / 60;
    
    // Formatowanie w formacie HH:MM
    snprintf(buffer, size, "%02d:%02d", hours, minutes);
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

// --- Setup
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
    Serial.print("Łączenie z WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        ESP.wdtFeed(); // Reset watchdoga podczas łączenia
    }
    Serial.println("\nPołączono z WiFi");

    configOTA();

    // Konfiguracja czasu
    Serial.println("Konfiguracja czasu...");
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");    // Serwery NTP
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);       // Strefa czasowa dla Polski
    tzset();

    // Czekaj na synchronizację czasu
    Serial.print("Oczekiwanie na synchronizację czasu");
    time_t now = time(nullptr);
    while (now < 24 * 3600) {
        delay(500);
        Serial.print(".");
        ESP.wdtFeed();
        now = time(nullptr);
    }
    Serial.println("\nCzas zsynchronizowany");

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
    
    // Inicjalizacja statystyk z aktualnym czasem
    time_t local = time(nullptr);
    struct tm *timeinfo = localtime(&local);
    
    // Ustaw znaczniki czasowe resetów
    pumpStats.lastDailyReset = local;
    pumpStats.lastWeeklyReset = local;
    pumpStats.lastMonthlyReset = local;

    // Debug info
    Serial.print("Aktualny czas: ");
    Serial.print(timeinfo->tm_year + 1900);
    Serial.print("-");
    Serial.print(timeinfo->tm_mon + 1);
    Serial.print("-");
    Serial.print(timeinfo->tm_mday);
    Serial.print(" ");
    Serial.print(timeinfo->tm_hour);
    Serial.print(":");
    Serial.print(timeinfo->tm_min);
    Serial.print(":");
    Serial.println(timeinfo->tm_sec);

    Serial.println("Setup zakończony pomyślnie!");
    
    if (status.soundEnabled) {
        //welcomeMelody();
    }  
}

// --- Pętla główna
// void loop() {
//     unsigned long currentMillis = millis();
//     static unsigned long lastMQTTRetry = 0;
//     static unsigned long lastMeasurement = 0;

//     // KRYTYCZNE OPERACJE SYSTEMOWE
    
//     // Zabezpieczenie przed zawieszeniem systemu
//     ESP.wdtFeed();      // Resetowanie licznika watchdog
//     yield();            // Obsługa krytycznych zadań systemowych ESP8266
    
//     // System aktualizacji bezprzewodowej
//     ArduinoOTA.handle(); // Nasłuchiwanie żądań aktualizacji OTA

//     // ZARZĄDZANIE ŁĄCZNOŚCIĄ
    
//     // Sprawdzanie i utrzymanie połączenia WiFi
//     if (WiFi.status() != WL_CONNECTED) {
//         setupWiFi();    // Próba ponownego połączenia z siecią
//         return;         // Powrót do początku pętli po próbie połączenia
//     }
    
//     // Zarządzanie połączeniem MQTT
//     if (!mqtt.isConnected()) {
//         if (currentMillis - lastMQTTRetry >= 10000) { // Ponowna próba co 10 sekund
//             lastMQTTRetry = currentMillis;
//             Serial.println("\nBrak połączenia MQTT - próba reconnect...");
//             if (mqtt.begin(MQTT_SERVER, 1883, MQTT_USER, MQTT_PASSWORD)) {
//                 Serial.println("MQTT połączono ponownie!");
//             }
//         }
//     }
        
//     mqtt.loop();  // Obsługa komunikacji MQTT

//     // GŁÓWNE FUNKCJE URZĄDZENIA
    
//     // Obsługa interfejsu i sterowania
//     handleButton();     // Przetwarzanie sygnałów z przycisków
//     updatePump();       // Sterowanie pompą
//     checkAlarmConditions(); // System ostrzeżeń dźwiękowych
    
//     // Monitoring poziomu wody
//     if (millis() - lastMeasurement >= MEASUREMENT_INTERVAL) {
//         updateWaterLevel();  // Pomiar i aktualizacja stanu wody
//         lastMeasurement = millis();
//     }

//     // STATYSTYKI I ZARZĄDZANIE CZASEM
    
//     // Aktualizacja czasu i resetowanie statystyk (co godzinę)
//     // Aktualizacja czasu
//     if (millis() - lastTimeUpdate >= TIME_UPDATE_INTERVAL) {
//         timeClient.update();
//         lastTimeUpdate = millis();
//     }
    
//     // Aktualizacja statystyk Home Assistant (co minutę)
//     if (currentMillis - lastStatsUpdate >= 60000) {
//         updateHAStatistics();
//         lastStatsUpdate = currentMillis;
//     }
// }

void loop() {
    unsigned long currentMillis = millis();
    static unsigned long lastMQTTRetry = 0;
    static unsigned long lastMeasurement = 0;
    static unsigned long lastStatsUpdate = 0;

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
    if (currentMillis - lastMeasurement >= MEASUREMENT_INTERVAL) {
        updateWaterLevel();  // Pomiar i aktualizacja stanu wody
        lastMeasurement = currentMillis;
    }

    // STATYSTYKI I ZARZĄDZANIE CZASEM
    
    // Sprawdzanie resetów statystyk
    static time_t lastTimeCheck = 0;
    time_t now = time(nullptr);
    
    // Sprawdzaj resety co minutę
    if (now - lastTimeCheck >= 60) {
        checkStatisticsReset();
        lastTimeCheck = now;
        
        // Debug - wyświetl aktualny czas
        #ifdef DEBUG
        struct tm timeinfo;
        localtime_r(&now, &timeinfo);
        Serial.printf("Aktualny czas: %04d-%02d-%02d %02d:%02d:%02d\n",
                     timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                     timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        #endif
    }
    
    // Aktualizacja statystyk Home Assistant (co minutę)
    if (currentMillis - lastStatsUpdate >= 60000) {
        updateHAStatistics();
        lastStatsUpdate = currentMillis;
        
        // Synchronizacja czasu (co godzinę)
        static time_t lastSync = 0;
        if (now - lastSync >= 3600) {
            configTime(0, 0, "pool.ntp.org", "time.nist.gov");
            lastSync = now;
            #ifdef DEBUG
            Serial.println("Wykonano synchronizację czasu");
            #endif
        }
    }
}
