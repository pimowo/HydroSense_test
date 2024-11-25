//----------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------

// ---------------------- BIBLIOTEKI ----------------------------------

#include <Arduino.h>                  // Podstawowa biblioteka Arduino zawierająca funkcje rdzenia
#include <ArduinoHA.h>                // Biblioteka do integracji z Home Assistant przez protokół MQTT
#include <ArduinoOTA.h>               // Biblioteka do aktualizacji oprogramowania przez sieć WiFi
#include <ESP8266WiFi.h>              // Biblioteka WiFi dedykowana dla układu ESP8266
#include <EEPROM.h>                   // Biblioteka do dostępu do pamięci nieulotnej EEPROM
#include <WiFiManager.h>              // Biblioteka do zarządzania połączeniami WiFi
#include <ESP8266WebServer.h>         // Biblioteka do obsługi serwera HTTP na ESP8266
//#include <WebSocketsServer.h>         // Biblioteka do obsługi serwera WebSockets na ESP8266
#include <ESP8266HTTPUpdateServer.h>  //

// ---------------------- DEFINICJE STAŁYCH ---------------------------

// Wersja programu
const char* SOFTWARE_VERSION = "24.11.24";

// Konfiguracja pinów ESP8266
const int PIN_ULTRASONIC_TRIG = D6;  // Pin TRIG czujnika ultradźwiękowego
const int PIN_ULTRASONIC_ECHO = D7;  // Pin ECHO czujnika ultradźwiękowego
const int PIN_WATER_LEVEL = D5;      // Pin czujnika poziomu wody w akwarium
const int POMPA_PIN = D1;            // Pin sterowania pompą
const int BUZZER_PIN = D2;           // Pin buzzera do alarmów dźwiękowych
const int PRZYCISK_PIN = D3;         // Pin przycisku do kasowania alarmów

// Parametry czasowe (wszystkie wartości w milisekundach)
const unsigned long ULTRASONIC_TIMEOUT = 50;                          // Timeout pomiaru czujnika ultradźwiękowego
const unsigned long MEASUREMENT_INTERVAL = 60000;                     // Interwał między pomiarami
const unsigned long WATCHDOG_TIMEOUT = 8000;                          // Timeout dla watchdoga
const unsigned long LONG_PRESS_TIME = 1000;                           // Czas długiego naciśnięcia przycisku
const unsigned long MQTT_LOOP_INTERVAL = 100;                         // Obsługa MQTT co 100ms
const unsigned long OTA_CHECK_INTERVAL = 1000;                        // Sprawdzanie OTA co 1s
const unsigned long MQTT_RETRY_INTERVAL = 10000;                      // Próba połączenia MQTT co 10s
const unsigned long MILLIS_OVERFLOW_THRESHOLD = 4294967295U - 60000;  // ~49.7 dni

// Parametry czujnika
// Wszystkie odległości w milimetrach od czujnika do powierzchni wody
// Mniejsza odległość = wyższy poziom wody
const int HYSTERESIS = 10;          // Histereza przy zmianach poziomu (mm)
const int SENSOR_MIN_RANGE = 20;    // Minimalny zakres czujnika (mm)
const int SENSOR_MAX_RANGE = 1020;  // Maksymalny zakres czujnika (mm)
const float EMA_ALPHA = 0.2f;       // Współczynnik wygładzania dla średniej wykładniczej (0-1)
const int SENSOR_AVG_SAMPLES = 3;   // Liczba próbek do uśrednienia pomiaru

// Definicja debugowania
#define DEBUG 1  // 0 wyłącza debug, 1 włącza debug

#if DEBUG
    #define DEBUG_PRINT(x) Serial.println(x)
    #define DEBUG_PRINTF(format, ...) Serial.printf(format, __VA_ARGS__)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTF(format, ...)
#endif

// ---------------------- STRUKTURY DANYCH ----------------------------

// Definicja struktury konfiguracji
struct Config {
    // Wersja konfiguracji i flagi systemowe
    uint8_t version;          // Wersja konfiguracji do kontroli kompatybilności
    bool soundEnabled;        // Globalne ustawienie dźwięku
    bool ha_enabled;         // Czy integracja z Home Assistant jest aktywna

    // Ustawienia AP
    char ap_password[32];     // Hasło AP
    
    // Ustawienia WWW
    char www_username[32];    // Użytkownik www
    char www_password[32];    // Hasło wwww

    // Ustawienia sieciowe
    char wifi_ssid[32];     // SSID sieci WiFi (zapisywane przez WiFiManager)
    char wifi_pass[64];     // Hasło do sieci WiFi (zapisywane przez WiFiManager)
    
    // Ustawienia MQTT/Home Assistant
    char mqtt_server[40];    // Adres brokera MQTT
    uint16_t mqtt_port;      // Port MQTT (domyślnie 1883)
    char mqtt_user[32];      // Login MQTT
    char mqtt_password[32];  // Hasło MQTT
    
    // Ustawienia dostępu do interfejsu web
    char webUser[32];       // Login do interfejsu web
    char webPassword[32];   // Hasło do interfejsu web
    
    // Ustawienia zbiornika
    int tank_full;          // Poziom pełnego zbiornika [cm]
    int tank_empty;         // Poziom pustego zbiornika [cm]
    int reserve_level;      // Poziom rezerwy [cm]
    int tank_diameter;      // Średnica zbiornika [cm]
    
    // Ustawienia pompy
    int pump_delay;         // Opóźnienie startu pompy [s]
    int pump_work_time;     // Maksymalny czas pracy [s]
    
    // Suma kontrolna
    char checksum;          // XOR wszystkich poprzednich pól

    // Metoda do aktualizacji sumy kontrolnej
    void updateChecksum() {
        char sum = 0;
        uint8_t* ptr = (uint8_t*)this;
        for(size_t i = 0; i < sizeof(Config) - 1; i++) {
            sum ^= ptr[i];
        }
        checksum = sum;
    }

    // Konstruktor
    Config() : 
        version(1),
        mqtt_port(1883) {
        strcpy(ap_password, "hydrosense");
        strcpy(www_username, "HydroSense");
        strcpy(www_password, "hydrosense");
        wifi_ssid[0] = '\0';
        wifi_pass[0] = '\0';
        mqtt_server[0] = '\0';
        mqtt_user[0] = '\0';
        //mqtt_pass[0] = '\0';
        updateChecksum();
    }
};

// Status urządzenia
struct Status {
    bool soundEnabled;                        // Flaga wskazująca, czy dźwięk jest włączony
    bool waterAlarmActive;                    // Flaga wskazująca, czy alarm wodny jest aktywny
    bool waterReserveActive;                  // Flaga wskazująca, czy rezerwa wody jest aktywna
    bool isPumpActive;                        // Flaga wskazująca, czy pompa jest aktywna
    bool isPumpDelayActive;                   // Flaga wskazująca, czy opóźnienie pompy jest aktywne
    bool pumpSafetyLock;                      // Flaga wskazująca, czy blokada bezpieczeństwa pompy jest aktywna
    bool isServiceMode;                       // Flaga wskazująca, czy tryb serwisowy jest włączony
    bool needsUpdate;                         // Flaga wskazująca potrzebę aktualizacji interfejsu
    float waterLevelPercent;    // Dodane
    float currentDistance;      // Dodane   
    //pumpSwitchfloat waterLevelBeforePump;               // Poziom wody przed uruchomieniem pompy
    unsigned long pumpStartTime;              // Znacznik czasu uruchomienia pompy
    unsigned long pumpDelayStartTime;         // Znacznik czasu rozpoczęcia opóźnienia pompy
    unsigned long lastSoundAlert;             // Znacznik czasu ostatniego alertu dźwiękowego
    unsigned long lastSuccessfulMeasurement;  // Znacznik czasu ostatniego udanego pomiaru

    // Konstruktor - inicjalizacja wszystkich pól
    Status() : 
        soundEnabled(false),
        waterAlarmActive(false),
        waterReserveActive(false),
        isPumpActive(false),
        isPumpDelayActive(false),
        pumpSafetyLock(false),
        isServiceMode(false),
        needsUpdate(false),
        //waterLevelBeforePump(0),
        pumpStartTime(0),
        pumpDelayStartTime(0),
        lastSoundAlert(0),
        lastSuccessfulMeasurement(0)
    {}
};

// Stan przycisku
struct ButtonState {
    bool lastState;                   // Poprzedni stan przycisku
    bool isInitialized = false; 
    bool isLongPressHandled = false;  // Flaga obsłużonego długiego naciśnięcia
    unsigned long pressedTime = 0;    // Czas wciśnięcia przycisku
    unsigned long releasedTime = 0;   // Czas puszczenia przycisku
};

// Timery systemowe
struct Timers {
    unsigned long lastMQTTRetry;    // Znacznik czasu ostatniej próby połączenia MQTT
    unsigned long lastMeasurement;  // Znacznik czasu ostatniego pomiaru
    unsigned long lastOTACheck;     // Znacznik czasu ostatniego sprawdzenia OTA (Over-The-Air)
    unsigned long lastMQTTLoop;     // Znacznik czasu ostatniego cyklu pętli MQTT
    
    // Konstruktor inicjalizujący wszystkie znaczniki czasowe na 0
    Timers() : lastMQTTRetry(0), lastMeasurement(0), lastOTACheck(0), lastMQTTLoop(0) {}
};

// ---------------------- ZMIENNE GLOBALNE ----------------------------

// Instancje obiektów
ESP8266WebServer server(80);
//WebSocketsServer webSocket = WebSocketsServer(81);
WiFiClient client;  // Klient połączenia WiFi
HADevice device("HydroSense");  // Definicja urządzenia dla Home Assistant
HAMqtt mqtt(client, device);  // Klient MQTT dla Home Assistant

// Sensory Home Assistant

//     Sensory pomiarowe
HASensor sensorDistance("water_level");         // Odległość od lustra wody (w mm)
HASensor sensorLevel("water_level_percent");    // Poziom wody w zbiorniku (w procentach)
HASensor sensorVolume("water_volume");          // Objętość wody (w litrach)
HASensor sensorPumpWorkTime("pump_work_time");  // Czas pracy pompy
//     Sensory statusu
HASensor sensorPump("pump");    // Praca pompy (ON/OFF)
HASensor sensorWater("water");  // Czujnik poziomu w akwarium (ON=niski/OFF=ok)
//     Sensory alarmowe
HASensor sensorAlarm("water_alarm");      // Brak wody w zbiorniku dolewki
HASensor sensorReserve("water_reserve");  // Rezerwa w zbiorniku dolewki
//     Przełączniki
HASwitch switchPumpAlarm("pump_alarm");  // Resetowania blokady pompy
HASwitch switchService("service_mode");  // Tryb serwisowy
HASwitch switchSound("sound_switch");    // Dźwięki systemu

// Zmienne stanu
Config config;
Status status;
ButtonState buttonState;         // Instancja struktury ButtonState
static Timers timers;            // Instancja struktury Timers
float lastFilteredDistance = 0;  // Dla filtra EMA (Exponential Moving Average)
float lastReportedDistance = 0;
float currentDistance = 0;
float volume = 0;
unsigned long lastMeasurement = 0;
unsigned long pumpStartTime = 0;
float waterLevelBeforePump = 0;

// ---------------------- STAŁE STRINGI I SZABLONY HTML ---------------

//
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

// Stopka
const char PAGE_FOOTER[] PROGMEM = R"rawliteral(
<div class='footer'>
    <a href='https://github.com/pimowo/HydroSense' target='_blank'>Project by PMW</a>
</div>
)rawliteral";

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
            /* [Wszystkie style CSS pozostają bez zmian] */
        </style>

        <!-- Skrypty JavaScript -->
        <script>
            // Inicjalizacja SSE
            const evtSource = new EventSource('/events');
            
            evtSource.onmessage = function(event) {
                const data = JSON.parse(event.data);
                updateStatus(data);
            };
            
            evtSource.onerror = function(err) {
                console.error("EventSource failed:", err);
                showMessage('Błąd połączenia z serwerem', 'error');
            };
            
            // Funkcja aktualizująca status
            function updateStatus(data) {
                if (data.waterLevel !== undefined) {
                    document.getElementById('waterLevel').textContent = data.waterLevel.toFixed(1);
                }
                if (data.isPumpActive !== undefined) {
                    document.getElementById('pumpState').textContent = data.isPumpActive ? 'Włączona' : 'Wyłączona';
                    document.getElementById('pumpState').className = data.isPumpActive ? 'status success' : 'status error';
                }
                if (data.waterAlarmActive !== undefined) {
                    document.getElementById('alarmState').textContent = data.waterAlarmActive ? 'Aktywny' : 'Nieaktywny';
                    document.getElementById('alarmState').className = data.waterAlarmActive ? 'status error' : 'status success';
                }
                if (data.soundEnabled !== undefined) {
                    document.getElementById('soundStatus').textContent = data.soundEnabled ? 'Włączony' : 'Wyłączony';
                    document.getElementById('soundStatus').className = data.soundEnabled ? 'status success' : 'status error';
                }
            }

            // Funkcja wyświetlająca komunikaty
            function showMessage(text, type) {
                const messageBox = document.createElement('div');
                messageBox.className = 'message ' + type;
                messageBox.innerHTML = text;
                document.body.appendChild(messageBox);
                
                setTimeout(() => messageBox.style.opacity = '1', 10);
                setTimeout(() => {
                    messageBox.style.opacity = '0';
                    setTimeout(() => messageBox.remove(), 300);
                }, 3000);
            }

            // Funkcje zarządzania urządzeniem
            function rebootDevice() {
                if(confirm('Czy na pewno chcesz zrestartować urządzenie?')) {
                    fetch('/reboot', {method: 'POST'})
                        .then(response => response.json())
                        .then(data => {
                            showMessage(data.message, 'success');
                            setTimeout(() => window.location.reload(), 5000);
                        })
                        .catch(() => showMessage('Błąd podczas restartu urządzenia', 'error'));
                }
            }
            
            function factoryReset() {
                if(confirm('Czy na pewno chcesz przywrócić ustawienia fabryczne? Spowoduje to utratę wszystkich ustawień!')) {
                    fetch('/factory-reset', {method: 'POST'})
                        .then(response => response.json())
                        .then(data => {
                            showMessage(data.message, 'success');
                            setTimeout(() => window.location.reload(), 5000);
                        })
                        .catch(() => showMessage('Błąd podczas przywracania ustawień', 'error'));
                }
            }

            // Obsługa formularza konfiguracji
            document.addEventListener('DOMContentLoaded', function() {
                document.querySelectorAll('form').forEach(form => {
                    form.addEventListener('submit', function(e) {
                        e.preventDefault();
                        
                        fetch(this.action, {
                            method: 'POST',
                            body: new FormData(this)
                        })
                        .then(response => response.json())
                        .then(data => {
                            if (data.error) {
                                showMessage(data.error, 'error');
                            } else {
                                showMessage(data.message, 'success');
                                if (data.restart) {
                                    setTimeout(() => window.location.reload(), 5000);
                                }
                            }
                        })
                        .catch(() => showMessage('Błąd podczas zapisywania konfiguracji', 'error'));
                    });
                });
            });

            // Obsługa aktualizacji firmware
            function handleUpdate(form) {
                const formData = new FormData(form);
                const progressBar = document.getElementById('progress-bar');
                const progressContainer = document.getElementById('update-progress');
                
                progressContainer.style.display = 'block';
                progressBar.style.width = '0%';
                progressBar.textContent = '0%';

                fetch('/update', {
                    method: 'POST',
                    body: formData
                })
                .then(response => response.json())
                .then(data => {
                    if (data.error) {
                        showMessage(data.error, 'error');
                        progressContainer.style.display = 'none';
                    } else {
                        progressBar.style.width = '100%';
                        progressBar.textContent = '100%';
                        showMessage('Aktualizacja zakończona! Trwa restart...', 'success');
                        setTimeout(() => window.location.reload(), 5000);
                    }
                })
                .catch(() => {
                    showMessage('Błąd podczas aktualizacji', 'error');
                    progressContainer.style.display = 'none';
                });

                return false;
            }
        </script>
    </head>
    <body>
        <h1>HydroSense</h1>
        
        <div class='section'>
            <h2>Status systemu</h2>
            <table class='config-table'>
                <tr>
                    <td>Poziom wody</td>
                    <td><span id="waterLevel">...</span> cm</td>
                </tr>
                <tr>
                    <td>Stan pompy</td>
                    <td><span id="pumpState" class="status">...</span></td>
                </tr>
                <tr>
                    <td>Stan alarmu</td>
                    <td><span id="alarmState" class="status">...</span></td>
                </tr>
                <tr>
                    <td>Status MQTT</td>
                    <td><span class='status %MQTT_STATUS_CLASS%'>%MQTT_STATUS%</span></td>
                </tr>
                <tr>
                    <td>Status dźwięku</td>
                    <td><span id="soundStatus" class='status %SOUND_STATUS_CLASS%'>%SOUND_STATUS%</span></td>
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

// ---------------------- FUNKCJE KONFIGURACYJNE ----------------------

// Ustawienia domyślne urządzenia
// Wywoływana przy pierwszym uruchomieniu lub po resecie do ustawień fabrycznych
void setDefaultConfig() {
    // Podstawowa konfiguracja
    //config.version = CONFIG_VERSION;  // Ustawienie wersji konfiguracji
    config.soundEnabled = true;  // Włączenie powiadomień dźwiękowych
    
    // MQTT
    strlcpy(config.mqtt_server, "", sizeof(config.mqtt_server));
    config.mqtt_port = 1883;
    strlcpy(config.mqtt_user, "", sizeof(config.mqtt_user));
    strlcpy(config.mqtt_password, "", sizeof(config.mqtt_password));

    // WWW
    strlcpy(config.webUser, "admin", sizeof(config.webUser));
    strlcpy(config.webPassword, "hydrosense", sizeof(config.webPassword));
    
    // Konfiguracja zbiornika
    config.tank_full = 50;       // czujnik od powierzchni przy pełnym zbiorniku
    config.tank_empty = 1050;    // czujnik od dna przy pustym zbiorniku
    config.reserve_level = 550;  // czujnik od poziomu rezerwy
    config.tank_diameter = 100;  // średnica zbiornika
    
    // Konfiguracja pompy
    config.pump_delay = 5;       // opóźnienie przed startem pompy
    config.pump_work_time = 30;  // Maksymalny czas ciągłej pracy pompy
    
    // Finalizacja
    config.checksum = calculateChecksum(config);  // Obliczenie sumy kontrolnej
    saveConfig();                                 // Zapis do EEPROM
    
    DEBUG_PRINT(F("Utworzono domyślną konfigurację"));
}

// Wczytywanie konfiguracji
bool loadConfig() {
    EEPROM.begin(sizeof(Config) + 1);  // +1 dla sumy kontrolnej
    
    Config tempConfig;  // Tymczasowa struktura do wczytania danych
    
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
bool saveConfig() {
    EEPROM.begin(sizeof(Config) + 1);  // +1 dla sumy kontrolnej
    
    config.checksum = calculateChecksum(config);  // Oblicz sumę kontrolną przed zapisem
    
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
        return true;
    } else {
        DEBUG_PRINT("Błąd zapisu konfiguracji!");
        return false;
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

// ---------------------- FUNKCJE ZARZĄDZANIA URZĄDZENIEM -------------

// Resetowanie do ustawień fabrycznych
void factoryReset() {    
    WiFi.disconnect(true);  // Rozłącz WiFi; true = kasuj zapisane ustawienia
    WiFi.mode(WIFI_OFF);    // Wyłącz moduł WiFi  
   
    delay(100);
        
    WiFiManager wm;      // Utwórz instancję WiFiManager do zarządzania konfiguracją WiFi
    wm.resetSettings();  // Usuń zapisane ustawienia WiFiManager
    ESP.eraseConfig();   // Wyczyść konfigurację ESP zapisaną w pamięci flash
    
    setDefaultConfig();  // Załaduj domyślną konfigurację
    saveConfig();        // Zapisz domyślną konfigurację
    
    delay(100);
   
    ESP.reset();  // Zresetuj urządzenie ESP
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

// Konfiguracja kierunków pinów i stanów początkowych
void setupPin() {
    pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);    // Wyjście - trigger czujnika ultradźwiękowego
    pinMode(PIN_ULTRASONIC_ECHO, INPUT);     // Wejście - echo czujnika ultradźwiękowego
    digitalWrite(PIN_ULTRASONIC_TRIG, LOW);  // Upewnij się że TRIG jest LOW na starcie
    
    pinMode(PIN_WATER_LEVEL, INPUT_PULLUP);  // Wejście z podciąganiem - czujnik poziomu
    pinMode(PRZYCISK_PIN, INPUT_PULLUP);     // Wejście z podciąganiem - przycisk
    pinMode(BUZZER_PIN, OUTPUT);             // Wyjście - buzzer
    digitalWrite(BUZZER_PIN, LOW);           // Wyłączenie buzzera
    pinMode(POMPA_PIN, OUTPUT);              // Wyjście - pompa
    digitalWrite(POMPA_PIN, LOW);            // Wyłączenie pompy
}

// Konfiguracja i zarządzanie połączeniem WiFi
void setupWiFi() {
    WiFi.mode(WIFI_AP_STA); // Włącz tryb AP+STA
    
    // Tworzenie unikalnej nazwy AP na podstawie MAC
    uint8_t mac[6];
    WiFi.macAddress(mac);
    String apName = "HydroSense_" + 
                   String(mac[0], HEX) + String(mac[1], HEX) + 
                   String(mac[2], HEX) + String(mac[3], HEX) + 
                   String(mac[4], HEX) + String(mac[5], HEX);
    
    // Uruchom AP z hasłem z konfiguracji
    WiFi.softAP(apName.c_str(), config.ap_password);
    Serial.print(F("AP IP address: "));
    Serial.println(WiFi.softAPIP());
    
    // Konfiguracja WiFiManager
    WiFiManager wifiManager;
    wifiManager.setConfigPortalTimeout(180); // 3 minuty na konfigurację
    
    // Próba połączenia z zapisaną siecią
    if (!wifiManager.autoConnect(apName.c_str(), config.ap_password)) {
        Serial.println(F("Nie udało się połączyć z WiFi"));
    } else {
        Serial.println(F("Połączono z siecią WiFi"));
    }
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
    status.isServiceMode = false;  // Domyślnie wyłączony
    switchService.setState(false, true);  // Wymuszenie aktualizacji przy starcie
    
    // Przełącznik dźwięku
    switchSound.setName("Dźwięk");
    switchSound.setIcon("mdi:volume-high");
    switchSound.onCommand(onSoundSwitchCommand);
    switchSound.setState(status.soundEnabled);
    
    // Przełącznik alarmu pompy
    switchPumpAlarm.setName("Alarm pompy");
    switchPumpAlarm.setIcon("mdi:alert");
    switchPumpAlarm.onCommand(onPumpAlarmCommand);

    // Callbacks
    //pumpSwitch.onCommand(onPumpSwitchCommand);
    //pumpAlarmSwitch.onCommand(onPumpAlarmCommand);
    //serviceSwitch.onCommand(onServiceSwitchCommand);
    //soundSwitch.onCommand(onSoundSwitchCommand);
}

//
void setupWebServer() {
    // Endpoint główny z autentykacją
    server.on("/", HTTP_GET, []() {
        // if (!server.authenticate(config.webUser, config.webPassword)) {
        //     return server.requestAuthentication(BASIC_AUTH, "HydroSense", "Unauthorized");
        // }
        handleRoot();
    });
    
    // Endpoint zapisu konfiguracji z autentykacją
    server.on("/save", HTTP_POST, []() {
        // if (!server.authenticate(config.webUser, config.webPassword)) {
        //     return server.requestAuthentication(BASIC_AUTH, "HydroSense", "Unauthorized");
        // }
        handleSave();
    });
    
    // Endpoint SSE z autentykacją
    server.on("/events", HTTP_GET, []() {
        // if (!server.authenticate(config.webUser, config.webPassword)) {
        //     return server.requestAuthentication(BASIC_AUTH, "HydroSense", "Unauthorized");
        // }
        handleEvents();
    });
    
    // Endpoint aktualizacji firmware z autentykacją
    server.on("/update", HTTP_POST, []() {
        // if (!server.authenticate(config.webUser, config.webPassword)) {
        //     return server.requestAuthentication(BASIC_AUTH, "HydroSense", "Unauthorized");
        // }
        handleDoUpdate();
    });
    
    // Endpoint statusu aktualizacji z autentykacją
    server.on("/updateResult", HTTP_GET, []() {
        // if (!server.authenticate(config.webUser, config.webPassword)) {
        //     return server.requestAuthentication(BASIC_AUTH, "HydroSense", "Unauthorized");
        // }
        handleUpdateResult();
    });
    
    // Endpoint zmiany hasła z autentykacją
    server.on("/changePassword", HTTP_POST, []() {
        // if (!server.authenticate(config.webUser, config.webPassword)) {
        //     return server.requestAuthentication(BASIC_AUTH, "HydroSense", "Unauthorized");
        // }
        handleChangePassword();
    });
    
    // Endpoint restartu z autentykacją i opóźnieniem
    server.on("/reboot", HTTP_POST, []() {
        // if (!server.authenticate(config.webUser, config.webPassword)) {
        //     return server.requestAuthentication(BASIC_AUTH, "HydroSense", "Unauthorized");
        // }
        server.send(200, "application/json", "{\"message\":\"Restarting...\",\"success\":true}");
        delay(1000);  // Daj czas na wysłanie odpowiedzi
        ESP.restart();
    });
    
    // Endpoint resetu fabrycznego z autentykacją
    server.on("/factory-reset", HTTP_POST, []() {
        // if (!server.authenticate(config.webUser, config.webPassword)) {
        //     return server.requestAuthentication(BASIC_AUTH, "HydroSense", "Unauthorized");
        // }
        server.send(200, "application/json", "{\"message\":\"Resetting to factory defaults...\",\"success\":true}");
        delay(200);  // Daj czas na wysłanie odpowiedzi
        factoryReset();
    });
    
    // Obsługa błędu 404
    server.onNotFound([]() {
        // if (!server.authenticate(config.webUser, config.webPassword)) {
        //     return server.requestAuthentication(BASIC_AUTH, "HydroSense", "Unauthorized");
        // }
        server.send(404, "text/plain", "404: Not found");
    });
    
    server.begin();
}

void handleEvents() {
    String data = "data: {";
    
    // Status MQTT
    data += "\"mqttConnected\":" + String(mqtt.isConnected() ? "true" : "false") + ",";
    
    // Status systemu
    data += "\"soundEnabled\":" + String(config.soundEnabled ? "true" : "false") + ",";
    
    // Dane zbiornika
    data += "\"waterLevel\":" + String(status.waterLevelPercent, 1) + ",";
    
    // Status pompy
    data += "\"isPumpActive\":" + String(status.isPumpActive ? "true" : "false") + ",";
    data += "\"pumpStartTime\":" + String(status.pumpStartTime) + ",";
    
    // Status alarmów
    data += "\"waterAlarmActive\":" + String(status.waterAlarmActive ? "true" : "false") + ",";
    
    // Konfiguracja zbiornika
    data += "\"tankEmpty\":" + String(config.tank_empty) + ",";
    data += "\"tankFull\":" + String(config.tank_full) + ",";
    data += "\"reserveLevel\":" + String(config.reserve_level) + ",";
    data += "\"tankDiameter\":" + String(config.tank_diameter) + ",";
    
    // Konfiguracja pompy
    data += "\"pumpDelay\":" + String(config.pump_delay) + ",";
    data += "\"pumpWorkTimeConfig\":" + String(config.pump_work_time) + ",";
    
    // Informacje systemowe
    data += "\"freeHeap\":" + String(ESP.getFreeHeap()) + ",";
    data += "\"uptime\":" + String(millis() / 1000) + ",";
    data += "\"version\":\"" + String(SOFTWARE_VERSION) + "\"";
    
    data += "}\n\n";

    server.sendHeader("Cache-Control", "no-cache");
    server.sendHeader("Content-Type", "text/event-stream");
    server.sendHeader("Connection", "keep-alive");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/event-stream", data);
}

//
String getStatusPage() {
    String page = F("<!DOCTYPE html><html><head>");
    page += F("<meta name='viewport' content='width=device-width, initial-scale='1.0'>");
    page += F("<style>");
    page += F("body{font-family:Arial,sans-serif;margin:20px;background:#f0f0f0;}");
    page += F(".container{max-width:800px;margin:0 auto;background:white;padding:20px;border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,0.1);}");
    page += F(".status-box{background:#f8f8f8;padding:15px;border-radius:4px;margin:10px 0;}");
    page += F(".status-item{margin:5px 0;}");
    page += F(".success{color:green;}");
    page += F(".warning{color:orange;}");
    page += F(".error{color:red;}");
    page += F("</style>");
    page += F("</head><body><div class='container'>");
    
    // Nagłówek
    page += F("<h1>HydroSense - Status</h1>");
    
    // Status sieci
    page += F("<div class='status-box'>");
    page += F("<h2>Status sieci</h2>");
    page += F("<div class='status-item'>AP: ");
    page += WiFi.softAPIP().toString();
    page += F("</div>");
    page += F("<div class='status-item'>Sieć WiFi: ");
    if (WiFi.status() == WL_CONNECTED) {
        page += F("<span class='success'>");
        page += WiFi.localIP().toString();
        page += F(" (");
        page += WiFi.SSID();
        page += F(")</span>");
    } else {
        page += F("<span class='warning'>Niepołączony</span>");
    }
    page += F("</div>");
    
    // Status systemu
    page += F("<div class='status-item'>HA Integration: ");
    page += config.ha_enabled ? F("<span class='success'>Włączona</span>") : F("<span class='warning'>Wyłączona</span>");
    page += F("</div>");
    
    // Status pomiarów
    page += F("<h2>Status pomiarów</h2>");
    page += F("<div class='status-item'>Poziom wody: ");
    page += String(status.waterLevelPercent);
    page += F("%</div>");
    page += F("<div class='status-item'>Odległość: ");
    page += String(status.currentDistance);
    page += F(" cm</div>");
    
    // Status pompy
    page += F("<h2>Status pompy</h2>");
    page += F("<div class='status-item'>Stan pompy: ");
    page += status.isPumpActive ? F("<span class='warning'>Włączona</span>") : F("<span class='success'>Wyłączona</span>");
    page += F("</div>");
    
    // Alarmy
    page += F("<h2>Alarmy</h2>");
    page += F("<div class='status-item'>Alarm wody: ");
    page += status.waterAlarmActive ? F("<span class='error'>Aktywny</span>") : F("<span class='success'>Nieaktywny</span>");
    page += F("</div>");
    
    // Przyciski
    page += F("<div style='margin-top:20px'>");
    page += F("<a href='/config' style='background:#007bff;color:white;padding:10px 20px;text-decoration:none;border-radius:4px;margin-right:10px'>Konfiguracja</a>");
    page += F("<a href='/update' style='background:#28a745;color:white;padding:10px 20px;text-decoration:none;border-radius:4px'>Aktualizacja</a>");
    page += F("</div>");
    
    page += F("</div></body></html>");
    return page;
}

// ---------------------- FUNKCJE POMIAROWE ---------------------------

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

// ---------------------- FUNKCJE STEROWANIA --------------------------

// Funkcje pompy
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
    
    // Tryb serwisowy
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

    //  Maksymalny czas pracy
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
    
    //  Blokady bezpieczeństwa
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

    // Ochrona przed przepełnieniem
    if (!waterPresent && status.isPumpActive) {
        digitalWrite(POMPA_PIN, LOW);
        status.isPumpActive = false;
        sendPumpWorkTime();  // Wyślij czas przy wyłączeniu
        status.pumpStartTime = 0;
        status.isPumpDelayActive = false;
        sensorPump.setValue("OFF");
        return;
    }
    
    //  Logika włączenia pompy
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

/**
 * Funkcja obsługująca fizyczny przycisk na urządzeniu
 * 
 * Obsługuje dwa tryby naciśnięcia:
 * - Krótkie (< 1s): przełącza tryb serwisowy
 * - Długie (≥ 1s): kasuje blokadę bezpieczeństwa pompy
 */
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

//
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

// ---------------------- FUNKCJE DŹWIĘKOWE ---------------------------

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

// Melodia powitalna
void welcomeMelody() {
    tone(BUZZER_PIN, 1397, 100);  // F6
    delay(150);
    tone(BUZZER_PIN, 1568, 100);  // G6
    delay(150);
    tone(BUZZER_PIN, 1760, 150);  // A6
    delay(200);
}

// ---------------------- OBSŁUGA HOME ASSISTANT ----------------------

// Funkcja nawiązująca połączenie z serwerem MQTT (Home Assistant)
bool connectMQTT() {   
    if (!mqtt.begin(config.mqtt_server, 1883, config.mqtt_user, config.mqtt_password)) {
        DEBUG_PRINT("\nBŁĄD POŁĄCZENIA MQTT!");
        return false;
    }
    
    DEBUG_PRINT("MQTT połączono pomyślnie!");
    return true;
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
    switchSound.setState(false);
    mqtt.loop();
    
    // Ustawienie końcowych stanów i wysyłka do HA
    sensorAlarm.setValue(status.waterAlarmActive ? "ON" : "OFF");
    sensorReserve.setValue(status.waterReserveActive ? "ON" : "OFF");
    switchSound.setState(status.soundEnabled);
    mqtt.loop();

    sensorPumpWorkTime.setValue("0");
    mqtt.loop();
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

/**
 * Funkcja obsługująca przełączanie trybu serwisowego z poziomu Home Assistant
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

/**
 * Funkcja obsługująca przełączanie dźwięku alarmu z poziomu Home Assistant
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

// ---------------------- OBSŁUGA SERWERA WWW -------------------------

//
String getConfigPage() {
    String html = FPSTR(CONFIG_PAGE);
    
    // Przygotuj wszystkie wartości przed zastąpieniem
    bool mqttConnected = client.connected();
    String mqttStatus = mqttConnected ? "Połączony" : "Rozłączony";
    String mqttStatusClass = mqttConnected ? "success" : "error";
    String soundStatus = config.soundEnabled ? "Włączony" : "Wyłączony";
    String soundStatusClass = config.soundEnabled ? "success" : "error";
    
    // Sekcja przycisków
    String buttons = F(
        "<div class='section'>"
        "<div class='buttons-container'>"
        "<button class='btn btn-blue' onclick='rebootDevice()'>Restart urządzenia</button>"
        "<button class='btn btn-red' onclick='factoryReset()'>Przywróć ustawienia fabryczne</button>"
        "</div>"
        "</div>"
    );

    // Dodaj skrypty JavaScript dla SSE i obsługi formularza
    String scripts = F(
        "<script>"
        "const evtSource = new EventSource('/events');"
        "evtSource.onmessage = function(event) {"
        "    const data = JSON.parse(event.data);"
        "    updateStatus(data);"
        "};"
        
        "function updateStatus(data) {"
        "    document.getElementById('waterLevel').textContent = data.waterLevel.toFixed(1);"
        "    document.getElementById('pumpState').textContent = data.pumpState ? 'Włączona' : 'Wyłączona';"
        "    document.getElementById('alarmState').textContent = data.alarmState ? 'Aktywny' : 'Nieaktywny';"
        "}"
        
        "function showChangePasswordModal() {"
        "    const modal = document.getElementById('passwordModal');"
        "    modal.style.display = 'block';"
        "}"
        
        "function submitConfig(form) {"
        "    const formData = new FormData(form);"
        "    fetch('/save', {"
        "        method: 'POST',"
        "        body: formData"
        "    })"
        "    .then(response => response.json())"
        "    .then(data => {"
        "        if (data.error) {"
        "            alert('Błąd: ' + data.error);"
        "        } else {"
        "            alert(data.message);"
        "            if (data.restart) {"
        "                setTimeout(() => { window.location.reload(); }, 5000);"
        "            }"
        "        }"
        "    })"
        "    .catch(error => {"
        "        alert('Wystąpił błąd podczas zapisywania konfiguracji');"
        "    });"
        "    return false;"
        "}"
        
        "function rebootDevice() {"
        "    if (confirm('Czy na pewno chcesz zrestartować urządzenie?')) {"
        "        fetch('/reboot', { method: 'POST' })"
        "        .then(() => {"
        "            alert('Urządzenie zostanie zrestartowane...');"
        "            setTimeout(() => { window.location.reload(); }, 5000);"
        "        });"
        "    }"
        "}"
        
        "function factoryReset() {"
        "    if (confirm('Czy na pewno chcesz przywrócić ustawienia fabryczne? Ta operacja jest nieodwracalna!')) {"
        "        fetch('/factory-reset', { method: 'POST' })"
        "        .then(() => {"
        "            alert('Przywracanie ustawień fabrycznych...');"
        "            setTimeout(() => { window.location.reload(); }, 5000);"
        "        });"
        "    }"
        "}"
        "</script>"
    );

    // Modal do zmiany hasła
    String passwordModal = F(
        "<div id='passwordModal' class='modal'>"
        "<div class='modal-content'>"
        "<h2>Zmiana hasła</h2>"
        "<form onsubmit='return submitConfig(this)' method='POST'>"
        "<table class='config-table'>"
        "<tr><td>Obecne hasło:</td><td><input type='password' name='currentPassword' required></td></tr>"
        "<tr><td>Nowe hasło:</td><td><input type='password' name='newPassword' required></td></tr>"
        "<tr><td>Powtórz nowe hasło:</td><td><input type='password' name='confirmPassword' required></td></tr>"
        "</table>"
        "<div class='buttons-container'>"
        "<button type='submit' class='btn btn-blue'>Zapisz</button>"
        "<button type='button' class='btn btn-red' onclick='document.getElementById(\"passwordModal\").style.display=\"none\"'>Anuluj</button>"
        "</div>"
        "</form>"
        "</div>"
        "</div>"
    );

    // Przygotuj formularze konfiguracyjne
    String configForms = F("<form onsubmit='return submitConfig(this)' method='POST'>");
    
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

    // Ustawienia MQTT
    configForms += F("<div class='section'>"
                     "<h2>Ustawienia MQTT</h2>"
                     "<table class='config-table'>"
                     "<tr><td>Serwer MQTT</td><td><input type='text' name='mqtt_server' value='");
    configForms += config.mqtt_server;
    configForms += F("'></td></tr>"
                     "<tr><td>Port MQTT</td><td><input type='number' name='mqtt_port' value='");
    configForms += String(config.mqtt_port);
    configForms += F("'></td></tr>"
                     "<tr><td>Użytkownik MQTT</td><td><input type='text' name='mqtt_user' value='");
    configForms += config.mqtt_user;
    configForms += F("'></td></tr>"
                     "<tr><td>Hasło MQTT</td><td><input type='password' name='mqtt_password' value='");
    configForms += config.mqtt_password;
    configForms += F("'></td></tr>"
                     "</table>"
                     "</div>");
    
    // Ustawienia zbiornika
    configForms += F("<div class='section'>"
                     "<h2>Ustawienia zbiornika</h2>"
                     "<table class='config-table'>"
                     "<tr><td>Poziom pusty [cm]</td><td><input type='number' name='tank_empty' value='");
    configForms += String(config.tank_empty);
    configForms += F("' min='0' max='500'></td></tr>"
                     "<tr><td>Poziom pełny [cm]</td><td><input type='number' name='tank_full' value='");
    configForms += String(config.tank_full);
    configForms += F("' min='0' max='500'></td></tr>"
                     "<tr><td>Poziom rezerwy [cm]</td><td><input type='number' name='reserve_level' value='");
    configForms += String(config.reserve_level);
    configForms += F("' min='0' max='500'></td></tr>"
                     "<tr><td>Średnica zbiornika [cm]</td><td><input type='number' name='tank_diameter' value='");
    configForms += String(config.tank_diameter);
    configForms += F("' min='0' max='1000'></td></tr>"
                     "</table>"
                     "</div>");
    
    // Ustawienia pompy
    configForms += F("<div class='section'>"
                     "<h2>Ustawienia pompy</h2>"
                     "<table class='config-table'>"
                     "<tr><td>Opóźnienie pompy [s]</td><td><input type='number' name='pump_delay' value='");
    configForms += String(config.pump_delay);
    configForms += F("' min='0' max='3600'></td></tr>"
                     "<tr><td>Czas pracy pompy [s]</td><td><input type='number' name='pump_work_time' value='");
    configForms += String(config.pump_work_time);
    configForms += F("' min='0' max='3600'></td></tr>"
                     "</table>"
                     "</div>");
    
    // Ustawienia dźwięku
    configForms += F("<div class='section'>"
                     "<h2>Ustawienia dźwięku</h2>"
                     "<table class='config-table'>"
                     "<tr><td>Dźwięk włączony</td><td><input type='checkbox' name='soundEnabled' ");
    configForms += config.soundEnabled ? "checked" : "";
    configForms += F("></td></tr>"
                     "</table>"
                     "</div>");

    configForms += F("<div class='section'>"
                     "<input type='submit' value='Zapisz ustawienia' class='btn btn-blue'>"
                     "</div></form>");

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
    html.replace("%CONFIG_FORMS%", configForms);
    html.replace("%SCRIPTS%", scripts);
    html.replace("%PASSWORD_MODAL%", passwordModal);

    // Sprawdź, czy wszystkie znaczniki zostały zastąpione
    if (html.indexOf('%') != -1) {
        DEBUG_PRINT("Uwaga: Niektóre znaczniki nie zostały zastąpione!");
        int pos = html.indexOf('%');
        DEBUG_PRINT(html.substring(pos - 20, pos + 20));
    }
    
    return html;
}

//
void handleRoot() {
    // Sprawdzenie autoryzacji
    if (!server.authenticate(config.www_username, config.www_password)) {
        return server.requestAuthentication();
    }

    String content = "<!DOCTYPE html><html><head>";
    content += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    content += "<title>HydroSense - Status</title>";
    content += "<style>";
    content += "body{font-family:Arial,sans-serif;margin:20px;background:#f0f0f0;}";
    content += ".container{max-width:800px;margin:0 auto;background:white;padding:20px;border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,0.1);}";
    content += ".status-box{background:#f8f8f8;padding:15px;border-radius:4px;margin:10px 0;}";
    content += ".status-item{margin:5px 0;padding:5px;border-bottom:1px solid #eee;}";
    content += ".success{color:green;}";
    content += ".warning{color:orange;}";
    content += ".error{color:red;}";
    content += ".btn{background:#007bff;color:white;padding:10px 20px;text-decoration:none;border-radius:4px;display:inline-block;margin:5px;}";
    content += ".btn-config{background:#28a745;}";
    content += "</style>";
    content += "</head><body><div class='container'>";

    // Nagłówek
    content += "<h1>HydroSense - Panel główny</h1>";

    // Status połączeń sieciowych
    content += "<div class='status-box'>";
    content += "<h2>Status sieci</h2>";
    content += "<div class='status-item'>AP: " + WiFi.softAPIP().toString() + "</div>";
    content += "<div class='status-item'>WiFi: ";
    if (WiFi.status() == WL_CONNECTED) {
        content += "<span class='success'>" + WiFi.localIP().toString() + " (" + WiFi.SSID() + ")</span>";
    } else {
        content += "<span class='warning'>Niepołączony</span>";
    }
    content += "</div>";
    
    // Status HA
    content += "<div class='status-item'>Home Assistant: ";
    if (config.ha_enabled && mqtt.isConnected()) {
        content += "<span class='success'>Połączony</span>";
    } else if (config.ha_enabled) {
        content += "<span class='warning'>Rozłączony</span>";
    } else {
        content += "<span class='warning'>Wyłączony</span>";
    }
    content += "</div></div>";

    // Status pomiarów
    content += "<div class='status-box'>";
    content += "<h2>Status systemu</h2>";
    content += "<div class='status-item'>Poziom wody: " + String(status.waterLevelPercent, 1) + " %</div>";
    content += "<div class='status-item'>Procent wypełnienia: " + String(status.waterLevelPercent) + "%</div>";
    content += "<div class='status-item'>Stan pompy: ";
    content += status.isPumpActive ? "<span class='warning'>Włączona</span>" : "<span class='success'>Wyłączona</span>";
    content += "</div>";
    content += "<div class='status-item'>Alarm wody: ";
    content += status.waterAlarmActive ? "<span class='error'>Aktywny</span>" : "<span class='success'>Nieaktywny</span>";
    content += "</div>";
    content += "<div class='status-item'>Dźwięk: " + String(config.soundEnabled ? "Włączony" : "Wyłączony") + "</div>";
    content += "</div>";

    // Przyciski nawigacyjne
    content += "<div style='margin-top:20px;text-align:center;'>";
    content += "<a href='/config' class='btn'>Konfiguracja</a>";
    content += "<a href='/update' class='btn btn-config'>Aktualizacja</a>";
    content += "</div>";

    // Informacje systemowe
    content += "<div class='status-box' style='font-size:0.8em;margin-top:20px;'>";
    content += "<div class='status-item'>Wersja: " + String(SOFTWARE_VERSION) + "</div>";
    content += "<div class='status-item'>Uptime: " + String(millis() / 1000) + " s</div>";
    content += "<div class='status-item'>Free Heap: " + String(ESP.getFreeHeap()) + " bytes</div>";
    content += "</div>";

    content += "</div></body></html>";

    server.send(200, "text/html", content);
}

//
void handleSave() {
    // Sprawdzenie metody HTTP
    if (server.method() != HTTP_POST) {
        server.send(405, "text/plain", "Method Not Allowed");
        return;
    }

    // Sprawdzenie autoryzacji
    if (!server.authenticate(config.webUser, config.webPassword)) {
        return server.requestAuthentication(BASIC_AUTH, "HydroSense", "Unauthorized");
    }

    // Kopia bezpieczeństwa konfiguracji
    Config oldConfig = config;
    bool needMqttReconnect = false;
    bool needRestart = false;
    String responseMessage;

    // Walidacja danych logowania WWW
    String newWebUser = server.arg("webUser");
    String newWebPassword = server.arg("webPassword");
    String newWebPasswordConfirm = server.arg("webPasswordConfirm");

    // Sprawdzenie nazwy użytkownika
    if (newWebUser.length() > 0) {
        if (newWebUser.length() >= sizeof(config.webUser)) {
            config = oldConfig;
            server.send(400, "application/json", "{\"error\": \"Nazwa użytkownika jest za długa\"}");
            return;
        }
        needRestart = true;
    }

    // Sprawdzenie haseł
    if (newWebPassword.length() > 0 || newWebPasswordConfirm.length() > 0) {
        if (newWebPassword.length() == 0 || newWebPasswordConfirm.length() == 0) {
            config = oldConfig;
            server.send(400, "application/json", "{\"error\": \"Oba pola hasła muszą być wypełnione\"}");
            return;
        }
        if (newWebPassword != newWebPasswordConfirm) {
            config = oldConfig;
            server.send(400, "application/json", "{\"error\": \"Podane hasła nie są identyczne\"}");
            return;
        }
        if (newWebPassword.length() >= sizeof(config.webPassword)) {
            config = oldConfig;
            server.send(400, "application/json", "{\"error\": \"Hasło jest za długie\"}");
            return;
        }
        needRestart = true;
    }

    // Zapisz poprzednie wartości MQTT do porównania
    String oldServer = config.mqtt_server;
    int oldPort = config.mqtt_port;
    String oldUser = config.mqtt_user;
    String oldPassword = config.mqtt_password;

    // Aktualizacja konfiguracji MQTT
    strlcpy(config.mqtt_server, server.arg("mqtt_server").c_str(), sizeof(config.mqtt_server));
    config.mqtt_port = server.arg("mqtt_port").toInt();
    strlcpy(config.mqtt_user, server.arg("mqtt_user").c_str(), sizeof(config.mqtt_user));
    strlcpy(config.mqtt_password, server.arg("mqtt_password").c_str(), sizeof(config.mqtt_password));
    
    // Aktualizacja konfiguracji zbiornika
    config.tank_full = server.arg("tank_full").toInt();
    config.tank_empty = server.arg("tank_empty").toInt();
    config.reserve_level = server.arg("reserve_level").toInt();
    config.tank_diameter = server.arg("tank_diameter").toInt();

    // Aktualizacja konfiguracji pompy
    config.pump_delay = server.arg("pump_delay").toInt();
    config.pump_work_time = server.arg("pump_work_time").toInt();

    // Walidacja wszystkich wartości
    if (!validateConfigValues()) {
        config = oldConfig;
        server.send(400, "application/json", "{\"error\": \"Nieprawidłowe wartości! Sprawdź wprowadzone dane.\"}");
        return;
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

    // Zapisz konfigurację
    if (!saveConfig()) {
        config = oldConfig;
        server.send(500, "application/json", "{\"error\": \"Błąd zapisu konfiguracji!\"}");
        return;
    }

    // Obsługa MQTT jeśli potrzebna
    if (needMqttReconnect) {
        if (mqtt.isConnected()) {
            mqtt.disconnect();
        }
        connectMQTT();
    }

    // Przygotuj odpowiedź JSON
    String response;
    if (needRestart) {
        response = "{\"success\": true, \"message\": \"Zapisano ustawienia! Urządzenie zostanie zrestartowane...\", \"restart\": true}";
    } else {
        response = "{\"success\": true, \"message\": \"Zapisano ustawienia!\", \"restart\": false}";
    }
    
    // Wyślij odpowiedź
    server.send(200, "application/json", response);
    status.needsUpdate = true;

    // Jeśli potrzebny restart, wykonaj go po wysłaniu odpowiedzi
    if (needRestart) {
        delay(1000);
        ESP.restart();
    }
}

//
void handleDoUpdate() {
    HTTPUpload& upload = server.upload();
    
    if(upload.status == UPLOAD_FILE_START) {
        if(!upload.filename.length()) {
            server.send(400, "text/plain", "Nie wybrano pliku!");
            return;
        }
        
        uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
        if(!Update.begin(maxSketchSpace)) {  // Zamiast UPDATE_SIZE_UNKNOWN używamy rzeczywistego rozmiaru
            server.send(500, "text/plain", "Błąd inicjalizacji aktualizacji!");
            return;
        }
        
    } else if(upload.status == UPLOAD_FILE_WRITE) {
        if(Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            server.send(500, "text/plain", "Błąd zapisu pliku!");
            return;
        }
        
        // Oblicz i wyślij postęp
        int progress = (upload.totalSize > 0) ? (upload.currentSize * 100) / upload.totalSize : 0;
        String eventData = "data: {\"type\":\"update\",\"progress\":" + String(progress) + "}\n\n";
        server.sendContent(eventData);
        
    } else if(upload.status == UPLOAD_FILE_END) {
        if(Update.end(true)) {
            server.send(200, "text/plain", "Aktualizacja zakończona pomyślnie!");
            delay(1000);
            ESP.restart();
        } else {
            server.send(500, "text/plain", "Błąd zakończenia aktualizacji!");
        }
    }
}

//
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

//
void handleChangePassword() {
    if (!server.hasArg("current") || !server.hasArg("new") || !server.hasArg("confirm")) {
        server.send(400, "text/plain", "Brak wymaganych parametrów");
        return;
    }

    String currentPassword = server.arg("current");
    String newPassword = server.arg("new");
    String confirmPassword = server.arg("confirm");

    // Zmiana z webControlPassword na webPassword
    if (currentPassword != config.webPassword) {
        server.send(401, "text/plain", "Nieprawidłowe obecne hasło");
        return;
    }

    if (newPassword != confirmPassword) {
        server.send(400, "text/plain", "Nowe hasła nie są zgodne");
        return;
    }

    if (newPassword.length() < 8) {
        server.send(400, "text/plain", "Hasło musi mieć co najmniej 8 znaków");
        return;
    }

    // Zmiana z webControlPassword na webPassword
    strlcpy(config.webPassword, newPassword.c_str(), sizeof(config.webPassword));
    
    if (!saveConfig()) {
        server.send(500, "text/plain", "Błąd zapisu konfiguracji");
        return;
    }

    server.send(200, "text/plain", "Hasło zostało zmienione");
    delay(500);
    ESP.restart();
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
void sendSerialMessage(String message) {
    Serial.println(message);
}

bool handleAuth() {
    if (!server.authenticate(config.www_username, config.www_password)) {
        server.requestAuthentication();
        return false;
    }
    return true;
}

// ---------------------- GŁÓWNE FUNKCJE PROGRAMU ---------------------

// Inicjalizacja sprzętu i usług
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

// Główna pętla programu
void loop() {
    unsigned long currentMillis = millis();  // Pobierz bieżącą wartość millis()

    // KRYTYCZNE OPERACJE CZASOWE
    handleMillisOverflow();  // Obsługa przepełnienia millis()
    updatePump();  // Aktualizacja stanu pompy
    ESP.wdtFeed();  // Reset watchdog timer ESP
    handleMillisOverflow();
    yield();  // Umożliwienie przetwarzania innych zadań

    // BEZPOŚREDNIA INTERAKCJA
    handleButton();  // Obsługa naciśnięcia przycisku
    checkAlarmConditions();  // Sprawdzenie warunków alarmowych
    server.handleClient();  // Obsługa serwera WWW

    // POMIARY I AKTUALIZACJE
    if (currentMillis - timers.lastMeasurement >= MEASUREMENT_INTERVAL) {
        updateWaterLevel();  // Aktualizacja poziomu wody
        timers.lastMeasurement = currentMillis;  // Aktualizacja znacznika czasu ostatniego pomiaru
        status.needsUpdate = true;
    }

    // Jeśli status wymaga aktualizacji, wyślij nowe dane przez SSE
    if (status.needsUpdate) {
        handleEvents();
        status.needsUpdate = false;
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

//----------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------
