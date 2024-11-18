// --- Biblioteki

#include <Arduino.h>  // Podstawowa biblioteka Arduino zawierająca funkcje rdzenia
#include <ArduinoHA.h>  // Integracja z Home Assistant przez protokół MQTT
#include <ArduinoOTA.h>  // Aktualizacja oprogramowania przez sieć WiFi (Over-The-Air)
#include <ESP8266WiFi.h>  // Biblioteka WiFi dedykowana dla układu ESP8266
//#include <EEPROM.h>  // Dostęp do pamięci nieulotnej EEPROM
#include <ESP8266WebServer.h>
#include <DNSServer.h>

#include "ConfigManager.h"

ConfigManager configManager;

// --- Definicje stałych i zmiennych globalnych

// Konfiguracja WiFi i MQTT
// const char* WIFI_SSID = "pimowo";                  // Nazwa sieci WiFi
// const char* WIFI_PASSWORD = "ckH59LRZQzCDQFiUgj";  // Hasło do sieci WiFi
// const char* MQTT_SERVER = "192.168.1.14";          // Adres IP serwera MQTT (Home Assistant)
// const char* MQTT_USER = "hydrosense";              // Użytkownik MQTT
// const char* MQTT_PASSWORD = "hydrosense";          // Hasło MQTT

// Stałe dla trybu AP
const byte DNS_PORT = 53;
const char* AP_SSID = "HydroSense";
const char* AP_PASSWORD = "hydrosense";

// Obiekty dla serwera webowego i DNS
ESP8266WebServer webServer(80);
DNSServer dnsServer;

// Konfiguracja pinów ESP8266
#define PIN_ULTRASONIC_TRIG D6  // Pin TRIG czujnika ultradźwiękowego
#define PIN_ULTRASONIC_ECHO D7  // Pin ECHO czujnika ultradźwiękowego

#define PIN_WATER_LEVEL D5  // Pin czujnika poziomu wody w akwarium
#define POMPA_PIN D1  // Pin sterowania pompą
#define BUZZER_PIN D2  // Pin buzzera do alarmów dźwiękowych
#define PRZYCISK_PIN D3  // Pin przycisku do kasowania alarmów

// Stałe czasowe (wszystkie wartości w milisekundach)
const unsigned long ULTRASONIC_TIMEOUT = 50;       // Timeout pomiaru czujnika ultradźwiękowego
const unsigned long MEASUREMENT_INTERVAL = 15000;  // Interwał między pomiarami
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
#define DEBUG 0

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
const int SENSOR_AVG_SAMPLES = 5;  // Liczba próbek do uśrednienia pomiaru
const int PUMP_DELAY = 5;  // Opóźnienie uruchomienia pompy (sekundy)
const int PUMP_WORK_TIME = 60;  // Czas pracy pompy

float aktualnaOdleglosc = 0;  // Aktualny dystans
float lastFilteredDistance = 0;
const float EMA_ALPHA = 0.2;        // Współczynnik filtru EMA (0.0-1.0)
const int MEASUREMENT_DELAY = 30;    // Opóźnienie między pomiarami w ms
const int VALID_MARGIN = 20;        // Margines błędu pomiaru (mm)

unsigned long ostatniCzasDebounce = 0;  // Ostatni czas zmiany stanu przycisku

// Obiekty do komunikacji
WiFiClient client;  // Klient połączenia WiFi
HADevice device("HydroSense");  // Definicja urządzenia dla Home Assistant
HAMqtt mqtt(client, device);  // Klient MQTT dla Home Assistant

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
HASwitch switchPumpAlarm("pump_alarm");                        // Przełącznik resetowania blokady pompy
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

void setupAP() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    
    // Konfiguracja DNS dla captive portal
    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
    
    Serial.print(F("AP Started. IP: "));
    Serial.println(WiFi.softAPIP());
}

void setupWebServer() {
    // Strona główna
    webServer.on("/", HTTP_GET, handleRoot);
    
    // Endpointy API
    webServer.on("/api/config", HTTP_GET, handleGetConfig);
    webServer.on("/api/config", HTTP_POST, handleSaveConfig);
    webServer.on("/api/wifi/scan", HTTP_GET, handleWiFiScan);
    
    // Obsługa nieznanych ścieżek
    webServer.onNotFound([]() {
        if (!handleCaptivePortal()) {
            webServer.send(404, "text/plain", "Not Found");
        }
    });
    
    webServer.begin();
}

bool handleCaptivePortal() {
    if (!isIp(webServer.hostHeader())) {
        Serial.println(F("Redirect to captive portal"));
        webServer.sendHeader("Location", String("http://") + toStringIp(WiFi.softAPIP()), true);
        webServer.send(302, "text/plain", "");
        webServer.client().stop();
        return true;
    }
    return false;
}

void handleRoot() {
    String html = F("<!DOCTYPE html>"
        "<html>"
        "<head>"
        "<meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<title>HydroSense Configuration</title>"
        "<style>"
        "body { font-family: Arial, sans-serif; margin: 0; padding: 20px; background: #f0f0f0; }"
        ".container { max-width: 800px; margin: 0 auto; background: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }"
        ".section { margin-bottom: 20px; padding: 15px; border: 1px solid #ddd; border-radius: 4px; }"
        ".section h2 { margin-top: 0; color: #333; }"
        "input, select { width: calc(100% - 20px); padding: 8px; margin: 5px 0; border: 1px solid #ddd; border-radius: 4px; }"
        "label { display: block; margin-top: 10px; color: #666; }"
        "button { background-color: #4CAF50; color: white; padding: 10px 15px; border: none; border-radius: 4px; cursor: pointer; margin-right: 10px; }"
        "button:hover { background-color: #45a049; }"
        ".button-group { margin-top: 20px; }"
        ".status { margin-top: 10px; padding: 10px; border-radius: 4px; }"
        ".success { background-color: #dff0d8; color: #3c763d; }"
        ".error { background-color: #f2dede; color: #a94442; }"
        "</style>"
        "</head>"
        "<body>"
        "<div class='container'>"
        "<h1>HydroSense Configuration</h1>"
        
        "<div class='section' id='network-section'>"
        "<h2>Network Settings</h2>"
        "<label>WiFi SSID:</label>"
        "<input type='text' id='wifi_ssid'>"
        "<button onclick='scanWiFi()'>Scan Networks</button>"
        "<select id='wifi_networks' style='display:none' onchange='selectNetwork()'></select>"
        "<label>WiFi Password:</label>"
        "<input type='password' id='wifi_password'>"
        "<label>MQTT Server:</label>"
        "<input type='text' id='mqtt_server'>"
        "<label>MQTT User:</label>"
        "<input type='text' id='mqtt_user'>"
        "<label>MQTT Password:</label>"
        "<input type='password' id='mqtt_password'>"
        "</div>"
        
        "<div class='section' id='tank-section'>"
        "<h2>Tank Settings</h2>"
        "<label>Tank Full Level (mm):</label>"
        "<input type='number' id='tank_full'>"
        "<label>Tank Empty Level (mm):</label>"
        "<input type='number' id='tank_empty'>"
        "<label>Reserve Level (mm):</label>"
        "<input type='number' id='reserve_level'>"
        "<label>Hysteresis (mm):</label>"
        "<input type='number' id='hysteresis'>"
        "<label>Tank Diameter (mm):</label>"
        "<input type='number' id='tank_diameter'>"
        "</div>"
        
        "<div class='section' id='pump-section'>"
        "<h2>Pump Settings</h2>"
        "<label>Pump Delay (seconds):</label>"
        "<input type='number' id='pump_delay'>"
        "<label>Pump Work Time (seconds):</label>"
        "<input type='number' id='pump_work_time'>"
        "</div>"
        
        "<div class='button-group'>"
        "<button onclick='saveConfig()'>Save Configuration</button>"
        "<button onclick='loadConfig()'>Reload Configuration</button>"
        "<button onclick='restartDevice()' style='background-color: #d9534f;'>Restart Device</button>"
        "</div>"
        
        "<div id='status' class='status' style='display:none;'></div>"
        "</div>"
        
        "<script>"
        "function showStatus(message, isError = false) {"
        "    const status = document.getElementById('status');"
        "    status.className = 'status ' + (isError ? 'error' : 'success');"
        "    status.textContent = message;"
        "    status.style.display = 'block';"
        "    setTimeout(() => status.style.display = 'none', 3000);"
        "}"

        "function loadConfig() {"
        "    fetch('/api/config')"
        "        .then(response => response.json())"
        "        .then(data => {"
        "            if (data.network) {"
        "                document.getElementById('wifi_ssid').value = data.network.wifi_ssid || '';"
        "                document.getElementById('mqtt_server').value = data.network.mqtt_server || '';"
        "                document.getElementById('mqtt_user').value = data.network.mqtt_user || '';"
        "            }"
        "            if (data.tank) {"
        "                document.getElementById('tank_full').value = data.tank.full || '';"
        "                document.getElementById('tank_empty').value = data.tank.empty || '';"
        "                document.getElementById('reserve_level').value = data.tank.reserve_level || '';"
        "                document.getElementById('hysteresis').value = data.tank.hysteresis || '';"
        "                document.getElementById('tank_diameter').value = data.tank.diameter || '';"
        "            }"
        "            if (data.pump) {"
        "                document.getElementById('pump_delay').value = data.pump.delay || '';"
        "                document.getElementById('pump_work_time').value = data.pump.work_time || '';"
        "            }"
        "            showStatus('Configuration loaded');"
        "        })"
        "        .catch(error => showStatus('Error loading configuration: ' + error, true));"
        "}"

        "function saveConfig() {"
        "    const config = {"
        "        network: {"
        "            wifi_ssid: document.getElementById('wifi_ssid').value,"
        "            wifi_password: document.getElementById('wifi_password').value,"
        "            mqtt_server: document.getElementById('mqtt_server').value,"
        "            mqtt_user: document.getElementById('mqtt_user').value,"
        "            mqtt_password: document.getElementById('mqtt_password').value"
        "        },"
        "        tank: {"
        "            full: parseInt(document.getElementById('tank_full').value),"
        "            empty: parseInt(document.getElementById('tank_empty').value),"
        "            reserve_level: parseInt(document.getElementById('reserve_level').value),"
        "            hysteresis: parseInt(document.getElementById('hysteresis').value),"
        "            diameter: parseInt(document.getElementById('tank_diameter').value)"
        "        },"
        "        pump: {"
        "            delay: parseInt(document.getElementById('pump_delay').value),"
        "            work_time: parseInt(document.getElementById('pump_work_time').value)"
        "        }"
        "    };"
        
        "    fetch('/api/config', {"
        "        method: 'POST',"
        "        headers: {'Content-Type': 'application/json'},"
        "        body: JSON.stringify(config)"
        "    })"
        "    .then(response => response.text())"
        "    .then(result => showStatus('Configuration saved'))"
        "    .catch(error => showStatus('Error saving configuration: ' + error, true));"
        "}"

        "function scanWiFi() {"
        "    const button = event.target;"
        "    const select = document.getElementById('wifi_networks');"
        "    button.disabled = true;"
        "    button.textContent = 'Scanning...';"
        "    select.style.display = 'none';"
        
        "    fetch('/api/wifi/scan')"
        "        .then(response => response.json())"
        "        .then(data => {"
        "            select.innerHTML = '';"
        "            select.appendChild(new Option('Select network...', ''));"
        "            data.networks.sort((a, b) => b.rssi - a.rssi)"
        "                .forEach(network => {"
        "                    const option = new Option(network.ssid + ' (' + network.rssi + 'dBm)', network.ssid);"
        "                    select.appendChild(option);"
        "                });"
        "            select.style.display = 'block';"
        "            button.disabled = false;"
        "            button.textContent = 'Scan Networks';"
        "        })"
        "        .catch(error => {"
        "            showStatus('Error scanning networks: ' + error, true);"
        "            button.disabled = false;"
        "            button.textContent = 'Scan Networks';"
        "        });"
        "}"

        "function selectNetwork() {"
        "    const select = document.getElementById('wifi_networks');"
        "    const ssidInput = document.getElementById('wifi_ssid');"
        "    ssidInput.value = select.value;"
        "}"

        "function restartDevice() {"
        "    if (confirm('Are you sure you want to restart the device?')) {"
        "        showStatus('Restarting device...');"
        "        fetch('/restart', { method: 'POST' })"
        "            .then(() => showStatus('Device is restarting...'));"
        "    }"
        "}"

        "// Load configuration when page loads"
        "document.addEventListener('DOMContentLoaded', loadConfig);"
        "</script>"
        "</body>"
        "</html>");
    
    webServer.send(200, "text/html", html);
}

void setupWebServer() {
    // Strona główna
    webServer.on("/", HTTP_GET, handleRoot);
    
    // Endpointy API
    webServer.on("/api/config", HTTP_GET, handleGetConfig);
    webServer.on("/api/config", HTTP_POST, handleSaveConfig);
    webServer.on("/api/wifi/scan", HTTP_GET, handleWiFiScan);
    
    // Obsługa nieznanych ścieżek
    webServer.onNotFound([]() {
        if (!handleCaptivePortal()) {
            webServer.send(404, "text/plain", "Not Found");
        }
    });
    
    webServer.begin();
}

bool handleCaptivePortal() {
    if (!isIp(webServer.hostHeader())) {
        Serial.println(F("Redirect to captive portal"));
        webServer.sendHeader("Location", String("http://") + toStringIp(WiFi.softAPIP()), true);
        webServer.send(302, "text/plain", "");
        webServer.client().stop();
        return true;
    }
    return false;
}

void handleGetConfig() {
    DynamicJsonDocument doc(1024);
    
    // Pobierz aktualną konfigurację
    ConfigManager::NetworkConfig& networkConfig = configManager.getNetworkConfig();
    ConfigManager::TankConfig& tankConfig = configManager.getTankConfig();
    ConfigManager::PumpConfig& pumpConfig = configManager.getPumpConfig();
    
    JsonObject network = doc.createNestedObject("network");
    network["wifi_ssid"] = networkConfig.wifi_ssid;
    // Nie wysyłamy hasła WiFi ze względów bezpieczeństwa
    network["mqtt_server"] = networkConfig.mqtt_server;
    network["mqtt_user"] = networkConfig.mqtt_user;
    
    JsonObject tank = doc.createNestedObject("tank");
    tank["full"] = tankConfig.full;
    tank["empty"] = tankConfig.empty;
    tank["reserve_level"] = tankConfig.reserve_level;
    tank["hysteresis"] = tankConfig.hysteresis;
    tank["diameter"] = tankConfig.diameter;
    
    JsonObject pump = doc.createNestedObject("pump");
    pump["delay"] = pumpConfig.delay;
    pump["work_time"] = pumpConfig.work_time;
    
    String response;
    serializeJson(doc, response);
    webServer.send(200, "application/json", response);
}

void handleSaveConfig() {
    if (webServer.hasArg("plain")) {
        DynamicJsonDocument doc(1024);
        DeserializationError error = deserializeJson(doc, webServer.arg("plain"));
        
        if (error) {
            webServer.send(400, "text/plain", "Invalid JSON");
            return;
        }
        
        // Aktualizacja konfiguracji
        ConfigManager::NetworkConfig& networkConfig = configManager.getNetworkConfig();
        ConfigManager::TankConfig& tankConfig = configManager.getTankConfig();
        ConfigManager::PumpConfig& pumpConfig = configManager.getPumpConfig();
        
        if (doc.containsKey("network")) {
            JsonObject network = doc["network"];
            if (network.containsKey("wifi_ssid")) 
                networkConfig.wifi_ssid = network["wifi_ssid"].as<String>();
            if (network.containsKey("wifi_password")) 
                networkConfig.wifi_password = network["wifi_password"].as<String>();
            if (network.containsKey("mqtt_server")) 
                networkConfig.mqtt_server = network["mqtt_server"].as<String>();
            if (network.containsKey("mqtt_user")) 
                networkConfig.mqtt_user = network["mqtt_user"].as<String>();
            if (network.containsKey("mqtt_password")) 
                networkConfig.mqtt_password = network["mqtt_password"].as<String>();
        }
        
        // Podobnie dla pozostałych sekcji...
        
        if (configManager.saveConfig()) {
            webServer.send(200, "text/plain", "Configuration saved");
        } else {
            webServer.send(500, "text/plain", "Failed to save configuration");
        }
    } else {
        webServer.send(400, "text/plain", "No data received");
    }
}

void handleWiFiScan() {
    DynamicJsonDocument doc(1024);
    JsonArray networks = doc.createNestedArray("networks");
    
    int n = WiFi.scanNetworks();
    for (int i = 0; i < n; ++i) {
        JsonObject network = networks.createNestedObject();
        network["ssid"] = WiFi.SSID(i);
        network["rssi"] = WiFi.RSSI(i);
        network["encryption"] = WiFi.encryptionType(i);
    }
    
    String response;
    serializeJson(doc, response);
    webServer.send(200, "application/json", response);
}

// Funkcje pomocnicze
bool isIp(String str) {
    for (int i = 0; i < str.length(); i++) {
        int c = str.charAt(i);
        if (c != '.' && (c < '0' || c > '9')) {
            return false;
        }
    }
    return true;
}

String toStringIp(IPAddress ip) {
    String res = "";
    for (int i = 0; i < 3; i++) {
        res += String((ip >> (8 * i)) & 0xFF) + ".";
    }
    res += String(((ip >> 8 * 3)) & 0xFF);
    return res;
}

webServer.on("/restart", HTTP_POST, []() {
    webServer.send(200, "text/plain", "Restarting...");
    delay(1000);
    ESP.restart();
});

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

// Sygnał potwierdzenia
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
    if (status.isPumpActive && (currentMillis - status.pumpStartTime > PUMP_WORK_TIME * 1000)) {
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
        playConfirmationSound();  // Sygnał potwierdzenia zmiany trybu
        status.pumpSafetyLock = false;  // Wyłącz blokadę bezpieczeństwa pompy
        switchPumpAlarm.setState(false);  // Aktualizuj stan przełącznika w HA na OFF        
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
    
    switchPumpAlarm.setName("Alarm pompy");
    switchPumpAlarm.setIcon("mdi:alert");               // Ikona alarmu
    switchPumpAlarm.onCommand(onPumpAlarmCommand);      // Funkcja obsługi zmiany stanu
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
                DEBUG_PRINT(F("Echo start timeout"));
                validEcho = false;
                break;
            }
        }
        
        if (validEcho) {
            startTime = micros();
            
            // Czekaj na koniec echa
            while (digitalRead(PIN_ULTRASONIC_ECHO) == HIGH) {
                if (micros() > timeout) {
                    DEBUG_PRINT(F("Echo end timeout"));
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
                    DEBUG_PRINT(F("Distance out of range: "));
                    DEBUG_PRINT(distance);
                }
            }
        }

        delay(MEASUREMENT_DELAY);
    }

    // Sprawdź czy mamy wystarczająco dużo poprawnych pomiarów
    if (validCount < (SENSOR_AVG_SAMPLES / 2)) {
        DEBUG_PRINT(F("Too few valid measurements: "));
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
                    playConfirmationSound();  // Sygnał potwierdzenia zmiany trybu
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
    
    Serial.printf("Zmieniono stan dźwięku na: %s\n", state ? "WŁĄCZONY" : "WYŁĄCZONY");
}

// --- Setup
void setup() {
    ESP.wdtEnable(WATCHDOG_TIMEOUT);  // Aktywacja watchdoga
    Serial.begin(115200);  // Inicjalizacja portu szeregowego
    DEBUG_PRINT("\nStarting HydroSense...");  // Komunikat startowy
    
    // Inicjalizacja managera konfiguracji
    if (!configManager.begin()) {
        Serial.println(F("Failed to initialize configuration"));
        // Możesz dodać tutaj obsługę błędów
    }
    
    // Przykład dostępu do konfiguracji
    ConfigManager::TankConfig& tankConfig = configManager.getTankConfig();
    // Używamy tankConfig.full zamiast TANK_FULL itd.

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

    dnsServer.processNextRequest();
    webServer.handleClient();

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
