// Biblioteki związane z obsługą komunikacji
#include <ESP8266WiFi.h>  // Obsługa WiFi dla ESP8266
#include <PubSubClient.h>  // MQTT - protokół komunikacji z serwerem MQTT
// Biblioteka do integracji z Home Assistant
#include <ArduinoHA.h>  // Ułatwia komunikację z Home Assistant
// Obsługa aktualizacji OTA (Over-The-Air)
#include <ArduinoOTA.h>  // Aktualizacje firmware bezpośrednio przez WiFi
// Obsługa pamięci EEPROM, wykorzystywana do przechowywania danych na stałe
#include <EEPROM.h>  // Przechowuje dane między restartami urządzenia
// Biblioteka CRC32 - używana do weryfikacji integralności danych
#include <CRC32.h>  // Zapewnia spójność danych za pomocą sumy kontrolnej

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
    //unsigned long lastSuccessfulMQTTPublish = 0; // Czas ostatniej udanej publikacji MQTT
} status;

// Struktura dla obsługi przycisku
struct ButtonState {
    //bool currentState = HIGH; // Aktualny stan przycisku
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

// Zapis konfiguracji
void saveConfig() {
    config.version = CONFIG_VERSION;
    config.checksum = calculateChecksum(config);
    
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.put(0, config);
    bool success = EEPROM.commit();
    
    if (success) {
        Serial.println(F("Konfiguracja zapisana"));
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
    
    Serial.println(F("Konfiguracja wczytana"));
    return true;
}

// Ustawienia domyślne
void setDefaultConfig() {
    config.version = CONFIG_VERSION;
    config.soundEnabled = true;  // Domyślnie dźwięk włączony
    
    saveConfig();
    Serial.println(F("Utworzono domyślną konfigurację"));
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
    // Aktualizuj stan w pamięci RAM
    status.soundEnabled = state;
    
    // Zapisz stan w konfiguracji EEPROM
    config.soundEnabled = state;
    saveConfig();  // Zapisz do EEPROM
    
    // Aktualizuj stan w Home Assistant
    switchSound.setState(state);
    
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
    connectMQTT();

    // Konfiguracja urządzenia dla Home Assistant
    device.setName("HydroSense");                  // Nazwa urządzenia
    device.setModel("HS ESP8266");                 // Model urządzenia
    device.setManufacturer("PMW");                 // Producent
    device.setSoftwareVersion("13.11.24");         // Wersja oprogramowania

    // Konfiguracja sensorów pomiarowych w HA
    sensorDistance.setName("Pomiar odległości");
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

    sensorReserve.setName("Rezerwa wody");
    sensorReserve.setIcon("mdi:alert-outline");    // Ikona ostrzeżenia
    
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

    // Wczytaj konfigurację z EEPROM
    if (loadConfig()) {
        // Synchronizuj stan dźwięku z wczytanej konfiguracji
        status.soundEnabled = config.soundEnabled;
        switchSound.setState(config.soundEnabled); // Aktualizuj stan w HA
    }
    
    // Wykonaj pierwszy pomiar i ustaw stany
    float initialDistance = measureDistance();
    
    // Ustaw początkowe stany na podstawie pomiaru
    status.waterAlarmActive = (initialDistance >= DISTANCE_WHEN_EMPTY);
    status.waterReserveActive = (initialDistance >= DISTANCE_RESERVE);
    
    // Wymuś stan OFF na początku
    sensorAlarm.setValue("OFF");
    sensorReserve.setValue("OFF");
    mqtt.loop();
    
    // Ustawienie końcowych stanów i wysyłka do HA
    sensorAlarm.setValue(status.waterAlarmActive ? "ON" : "OFF");
    sensorReserve.setValue(status.waterReserveActive ? "ON" : "OFF");
    mqtt.loop();
    
    // Konfiguracja OTA (Over-The-Air) dla aktualizacji oprogramowania
    ArduinoOTA.setHostname("HydroSense");  // Ustaw nazwę urządzenia
    ArduinoOTA.setPassword("hydrosense");  // Ustaw hasło dla OTA
    ArduinoOTA.begin();  // Uruchom OTA

    // Prosta melodia powitalna
    tone(PIN_BUZZER, 1397, 100);  // F6
    delay(150);
    tone(PIN_BUZZER, 1568, 100);  // G6
    delay(150);
    tone(PIN_BUZZER, 1760, 150);  // A6
    delay(200);

    Serial.println("Setup zakończony pomyślnie!");
}

void loop() {
    unsigned long currentMillis = millis();
    static unsigned long lastMQTTRetry = 0;
    static unsigned long lastMeasurement = 0;

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
           
    // Pomiar co MEASUREMENT_INTERVAL
    if (millis() - lastMeasurement >= MEASUREMENT_INTERVAL) {
        updateWaterLevel();  // Ta funkcja zawiera wszystko co potrzebne
        lastMeasurement = millis();
    }

    updatePump();
    yield();

    // Obsługa OTA (aktualizacji oprogramowania przez sieć)
    ArduinoOTA.handle();
}
