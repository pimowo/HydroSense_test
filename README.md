# HydroSense_mini 🌊

## O Projekcie
HydroSense_mini to inteligentny system automatycznego uzupełniania wody zaprojektowany dla akwarystów. Wykorzystując ESP8266, system zapewnia precyzyjną kontrolę poziomu wody w akwarium i zbiornika dolewki z pełną integracją z Home Assistant przez MQTT.

## Funkcje
- 📊 Pomiar poziomu wody z dokładnością do 1mm (HC-SR04)
- 🚰 Automatyczne uzupełnianie wody w akwarium
- 🏠 Pełna integracja z Home Assistant przez MQTT
- ⚡ Zaawansowane zabezpieczenia pompy:
  - Maksymalny czas pracy (5 min)
  - Opóźnienie ponownego startu (1 min)
  - Blokada bezpieczeństwa
- 🔧 Tryb serwisowy
- 🔔 Konfigurowalne alarmy dźwiękowe
- 💾 Trwała pamięć ustawień (EEPROM)

## Wymagania Sprzętowe

### Komponenty
- ESP8266 (NodeMCU v3)
- Czujnik ultradźwiękowy HC-SR04
- Czujnik poziomu wody (pływakowy)
- Przekaźnik do sterowania pompą
- Buzzer aktywny
- Przycisk taktowy
- Zasilacz 5V/1A

### Podłączenie Pinów
| Komponent | Pin ESP8266 |
|-----------|-------------|
| HC-SR04 TRIG | D6 (GPIO12) |
| HC-SR04 ECHO | D7 (GPIO13) |
| Czujnik poziomu | D5 (GPIO14) |
| Przekaźnik pompy | D1 (GPIO5) |
| Buzzer | D2 (GPIO4) |
| Przycisk | D3 (GPIO0) |

## Wymagania Programowe
- Arduino IDE
- Biblioteki:
  - ESP8266WiFi
  - ArduinoHA
  - PubSubClient
  - EEPROM

## Konfiguracja
1. Skopiuj plik `HydroSense_mini.ino`
2. Uzupełnij dane w sekcji konfiguracyjnej:
   ```cpp
   const char* WIFI_SSID = "twoja_siec";
   const char* WIFI_PASSWORD = "twoje_haslo";
   const char* MQTT_SERVER = "ip_home_assistant";
   const char* MQTT_USER = "uzytkownik";
   const char* MQTT_PASSWORD = "haslo";
Dostosuj parametry zbiornika:
C++
const int DISTANCE_WHEN_FULL = 65;   // mm
const int DISTANCE_WHEN_EMPTY = 510;  // mm
const int DISTANCE_RESERVE = 450;     // mm
const int TANK_DIAMETER = 150;        // mm
Funkcje w Home Assistant
Sensory
📏 Pomiar odległości (mm)
💧 Poziom wody (%)
🌊 Objętość wody (L)
💪 Status pompy (ON/OFF)
⚠️ Alarm braku wody
⚡ Alarm rezerwy
Przełączniki
🔧 Tryb serwisowy
🔔 Włącznik dźwięku
⚠️ Reset alarmu pompy
Obsługa
Przycisk Fizyczny
Krótkie naciśnięcie: włącza/wyłącza tryb serwisowy
Długie naciśnięcie (>1s): kasuje blokadę bezpieczeństwa pompy
Zabezpieczenia
Maksymalny czas pracy pompy (5 min)
Opóźnienie ponownego startu (1 min)
Watchdog programowy
Blokada bezpieczeństwa po przekroczeniu czasu pracy
Kontrola poziomu w akwarium
Status Projektu
🚧 Wersja: 13.11.2024

Podstawowa funkcjonalność
Integracja z Home Assistant
System alarmów
Interfejs Web
Aktualizacje OTA
Autor
@pimowo
PMW 
pimowo@gmail.com

Licencja
MIT License - możesz swobodnie używać, modyfikować i rozpowszechniać kod.

Wsparcie
W razie problemów lub pytań, utwórz Issue w repozytorium GitHub.
