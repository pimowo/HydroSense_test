# HydroSense - System Inteligentnego Monitorowania Wody

## Opis Projektu
HydroSense to zaawansowany system kontroli i monitorowania poziomu wody oparty na platformie ESP8266. System oferuje integrację z Home Assistant poprzez protokół MQTT, umożliwiając zdalne monitorowanie i sterowanie.

## Główne Funkcje
- Pomiar poziomu wody za pomocą czujnika ultradźwiękowego
- Automatyczne sterowanie pompą
- Integracja z Home Assistant
- Bezprzewodowa aktualizacja oprogramowania (OTA)
- Wbudowany interfejs WWW
- System alarmowy z powiadomieniami dźwiękowymi

## Komponenty Sprzętowe
- ESP8266
- Czujnik ultradźwiękowy (HC-SR04)
- Czujnik poziomu wody
- Pompa wodna
- Buzzer alarmowy
- Przycisk reset alarmu

## Wymagania Systemowe
- Sieć WiFi
- Serwer MQTT (opcjonalnie)
- Home Assistant (opcjonalnie)

## Instalacja
1. Sklonuj repozytorium
2. Skonfiguruj ustawienia WiFi i MQTT w pliku konfiguracyjnym
3. Wgraj program do ESP8266
4. Podłącz komponenty zgodnie ze schematem

## Konfiguracja
System można skonfigurować poprzez:
- Interfejs WWW
- Panel Home Assistant
- Pamięć EEPROM dla trwałych ustawień

## Parametry Konfiguracyjne
- Czas pracy pompy
- Opóźnienia systemowe
- Wymiary zbiornika
- Progi alarmowe
- Ustawienia powiadomień

## Autor
- [@pimowo](https://github.com/pimowo)

## Licencja
Ten projekt jest udostępniany na licencji MIT.

## Status Projektu
🚧 W trakcie rozwoju

## Współpraca
Zachęcamy do zgłaszania problemów i propozycji ulepszeń poprzez GitHub Issues.
