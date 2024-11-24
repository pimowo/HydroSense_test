# HydroSense wersja testowa

HydroSense to inteligentny system monitorowania i kontroli zbiornika wody oparty na ESP8266, zintegrowany z Home Assistant. System umożliwia zdalne monitorowanie poziomu wody, automatyczne sterowanie pompą oraz obsługę alarmów.

## 🌟 Możliwości systemu

- 📊 Monitorowanie poziomu wody w czasie rzeczywistym
- 💧 Automatyczne sterowanie pompą z zabezpieczeniami
- 🚨 Wielopoziomowy system alarmowy
- 🌐 Pełna integracja z Home Assistant przez MQTT
- 🌐 Interfejs webowy do konfiguracji
- 📶 Praca w trybie AP (konfiguracja Wi-Fi) lub Client (Konfiguracja urządzenia i normalna praca)
- 💾 Przechowywanie konfiguracji w pamięci EEPROM

## 🛠️ Komponenty sprzętowe

### Wymagane

- ⚙️ ESP8266 (Wemos D1 MINI lub kompatybilny)
- 🎛️ Czujnik ultradźwiękowy JSN-SR04T
- 🔌 Przekaźnik sterujący pompą
- 🔊 Brzęczyk do sygnalizacji alarmów
- 🔘 Przycisk fizyczny do resetowania alarmów i przęłączania trybu "Serwis"

## 🚀 Instalacja

1. Sklonuj repozytorium:

   ```bash
   git clone https://github.com/pimowo/HydroSense.git
   ```

2. Potrzebne biblioteki Arduino:

   - ArduinoHA (Home Assistant)
   - ArduinoOTA (aktualizacja przez Wi-Fi)
   - ESP8266WiFi
   - EEPROM (zapis konfiguracji)
   - WiFiManager (połączenie Wi-Fi)

3. W Arduino IDE:

   - Wybierz płytkę: "Wemod D1 MINI"
   - Wybierz port szeregowy
   - Wgraj program do ESP8266
   - po pierwszym programowaniu aktualizacje przez OTA (Wi-Fi)

## 🏁 Pierwsze uruchomienie

1. Po pierwszym uruchomieniu, urządzenie utworzy AP:
  - SSID: HydroSense
  - Hasło: hydrosense
2. Połącz się z tą siecią
3. Otwórz przeglądarkę i wpisz adres: http://192.168.4.1
4. Skonfiguruj:
   - Połączenie WiFi
5. Gdy urządzenie połączy się z siecią Wi-Fi
  - Otwórz przeglądarkę i wpsz adres urządzenia
  - Skonfiguruj serer MQTT (Home Assistant)
  - Skonfiguruj czas opóźnienia pompy i czas pracy pompy
  - Skonfiguruj wymiary zbiornik 

## 🏡 Integracja z Home Assistant

System udostępnia w Home Assistant:

- 🚨  Przełącznik 1 Alarm pompy (OFF)
- 🔊  Przełącznik 2 Dźwięk (ON/OFF)
- 🛠️  Przełącznik 3 Serwis (ON/OFF)
- 🚱  Sensor 1 Brak wody (ON/OFF)
- 🌊  Sensor 2 Czujnik wody (ON/OFF)
- 💧  Sensor 3 Objętość wody (L)
- 📏  Sensor 4 Pomiar odległości (mm)
- 📊  Sensor 5 Poziom wody (%)
- 🪣  Sensor 6 Rezerwa wody (ON/OFF)
- 🔌  Sensor 7 Status pompy (ON/OFF)

## 🔒 Funkcje bezpieczeństwa

- 🚱 Zabezpieczenie przed pracą pompy "na sucho"
- ⏱️ Monitorowanie czasu pracy pompy
- ⏲️ Automatyczne wyłączenie po przekroczeniu limitu czasu
- 🛠️ Wykrywanie awarii czujnika poziomu
- 🌐 Automatyczna rekonfiguracja WiFi przy utracie połączenia

## ⚙️ Konfiguracja

Wszystkie parametry można skonfigurować przez interfejs webowy:

- 📶 Parametry sieci (WiFi, MQTT)
- 📏 Wymiary zbiornika
- 🚨 Poziomy alarmowe
- ⏱️ Czas opóźnienia włączenia pompy
- ⏱️ Czasy pracy pompy
- 🛠️ Kalibracja czujnika
- 
## 📜 Licencja

Ten projekt jest udostępniany na licencji MIT.

## 👤 Autor

pimowo
pimowo@gmail.com

**Uwaga**

 * HydroSense - Mój system monitorowania poziomu wody
 * Wersja: Wieczna Beta 2.0 (tak, wiem, ciągle Beta...)
 * Autor: Ja, czyli PMW (Programista Mocno Wannabe) aka @pimowo
 * Data rozpoczęcia: Gdy Arduino było jeszcze cool...
 * Ostatnia modyfikacja: 2024-11-24 00:21:21 UTC (tak, nie śpię o tej porze)
 * 
 * SZCZERE WYZNANIE AMATORA:
 * 
 * Drogi Śmiałku, który odważyłeś się zajrzeć w mój kod!
 * 
 * Jestem programistą-amatorem, niedzielnym hobbystą i wiecznym eksperymentatorem.
 * Bez pomocy AI nawet nie wiedziałbym, gdzie w Arduino jest przycisk 'start'.
 * Ale hej! Z pomocą GitHub Copilota, Stack Overflow i niezliczonych tutoriali,
 * stworzyłem system do monitorowania i kontroli poziomu wody z integracją MQTT.
 * (Tak, sam jestem w szoku, że to działa!)
 * 
 * MOJE ZASADY JAKO AMATORA:
 * - Używasz na własną odpowiedzialność (ja nawet swojego kodu nie rozumiem)
 * - Jak Ci zaleje dom, nie pisz - będę guglowal jak osuszyć własną piwnicę
 * - Wsparcie? Jasne! Razem będziemy szukać na Stack Overflow
 * - Gwarancja? Gwarantuję jedynie, że kod napisał człowiek... chyba
 * 
 * AKTUALNY STAN MOJEGO DZIEŁA:
 * - Działa... ku mojemu wielkiemu zdziwieniu
 * - Nie działa... zgodnie z oczekiwaniami
 * - Błędy nazywam "niezamierzonymi funkcjami edukacyjnymi"
 * - Pracuję nad tym w każdą wolną niedzielę (i czasem w środy, jak żona pozwoli)
 * 
 * CZEGO POTRZEBUJESZ:
 * - Odwagi (dużo odwagi)
 * - Poczucia humoru (przyda się, serio)
 * - Zapasu ręczników (mówię z doświadczenia)
 * - Numeru do hydraulika (polecam mojego - już się przyzwyczaił)
 * - Cierpliwości do amatora (głównie do mnie)
 * 
 * Z CZYM TO WSPÓŁPRACUJE:
 * - Z Home Assistant (jak AI podpowie jak połączyć)
 * - Z MQTT (bo brzmi profesjonalnie)
 * - Ze mną (po trzeciej kawie i wizycie na 10 stronach dokumentacji)
 * 
 * SZCZERZE MÓWIĄC:
 * Jeśli szukasz czegoś pewnego i stabilnego - kup gotowy system.
 * Jeśli jednak, tak jak ja, lubisz uczyć się na błędach - witaj w moim świecie!
 * 
 * PS: Nadal nie wierzę, że czasem to wszystko działa.
 * Może to zasługa AI, może przypadek, a może jednak się uczę?
 * 
 * PPS: Kod powstał przy wsparciu:
 * - Niezliczonych kubków kawy
 * - Stack Overflow (mój drugi dom)
 * - GitHub Copilot (mój prywatny nauczyciel)
 * - Społeczności Arduino (dzięki za cierpliwość!)
 * - Dr. Google i Prof. YouTube
 * 
 * PPPS: Właśnie czytasz mój kod - witaj w rodzinie!
 * Razem będziemy się uczyć... głównie na moich błędach.
 * 
 * Z pozdrowieniami od wiecznego amatora,
 * Wasz @pimowo
 * 
 * PPPPS: Tak, nadal się uczę jak kończyć komentarze...
```
