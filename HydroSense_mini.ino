#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>

// Dane logowania do WiFi
const char* ssid = "pimowo";
const char* password = "ckH59LRZQzCDQFiUgj";

// Dane logowania do MQTT
const char* mqtt_server = "192.168.1.14";
const int mqtt_port = 1883;
const char* mqtt_user = "hydrosense";
const char* mqtt_password = "hydrosense";

// Piny
#define TRIG_PIN D6
#define ECHO_PIN D7

// Definicje pinów dla innych elementów
#define SENSOR_PIN D5  // Czujnik poziomu wody
#define PUMP_PIN D1    // Pompa
#define BUZZER_PIN D2  // Buzzer
#define BUTTON_PIN D3  // Przycisk kasowania alarmu

// Tworzenie obiektu klienta Wi-Fi, który będzie używany do komunikacji
WiFiClient espClient;
// Inicjalizacja urządzenia Home Assistant o nazwie "HydroSense"
HADevice device("HydroSense");
// Inicjalizacja obiektu MQTT z wykorzystaniem klienta Wi-Fi i urządzenia
HAMqtt haMqtt(espClient, device);
// Czujniki
HASensor waterSensor("water");  // Czujnik wody - informuje Home Assistant o aktualnym stanie czujnika wody (włączony/wyłączony).
HASensor pumpSensor("pump");  // Sterowanie pompą - ten obiekt umożliwia włączanie i wyłączanie pompy za pomocą Home Assistant.
HASensor waterLevelPercent("water_level_percent");  // Procent zapełnienia zbiornika - informuje Home Assistant o aktualnym poziomie wody w zbiorniku w procentach.
HASensor waterVolumeLiters("water_volume_liters");  // Ilość wody w litrach - informuje Home Assistant o ilości wody w zbiorniku w litrach.
HASensor reserveSensor("reserve");  // Czujnik rezerwy
HASensor pomiarSensor("pomiar");  // Czujnik odległości - aktualny odczyt w mm
HASensor waterEmptySensor("water_empty");  // Sensor "Brak wody" do HA
// Przełączniki
HASwitch buzzerSwitch("buzzer");  // Sterowanie buzzerem - umożliwia włączanie i wyłączanie buzzera przez Home Assistant.
HASwitch alarmSwitch("alarm_switch");  // Wyświetlanie i kasowanie alarmu - ten przełącznik pozwala na wyświetlenie stanu alarmu oraz jego ręczne kasowanie.
HASwitch serviceSwitch("service_mode");  // Tryb serwis - przełącznik pozwalający na włączenie lub wyłączenie trybu serwisowego (ignorowanie niektórych funkcji).
// Ustawienia liczbowe
HANumber pumpTimeNumber("pump_time");  // Czas pracy pompy - pozwala ustawić czas pracy pompy, z dokładnością do pełnych sekund.
HANumber delayTimeNumber("delay_time");  // Czas opóźnienia - umożliwia ustawienie opóźnienia przed włączeniem pompy, z dokładnością do pełnych sekund.
HANumber tankDiameterHA("tank_diameter");  // Średnica zbiornika - umożliwia ustawienie średnicy zbiornika w Home Assistant.
HANumber distanceFullHA("distance_full");  // Odległość pełnego zbiornika - ustawienie odległości czujnika od powierzchni wody, gdy zbiornik jest pełny.
HANumber distanceEmptyHA("distance_empty");  // Odległość pustego zbiornika - ustawienie odległości czujnika od dna zbiornika, gdy zbiornik jest pusty.
HANumber reserveThresholdHA("reserve_threshold");  // Ustawienie progu rezerwy


long duration;
int distance;
bool pumpState = false;
unsigned long previousMillis = 0;
const long interval = 2000; // Interwał pomiaru w milisekundach

// Nowe zmienne
int pompa_opoznienie = 5;
int pompa_praca = 60;
int zbiornik_pelny = 65;
int zbiornik_pusty = 510;
int zbiornik_rezerwa = 450;
int zbiornik_histereza = 10;
bool alarm = false;
bool brak_wody = false;
bool dzwiek = false;


void setup() {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_PUMP, OUTPUT);
  digitalWrite(PIN_PUMP, LOW);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_SENSOR, INPUT_PULLUP);

  Serial.begin(115200);
  setup_wifi();

  // Konfiguracja OTA
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    Serial.println("Start OTA: " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nKoniec");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Postęp: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Błąd[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Błąd autoryzacji");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Błąd rozpoczęcia");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Błąd połączenia");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Błąd odbioru");
    } else if (error == OTA_END_ERROR) {
      Serial.println("Błąd zakończenia");
    }
  });
  ArduinoOTA.begin();
}

void setup_wifi() {
  Serial.println();
  Serial.print("Łączenie z ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi połączone");
  Serial.println("Adres IP: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Próba połączenia z MQTT...");
    if (client.connect(mqtt_client_id, mqtt_user, mqtt_password)) {
      Serial.println("połączono");
      client.subscribe("hydrosense/control");
    } else {
      Serial.print("nieudane, rc=");
      Serial.print(client.state());
      Serial.println(" ponowna próba za 5 sekund");
      delay(5000);
    }
  }
}

void measureDistance() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  duration = pulseIn(PIN_ECHO, HIGH);
  distance = duration * 0.034 / 2;

  zbiornik_pelny = distance < 10;
  zbiornik_pusty = distance > 100;
  zbiornik_rezerwa = distance < 20 && distance > 10;

  if (zbiornik_pelny) { // Zbiornik pełny
    if (!pumpState) {
      digitalWrite(PIN_PUMP, HIGH);
      pumpState = true;
    }
  } else {
    if (pumpState) {
      digitalWrite(PIN_PUMP, LOW);
      pumpState = false;
    }
  }

  if (zbiornik_pelny && distance < 5) { // Próg alarmowy
    digitalWrite(PIN_BUZZER, HIGH);
    dzwiek = true;
  } else {
    digitalWrite(PIN_BUZZER, LOW);
    dzwiek = false;
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  ArduinoOTA.handle();

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    measureDistance();
  }
}
