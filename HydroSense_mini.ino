#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
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
#define PIN_TRIG D6
#define PIN_ECHO D7
#define PIN_SENSOR D5
#define PIN_PUMP D1
#define PIN_BUZZER D2
#define PIN_BUTTON D3

WiFiClient espClient;
PubSubClient client(espClient);
Ticker ticker;

long duration;
int distance;
bool pumpState = false;
unsigned long previousMillis = 0;
const long interval = 2000; // Interwał pomiaru w milisekundach

void setup() {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_PUMP, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // Konfiguracja OTA
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    // NOTE: jeśli używasz SPIFFS, "FS.begin()" tutaj.
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
  delay(10);
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

void callback(char* topic, byte* payload, unsigned int length) {
  // Obsługa wiadomości MQTT tutaj
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

  if (distance < 10) { // Dostosuj próg według potrzeb
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

  if (distance < 5) { // Próg alarmowy
    digitalWrite(PIN_BUZZER, HIGH);
  } else {
    digitalWrite(PIN_BUZZER, LOW);
  }

  client.publish("hydrosense/level", String(distance).c_str(), true);
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