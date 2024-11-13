#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <ArduinoJson.h>
#include <time.h>

const char *ssid = "Wokwi-GUEST";
const char *password = "";

const char *mqtt_broker = "broker.emqx.io";
const char *mqtt_topic = "testtopic/mqtt"; 
const char *mqtt_username = "raya_uts";
const char *mqtt_password = "raya123"; 
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient mqtt_client(espClient);

#define DHTPIN 33
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

const int LED_GREEN = 5;
const int LED_YELLOW = 18;
const int LED_RED = 12;
const int RELAY_PIN = 7;
const int BUZZER_PIN = 17;

int suhumax = -1;
int suhumin = 100;
float suhurata = 0.0;
float totalSuhu = 0.0;
int jumlahPembacaan = 0;
int id = 0;

void connectToWiFi();
void connectToMQTTBroker();
void mqttCallback(char *topic, byte *payload, unsigned int length);
String getTimestamp();

void setup() {
    Serial.begin(115200);

    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_YELLOW, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    dht.begin();

    connectToWiFi();
    mqtt_client.setServer(mqtt_broker, mqtt_port);
    mqtt_client.setCallback(mqttCallback);
    connectToMQTTBroker();

    // Konfigurasi waktu NTP
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
}

void connectToWiFi() {
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to the WiFi network");
}

void connectToMQTTBroker() {
    while (!mqtt_client.connected()) {
        String client_id = "esp32-client-" + String(WiFi.macAddress());
        Serial.printf("Connecting to MQTT Broker as %s.....\n", client_id.c_str());
        if (mqtt_client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
            Serial.println("Connected to MQTT broker");
            mqtt_client.subscribe(mqtt_topic);
        } else {
            Serial.print("Failed to connect to MQTT broker, rc=");
            Serial.print(mqtt_client.state());
            Serial.println(" try again in 2 seconds");
            delay(2000);
        }
    }
}

void mqttCallback(char *topic, byte *payload, unsigned int length) {
    Serial.print("Message received on topic: ");
    Serial.println(topic);

    String message;
    for (int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    Serial.println("Payload: " + message);

    StaticJsonDocument<1024> doc;

    DeserializationError error = deserializeJson(doc, message);
    if (error) {
        Serial.print("JSON Parsing failed: ");
        Serial.println(error.c_str());
        return;
    }

    Serial.print("Suhu Max: ");
    Serial.println(doc["suhumax"].as<int>());
    Serial.print("Suhu Min: ");
    Serial.println(doc["suhumin"].as<int>());
    Serial.print("Suhu Rata-rata: ");
    Serial.println(doc["suhurata"].as<float>());
}

String getTimestamp() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        return "Failed to get time";
    }
    char timeString[30];
    strftime(timeString, sizeof(timeString), "%Y-%m-%dT%H:%M:%S", &timeinfo);
    return String(timeString);
}

void loop() {
    mqtt_client.loop();

    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();

    if (isnan(temperature) || isnan(humidity)) {
        Serial.println("Failed to read from DHT sensor!");
        return;
    }

    if (temperature > suhumax) suhumax = temperature;
    if (temperature < suhumin) suhumin = temperature;

    totalSuhu += temperature;
    jumlahPembacaan++;
    id++;
    suhurata = totalSuhu / jumlahPembacaan;

    if (temperature > 35) {
        digitalWrite(LED_RED, HIGH);
        digitalWrite(BUZZER_PIN, HIGH);
        digitalWrite(LED_YELLOW, LOW);
        digitalWrite(LED_GREEN, LOW);
    } else if (temperature >= 30 && temperature <= 35) {
        digitalWrite(LED_YELLOW, HIGH);
        digitalWrite(BUZZER_PIN, LOW);
        digitalWrite(LED_RED, LOW);
        digitalWrite(LED_GREEN, LOW);
    } else {
        digitalWrite(LED_GREEN, HIGH);
        digitalWrite(BUZZER_PIN, LOW);
        digitalWrite(LED_YELLOW, LOW);
        digitalWrite(LED_RED, LOW);
    }

    if (temperature > 30) {
        digitalWrite(RELAY_PIN, HIGH);
    } else {
        digitalWrite(RELAY_PIN, LOW);
    }

    // Membuat payload dengan timestamp
    String payload = "{\"id\":" + String(id) +
                     ",\"temperature\":" + String(temperature) +
                     ",\"humidity\":" + String(humidity) +
                     ",\"suhumax\":" + String(suhumax) +
                     ",\"suhumin\":" + String(suhumin) +
                     ",\"suhurata\":" + String(suhurata) +
                     ",\"pump_status\":\"" + (digitalRead(RELAY_PIN) ? "ON" : "OFF") + "\"" +
                     ",\"timestamp\":\"" + getTimestamp() + "\"}";
                     
    mqtt_client.publish(mqtt_topic, payload.c_str());

    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" *C");

    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");

    delay(2000);
}
