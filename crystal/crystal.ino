#include "WiFi.h"
#include "PubSubClient.h"
#include <string.h>

const char* ssid = "OPPO";     // stored in the flash not in the memory pointer
const char* password = "123456789";
const char* broker_ip = "10.120.17.233";
const int broker_port = 1883;

//  for crystal
#define CLIENT_ID "ESPcrystal"
#define CMD_TOPIC "esp/cmd/crystal"
#define SENSOR_TOPIC "esp/sensor/1"
// // frostbite
// #define CLIENT_ID "ESPfrostbite"
// #define CMD_TOPIC "esp/cmd/frostbite"

// // glacio
// #define CLIENT_ID "ESPglacio"
// #define CMD_TOPIC "esp/cmd/glacio"

WiFiClient espClient;
PubSubClient mqttClient(espClient);

static inline void setup_wifi(){
    // WiFi.mode(WIFI_STA);
    // WiFi.setHostname("Crystal");
    WiFi.begin(ssid,password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts<20){
      Serial.println(F("Waiting for wifi"));
      delay(500);
      attempts++;
    }
    if (WiFi.status() != WL_CONNECTED){
      Serial.println(F("Connection failed"));
      ESP.restart();
    }
    Serial.println(F("wifi connected"));
    Serial.println(WiFi.localIP());
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived on topic: ");
    Serial.println(topic);
    payload[length] = '\0'; // Null terminate
    String message = String((char*)payload);
    if (strcmp(topic, CMD_TOPIC) == 0) {
        Serial.print("Command received: ");
        Serial.println(message);
        if (message == "LED_ON") {
            digitalWrite(2, HIGH);
            Serial.println("LED turned ON");
        } else if (message == "LED_OFF") {
            digitalWrite(2, LOW);
            Serial.println("LED turned OFF");
        }
    }
}

void reconnect() {
    while (!mqttClient.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (mqttClient.connect(CLIENT_ID)) {
            Serial.println("connected");
            mqttClient.subscribe(CMD_TOPIC);
            Serial.println(F("Subscribed to command topic."));
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void publishSensorData() {
    float temperature = analogRead(34) * (3.3 / 4095.0) * 100; // Placeholder for LM35-like sensor
    String payload = String(temperature, 2);
    mqttClient.publish(SENSOR_TOPIC, payload.c_str());
    Serial.print("Published temperature: ");
    Serial.println(payload);
}

void setup() {
    Serial.begin(115200);
    pinMode(2,OUTPUT);
    digitalWrite(2,LOW);
    setup_wifi();
    mqttClient.setServer(broker_ip,broker_port);
    mqttClient.setCallback(mqttCallback);
}

void loop() {
    if (!mqttClient.connected()) {
        reconnect();
    }
    mqttClient.loop();
    static unsigned long lastMsg = 0;
    if (millis() - lastMsg > 2000) {
        lastMsg = millis();
        publishSensorData();
    }
}