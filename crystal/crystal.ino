#include "WiFi.h"
#include "PubSubClient.h"
#include <string.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

#define BOT_ID 0   // 0=crystal, 2=frostbite, 4=glacio

#if BOT_ID == 0
  #define CLIENT_ID "ESPcrystal"
#elif BOT_ID == 2
  #define CLIENT_ID "ESPfrostbite"
#elif BOT_ID == 4
  #define CLIENT_ID "ESPglacio"
#else
  #error "Invalid BOT_ID"
#endif

Servo servo1; 
Servo servo2; 
Servo servo3;
Servo base_servo;
Servo elbow_servo; 

#define servo1_pin 27
#define servo2_pin 26
#define servo3_pin 25
#define base_servo_pin 33
#define elbow_servo_pin 32

#define NEUTRAL 1500
#define STOP_MIN 1446
#define STOP_MAX 1542

#define MAX_FWD 1900
#define MAX_REV 1100

#define MAX_VEL 80.0

const char* ssid = "OPPO";     // stored in the flash not in the memory pointer
const char* password = "123456789";
const char* broker_ip = "10.120.17.233";
const int broker_port = 1883;

#define CMD_TOPIC "esp/bot_cmd"
#define LED_TOPIC    "esp/led"
#define SENSOR_TOPIC "esp/sensor/1"

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
    StaticJsonDocument<256> doc;
    deserializeJson(doc, payload, length);
    Serial.println(topic);

    if (strcmp(topic,CMD_TOPIC) == 0){
        int id = doc["id"];
        float m1 = doc["m1"];
        float m2 = doc["m2"];
        float m3 = doc["m3"];
        float base = doc["base"];
        float elbow = doc["elbow"];
        if (BOT_ID == id){
            Serial.println(m1);
            Serial.println(m2);
            Serial.println(m3);
            Serial.println(base);
            Serial.println(elbow);
            servo1.writeMicroseconds(velocityToPWM(-m1));
            servo2.writeMicroseconds(velocityToPWM(m2));
            servo3.writeMicroseconds(velocityToPWM(-m3));
        }
    }

    if (strcmp(topic, LED_TOPIC) == 0) {
        payload[length] = '\0'; // Null terminate
        String message = String((char*)payload);
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
            mqttClient.subscribe(LED_TOPIC);
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

// void publishSensorData() {
//     float temperature = analogRead(34) * (3.3 / 4095.0) * 100; // Placeholder for LM35-like sensor
//     String payload = String(temperature, 2);
//     mqttClient.publish(SENSOR_TOPIC, payload.c_str());
//     Serial.print("Published temperature: ");
//     Serial.println(payload);
// }
int velocityToPWM(float vel) {
    vel = constrain(vel, -MAX_VEL, MAX_VEL);

    if (vel == 0) return NEUTRAL;

    int pulse;

    if (vel > 0) {
        // Map velocity to [STOP_MAX → MAX_FWD]
        pulse = STOP_MAX +
                (vel / MAX_VEL) * (MAX_FWD - STOP_MAX);
    } else {
        // Map velocity to [STOP_MIN → MAX_REV]
        pulse = STOP_MIN +
                (vel / MAX_VEL) * (STOP_MIN - MAX_REV);
    }

    return constrain(pulse, MAX_REV, MAX_FWD);
}


void setup() {
    Serial.begin(115200);
    servo1.attach(servo1_pin);
    servo2.attach(servo2_pin);
    servo3.attach(servo3_pin);
    base_servo.attach(base_servo_pin);
    elbow_servo.attach(elbow_servo_pin);
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
    // static unsigned long lastMsg = 0;
    // if (millis() - lastMsg > 2000) {
    //     lastMsg = millis();
    //     publishSensorData();
    // }
}