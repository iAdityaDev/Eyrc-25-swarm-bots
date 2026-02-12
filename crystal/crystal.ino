#include "WiFi.h"
#include "PubSubClient.h"
#include <string.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <Arduino.h>
#define IR_PIN 21

#define BOT_ID 2     // 0=crystal pink, 2=frostbite purple, 4=glacio red

#if BOT_ID == 0
  #define CLIENT_ID "ESPcrystal"
  #define ELEC_TOPIC "esp/crystal_elec"
  #define IR_TOPIC "esp/crystal_ir"
#elif BOT_ID == 2
  #define CLIENT_ID "ESPfrostbite"
  #define ELEC_TOPIC "esp/frostbite_elec"
  #define IR_TOPIC "esp/frostbite_ir"
#elif BOT_ID == 4
  #define CLIENT_ID "ESPglacio"
  #define ELEC_TOPIC "esp/glacio_elec"
  #define IR_TOPIC "esp/glacio_ir"
#else
  #error "Invalid BOT_ID"
#endif

Servo servo1; 
Servo servo2; 
Servo servo3;
Servo base_servo;
Servo elbow_servo; 

#define servo1_pin 27
#define servo2_pin 25
#define servo3_pin 26
#define base_servo_pin 33
#define elbow_servo_pin 32

const int ELEC_PIN = 23;
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 1000;    // 1 kHz
const int PWM_RES = 8;        // 8-bit resolution -> values 0..255

#define NEUTRAL 1500
#define STOP_MIN 1446
#define STOP_MAX 1542

// #define MAX_FWD 1900
// #define MAX_REV 1100
// #define MAX_FWD 2200
// #define MAX_REV 800
#define MAX_FWD 2400
#define MAX_REV 600

#define MAX_VEL 1000.0

const char* ssid = "OPPO";     // stored in the flash not in the memory pointer
const char* password = "123456789";
const char* broker_ip = "10.247.7.181";
const int broker_port = 1883;

#define CMD_TOPIC "esp/bot_cmd"
#define LED_TOPIC    "esp/led"
#define SENSOR_TOPIC "esp/sensor/1"

static float v1=0, v2=0, v3=0 ,base_angle=0,elbow_angle=0;

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
            // Serial.println(m1);
            // Serial.println(m2);
            // Serial.println(m3);
            // Serial.println(base);
            // Serial.println(elbow);
            
            // v1 = Step_vel(m1, v1);
            // v2 = Step_vel(m2, v2);
            // v3 = Step_vel(m3, v3);
            // v1 = smoothVel(m1, v1);
            // v2 = smoothVel(m2, v2);
            // v3 = smoothVel(m3, v3);
            // servo1.writeMicroseconds(velocityToPWM(-v1));    
            // servo2.writeMicroseconds(velocityToPWM(-v2));
            // servo3.writeMicroseconds(velocityToPWM(-v3));
            servo1.writeMicroseconds(velocityToPWM(-m1));
            servo2.writeMicroseconds(velocityToPWM(-m2));
            servo3.writeMicroseconds(velocityToPWM(-m3));
            base_servo.write(base);
            elbow_servo.write(elbow);
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

    if (strcmp(topic,ELEC_TOPIC) == 0){
        payload[length] = '\0';
        String message = String((char*)payload);
        Serial.print("Command received: ");
        Serial.println(message);
        if (message == "TRUE") {
            ledcWrite(ELEC_PIN, 254);
            Serial.println("ELEC turned ON");
        } else if (message == "FALSE") {
            ledcWrite(ELEC_PIN,0);
            Serial.println("ELEC turned OFF");
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
            mqttClient.subscribe(ELEC_TOPIC);
            Serial.println(F("Subscribed to command topic."));
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

int velocityToPWM(float vel) {

    if (vel == 0) return NEUTRAL;

    int pulse;

    if (vel > 0) {
        pulse = STOP_MAX +
                (vel / MAX_VEL) * (MAX_FWD - STOP_MAX);
    } else {
        pulse = STOP_MIN +
                (vel / MAX_VEL) * (STOP_MIN - MAX_REV);
    }
    return constrain(pulse, MAX_REV, MAX_FWD);
}


// float smoothVel(float target, float &current) {
//     if (target == 0.0) return target ;                          
//     float step = 10.0; // limit per cycle
//     if (target > current) current += step;
//     else if (target < current) current -= step;
//     return current;
// }


// float Step_vel(float target, float &current) {
//     if (target == 0.0) return target ;      
                        
//     // if (current>=0.0 && current<=20.0 && target <= 20.0 ){
//     //     return target;

//     // }
//     // if (current>=0.0 && current<20.0 && target >= 20.0 ){
//     //     return 20.0;

//     // }
//     // else if(current >= 20.0 && current <= 100.0 && target >= 20.0 && target <= 100.0){
//     //     return target;
//     // }
//     // else if(current >= 20.0 && current<100.0 && target>=100.0){
//     //      return 100.0;
//     // }

//     // else if(current >= 100.0 ){
//     //      return target;
//     // }
//     // else if(current >target ){
//     //      return target;
//     // }


//     else if (current <= 0.0 && current >= -20.0 && target >= -20.0) {
//         return target;
//     }
//     else if (current <= 0.0 && current > -20.0 && target <= -20.0) {
//         return -20.0;
//     }
//     else if (current <= -20.0 && current >= -100.0 && target <= -20.0 && target >= -100.0) {
//         return target;
//     }
//     else if (current <= -20.0 && current > -100.0 && target <= -100.0) {
//         return -100.0;
//     }

//     // ---------- -150 ----------
//     else if (current <= -100.0 && current >= -150.0 && target >= -150.0) {
//         return target;
//     }
//     else if (current <= -100.0 && current > -150.0 && target <= -150.0) {
//         return -150.0;
//     }

//     // ---------- -200 ----------
//     else if (current <= -150.0 && current >= -200.0 && target <= -150.0 && target >= -200.0) {
//         return target;
//     }
//     else if (current <= -150.0 && current > -200.0 && target <= -200.0) {
//         return -200.0;
//     }
//     else if (current <= -200.0) {
//         return target;
//     }

//     // ---------- FALLBACK ----------
//     else if (current < target) {
//         return target;
//     }
    

// }

// float Step_vel(float target, float current)
// {
//     // If already at target
//     if (current == target)
//         return current;

//     // If direction is changing, go to zero first
//     if ((current > 0 && target < 0) || (current < 0 && target > 0))
//     {
//         if (current != 0.0f)
//             return 0.0f;
//     }

//     // Determine direction
//     float dir = (target > current) ? 1.0f : -1.0f;

//     float abs_current = fabs(current);
//     float abs_target  = fabs(target);
    
//     // Step ladder
//     if (abs_current < 20.0f)
//         return dir * 20.0f;

//     else if (abs_current < 100.0f)
//         return dir * 100.0f;

//     else if (abs_current < 200.0f)
//         return dir * 200.0f;

//     // Clamp to target if beyond
//     return target;
// }

// float smoothVel(float target, float &current) {
//     target = constrain(target, -MAX_VEL, MAX_VEL);
//     current = constrain(current, -MAX_VEL, MAX_VEL);
//     if (target == 0.0) return target ;
//     if ((targetcurrent)<0.0) return 0.0 ;
//     float alpha = 0.1; 
//     if (abs(current-target)<=10){
//         float alpha = 1.0;
//     }
//     if ((abs(current-target))>10 && (abs(current-target))<=50){
//         float alpha = 0.5;
//     }
//     if ((abs(current-target))>100 && (abs(current-target))<=100){
//         float alpha = 0.25;
//     }
//     if ((abs(current-target))>100) && (abs(current-target))<=150){
//         float alpha = 0.2;
//     }
//     if ((abs(current-target))>150 && (abs(current-target))<=200){
//         float alpha = 0.125;
//     }
//     current += alpha (target - current);
//     return current;
// }


// float smoothVel(float target, float &current) {
//     target  = constrain((target/5),  -MAX_VEL, MAX_VEL);
//     current = constrain((current/5), -MAX_VEL, MAX_VEL);

//     // If target is zero, smoothly bring current to zero
//     if (target == 0.0) {
        
//         return target;
//     }

//     // If direction changes, stop first
//     if ((target * current) < 0.0) {
//         current = 0.0;
//         return current;
//     }

//     float error = abs(target - current);
//     float alpha = 0.1;

//     if (error <= 10) {
//         alpha = 1.0;
//     } else if (error <= 50) {
//         alpha = 0.5;
//     } else if (error <= 100) {
//         alpha = 0.25;
//     } else if (error <= 150) {
//         alpha = 0.2;
//     } else if (error <= 200) {
//         alpha = 0.125;
//     }

//     current += alpha * (target - current);
//     return current;
// }

void publishIRdata(int state) {
    String payload = String(state);
    mqttClient.publish(IR_TOPIC, payload.c_str());
    Serial.print("Published IR: ");
    Serial.println(payload);
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
    
    ledcAttach(ELEC_PIN, PWM_FREQ, PWM_RES);
    ledcWrite(ELEC_PIN, 0); 

    pinMode(IR_PIN, INPUT_PULLUP);

    setup_wifi();
    mqttClient.setServer(broker_ip,broker_port);
    mqttClient.setCallback(mqttCallback);
}

void loop() {
    if (!mqttClient.connected()) {
        reconnect();
    }
    mqttClient.loop();

    int state = digitalRead(IR_PIN);
    
    if (state == LOW) {
        Serial.println("Object Detected!");
        publishIRdata(state);
    
        // static unsigned long lastMsg = 0;
        // if (millis() - lastMsg > 200) {
        //     lastMsg = millis();
        //     publishIRdata(state);
        // }

    } else {
        publishIRdata(state);
        Serial.println("Nothing here...");
    }
    
    // static unsigned long lastMsg = 0;
    // if (millis() - lastMsg > 2000) {
    //     lastMsg = millis();
    //     publishSensorData();
    // }
}