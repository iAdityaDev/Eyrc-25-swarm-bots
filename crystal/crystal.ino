// ▪ 
// ▪  Team Id: HB_1005
// ▪  Author List:Vansh Gupta,Aditya Dev Singh,Anurag Choudhary,Moulik Garg
// ▪  Filename: crystal.ino
// ▪  Theme: Holo_Battalion
// ▪  Functions: setup_wifi, mqttCallback, reconnect, velocityToPWM, publishIRdata, setup, loop
// ▪  Global Variables: ssid, password, broker_ip, broker_port, v1, v2, v3, base_angle, elbow_angle
// ▪  Global Constants: IR_PIN, BOT_ID, CLIENT_ID, ELEC_TOPIC, IR_TOPIC, servo1_pin, servo2_pin, servo3_pin, base_servo_pin, elbow_servo_pin, 
//                     ELEC_PIN, PWM_CHANNEL, PWM_FREQ, PWM_RES, NEUTRAL, STOP_MIN, STOP_MAX, MAX_FWD, MAX_REV, MAX_VEL, CMD_TOPIC, LED_TOPIC, SENSOR_TOPIC 
// ▪ 

#include "WiFi.h"
#include "PubSubClient.h"
#include <string.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <Arduino.h>
#define IR_PIN 21

#define BOT_ID 0     // 0=crystal pink, 2=frostbite purple, 4=glacio red

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

const char* ssid = "dev";     // stored in the flash not in the memory pointer
const char* password = "123456789";
const char* broker_ip = "10.42.0.1";
const int broker_port = 1883;

#define CMD_TOPIC "esp/bot_cmd"
#define LED_TOPIC    "esp/led"
#define SENSOR_TOPIC "esp/sensor/1"

static float v1=0, v2=0, v3=0 ,base_angle=0,elbow_angle=0;

WiFiClient espClient;
PubSubClient mqttClient(espClient);



// * Function Name: setup_wifi
// * Input: None (uses global variables `ssid` and `password`)
// * Output: None 
// * Logic: 
// *   - Initiates WiFi connection using stored SSID and password.
// *   - Waits until connected or until 20 attempts (500ms delay each).
// *   - If connection fails after 20 attempts, restarts the ESP32.
// *   - If successful, prints confirmation .
// * Example Call: setup_wifi();
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

// * Function Name: mqttCallback
// * Input: 
// *   - topic : MQTT topic on which message is received
// *   - payload : data
// *   - length : Length of payload
// * Output: None
// * Logic:
// *   - Parses incoming JSON data using ArduinoJson.
// *   - If topic == CMD_TOPIC:
// *        → Extracts id, motor velocities (m1, m2, m3), base and elbow angles.
// *        → If BOT_ID matches received id, publishes motor PWM and servo angles.
// *   - If topic == LED_TOPIC:
// *        → Converts payload to string.
// *        → Turns onboard LED ON/OFF based on command.
// *   - If topic == ELEC_TOPIC:
// *        → Controls PWM output on ELEC_PIN .
// * Example Call: Automatically called when mqttClient.loop() processes a message.
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

// * Function Name: reconnect
// * Input: None
// * Output: None
// * Logic:
// *   - Continuously checks MQTT connection status.
// *   - If not connected, attempts to reconnect using CLIENT_ID.
// *   - On success, subscribes to LED_TOPIC, CMD_TOPIC, ELEC_TOPIC.
// *   - On failure, retries after 5 seconds.
// * Example Call: reconnect();
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

// * Function Name: velocityToPWM
// * Input: 
// *   - vel : Desired motor velocity
// * Output: 
// *   - int: Corresponding PWM pulse width in microseconds
// * Logic:
// *   - If velocity is zero, returns NEUTRAL pulse.
// *   - If velocity > 0 → maps velocity proportionally between STOP_MAX and MAX_FWD.
// *   - If velocity < 0 → maps velocity proportionally between STOP_MIN and MAX_REV.
// *   - Constrains final pulse within safe PWM range.
// * Example Call: 
// *   int pwm = velocityToPWM(500.0);
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

// * Function Name: publishIRdata
// * Input: 
// *   - state (int): Digital state of IR sensor (LOW/HIGH)
// * Output: None
// * Logic:
// *   - Publishes the state to IR_TOPIC via MQTT.
// * Example Call: publishIRdata(digitalRead(IR_PIN));
void publishIRdata(int state) {
    String payload = String(state);
    mqttClient.publish(IR_TOPIC, payload.c_str());
    Serial.print("Published IR: ");
    Serial.println(payload);
}

// * Function Name: setup
// * Input: None
// * Output: None
// * Logic:
// *   - Initializes Serial communication (115200 baud).
// *   - Attaches all motor and arm servos to respective pins.
// *   - Configures onboard LED pin as OUTPUT and sets it LOW.
// *   - Initializes PWM for ELEC_PIN and sets initial duty cycle to 0.
// *   - Configures IR sensor pin as INPUT_PULLUP.
// *   - Connects to WiFi using setup_wifi().
// *   - Sets MQTT broker address and registers callback function.
// * Example Call: Automatically executed once at startup.
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

// * Function Name: loop
// * Input: None
// * Output: None
// * Logic:
// *   - Processes incoming MQTT messages using mqttClient.loop().
// *   - Reads IR sensor state.
// *   - Runs continuously as main execution loop.
// * Example Call: Automatically runs repeatedly after setup().
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
}