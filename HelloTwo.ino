// Robo India Tutorial 
// Digital Input and Output on LED 
// Hardware: NodeMCU
#include <Wire.h>
#include "Adafruit_HTU21DF.h"

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define MQTT_TOPIC_HUMIDITY "home/hellotwo/humidity"
#define MQTT_TOPIC_TEMPERATURE "home/hellotwo/temperature"
#define MQTT_TOPIC_PM "home/hellotwo/particulate"
#define MQTT_TOPIC_STATE "home/hellotwo/status"
#define MQTT_PUBLISH_DELAY 60000
#define MQTT_CLIENT_ID "esp8266bme280"


const int analog_ip = A0; //Naming analog input pin
int inputVal  = 0;        //Variable to store analog input values


unsigned int samplingTime = 280;
unsigned int deltaTime = 40;
unsigned int sleepTime = 9680;

const int DUST_LED=D3;



const char *WIFI_SSID = "TP-Link_D310";
const char *WIFI_PASSWORD = "61193222";

const char *MQTT_SERVER = "192.168.0.128";
const char *MQTT_USER = "mqttuser"; // NULL for no authentication
const char *MQTT_PASSWORD = "mqttpassword"; // NULL for no authentication

float humidity;
float temperature;
long lastMsgTime = 0;

WiFiClient espClient;
PubSubClient mqttClient(espClient);
Adafruit_HTU21DF htu = Adafruit_HTU21DF();

void setup() {
  pinMode(DUST_LED, OUTPUT);  // Defining pin as output
  pinMode(D0,OUTPUT);
  digitalWrite(D0,LOW);
  pinMode(A0,INPUT);
  Serial.begin(115200);    // Initiating Serial communication
  htu.begin();

   setupWifi();
  mqttClient.setServer(MQTT_SERVER, 1883);

}                 

int dustSamples = 0;    
int dustAcc=0;
void accumulateDust() {
  digitalWrite(DUST_LED,LOW);
  delayMicroseconds(samplingTime);
  
  dustAcc += analogRead(A0);
  
  delayMicroseconds(deltaTime);
  digitalWrite(DUST_LED,HIGH);
  delayMicroseconds(sleepTime);
  dustSamples++;
}

float readDust(){
  float val = ((float)dustAcc) / dustSamples;
  dustSamples = 0;
  dustAcc = 0;
  return val;
}

                                                                                                                                                                                                      
void loop() {
  accumulateDust();
  if (!mqttClient.connected()) {
    mqttReconnect();
  }
  mqttClient.loop();

  long now = millis();
  if (now - lastMsgTime > MQTT_PUBLISH_DELAY) {
    lastMsgTime = now;
    
   temperature = htu.readTemperature();
  humidity = htu.readHumidity();
  float dust = readDust();

  Serial.print("Dust: "); Serial.print(dust); Serial.print("");
  Serial.print("Temp: "); Serial.print(temperature); Serial.print("C");
  Serial.print("Humidity: "); Serial.print(humidity); Serial.println("\%");

    // Publishing sensor data
    mqttPublish(MQTT_TOPIC_TEMPERATURE, temperature);
    mqttPublish(MQTT_TOPIC_HUMIDITY, humidity);
    mqttPublish(MQTT_TOPIC_PM, dust);
  }
}

void setupWifi() {
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void mqttReconnect() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");

    // Attempt to connect
    if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD, MQTT_TOPIC_STATE, 1, true, "disconnected", false)) {
      Serial.println("connected");

      // Once connected, publish an announcement...
      mqttClient.publish(MQTT_TOPIC_STATE, "connected", true);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void mqttPublish(char *topic, float payload) {
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(payload);

  mqttClient.publish(topic, String(payload).c_str(), true);
}
