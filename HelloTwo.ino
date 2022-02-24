// Robo India Tutorial
// Digital Input and Output on LED
// Hardware: NodeMCU
#include <Wire.h>
#include "Adafruit_HTU21DF.h"

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Gaussian.h>
#include <LinkedList.h>
#include <GaussianAverage.h>

#define MQTT_TOPIC_HUMIDITY "home/hellotwo/humidity"
#define MQTT_TOPIC_TEMPERATURE "home/hellotwo/temperature"
#define MQTT_TOPIC_PM "home/hellotwo/particulate"
#define MQTT_TOPIC_STATE "home/hellotwo/status"
#define MQTT_PUBLISH_DELAY 60000 * 2
#define MQTT_CLIENT_ID "esp8266bme280"

extern "C"
{
#include "user_interface.h"
}

const int analog_ip = A0; //Naming analog input pin
int inputVal = 0;         //Variable to store analog input values

unsigned int samplingTime = 280;
unsigned int deltaTime = 40;
unsigned int sleepTime = 9680;

const int DUST_LED = D3;

const char *WIFI_SSID = "";
const char *WIFI_PASSWORD = "";

const char *MQTT_SERVER = "192.168.0.128";
const char *MQTT_USER = "mqttuser";         // NULL for no authentication
const char *MQTT_PASSWORD = "mqttpassword"; // NULL for no authentication

float humidity;
float temperature;
long lastMsgTime = 0;

WiFiClient espClient;
PubSubClient mqttClient(espClient);
Adafruit_HTU21DF htu = Adafruit_HTU21DF();

void setup()
{
  pinMode(DUST_LED, OUTPUT); // Defining pin as output
  pinMode(D0, OUTPUT);
  digitalWrite(D0, LOW);
  pinMode(A0, INPUT);
  Serial.begin(115200); // Initiating Serial communication
  htu.begin();

  setupWifi();
  mqttClient.setServer(MQTT_SERVER, 1883);
}

GaussianAverage myAverage(100);
void accumulateDust()
{
  int s = 0;
  for (int i = 0; i < 9; i++)
  {
    digitalWrite(DUST_LED, LOW);
    delayMicroseconds(samplingTime);

    myAverage +=Gaussian(analogRead(A0),10);

    delayMicroseconds(deltaTime);
    digitalWrite(DUST_LED, HIGH);
    delayMicroseconds(sleepTime);
    yield();
  }
}


void loop()
{
  accumulateDust();

  long now = millis();
  if (now - lastMsgTime > MQTT_PUBLISH_DELAY)
  {
    lastMsgTime = now;

    temperature = htu.readTemperature();
    humidity = htu.readHumidity();
    float dust = myAverage.process().mean;

    Serial.print("Dust: ");
    Serial.print(dust);
    Serial.print(" ");
    Serial.print("Temp: ");
    Serial.print(temperature);
    Serial.print("C ");
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println("\%");

    setupWifi();
    if (!mqttClient.connected())
    {
      mqttReconnect();
    }

    // Publishing sensor data
    mqttPublish(MQTT_TOPIC_TEMPERATURE, temperature);
    mqttPublish(MQTT_TOPIC_HUMIDITY, humidity);
    mqttPublish(MQTT_TOPIC_PM, dust);

    mqttClient.loop();

    delay(1000);

    WiFi.mode(WIFI_OFF);
    WiFi.forceSleepBegin();
  }
  delay(1000);
}

IPAddress ip(192, 168, 0, 171);
IPAddress gateway(192, 168, 0, 254);
IPAddress subnet(255, 255, 255, 0);

void setupWifi()
{

  WiFi.forceSleepWake();
  delay(1);
  WiFi.persistent(false);
  Serial.println("");
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.config(ip, gateway, subnet);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  wifi_set_sleep_type(LIGHT_SLEEP_T);
}

void mqttReconnect()
{
  while (!mqttClient.connected())
  {
    Serial.print("Attempting MQTT connection...");

    // Attempt to connect
    if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD, MQTT_TOPIC_STATE, 1, true, "disconnected", false))
    {
      Serial.println("connected");

      // Once connected, publish an announcement...
      mqttClient.publish(MQTT_TOPIC_STATE, "connected", true);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void mqttPublish(char *topic, float payload)
{
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(payload);

  mqttClient.publish(topic, String(payload).c_str(), true);
}

#define MATH_E 2.718281828459045235360287
float absoluteHumidity(float T, float rh)
{
  return (6.112 * pow(MATH_E, ((17.67 * T) / (T + 243.5))) * rh * 18.02) / ((273.15 + T) * 100 * 0.08314);
}

float relativeHumidity(float absoluteHumidity, float T)
{
  return (absoluteHumidity * ((273.15 + T) * 100 * 0.08314)) / (6.112 * pow(MATH_E, ((17.67 * T) / (T + 243.5))) * 18.02);
}
