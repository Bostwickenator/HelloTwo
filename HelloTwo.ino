// Robo India Tutorial 
// Digital Input and Output on LED 
// Hardware: NodeMCU
#include <Wire.h>
#include "Adafruit_HTU21DF.h"

const int analog_ip = A0; //Naming analog input pin
int inputVal  = 0;        //Variable to store analog input values


unsigned int samplingTime = 280;
unsigned int deltaTime = 40;
unsigned int sleepTime = 9680;

const int DUST_LED=D3;

Adafruit_HTU21DF htu = Adafruit_HTU21DF();
void setup() {
  pinMode(DUST_LED, OUTPUT);  // Defining pin as output
  pinMode(D0,OUTPUT);
  digitalWrite(D0,LOW);
  pinMode(A0,INPUT);
  Serial.begin(115200);    // Initiating Serial communication
  htu.begin();

}                                                                                                                                                                                                                           
void loop() {
  for(int i = 0 ; i<9;i++){
  digitalWrite(DUST_LED,LOW);
  delayMicroseconds(samplingTime);

  inputVal += analogRead(A0);

  delayMicroseconds(deltaTime);
  digitalWrite(DUST_LED,HIGH);
  delayMicroseconds(sleepTime);
  }
  Serial.println (inputVal/10);
  inputVal =0;
  delay(100);

  float temp = htu.readTemperature();
  float rel_hum = htu.readHumidity();
  Serial.print("Temp: "); Serial.print(temp); Serial.print(" C");
  Serial.print("\t\t");
  Serial.print("Humidity: "); Serial.print(rel_hum); Serial.println(" \%");
}
