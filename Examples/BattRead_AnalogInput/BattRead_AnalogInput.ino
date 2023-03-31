/*
  Analog Input - Read Battery Level

 *  Created on: Feb 14, 2023
 *      Author: umberto@blackiot.ch
*/
#include "Arduino.h"

int sensorPin = ADC_BATTERY;   // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor

void setup() {
  Serial.println("starting analog battery test");
}

void loop() {
  // read the value from the sensor:
  sensorValue = analogRead(ADC_BATTERY);
  Serial.println(sensorValue);
  delay(1000);
}
