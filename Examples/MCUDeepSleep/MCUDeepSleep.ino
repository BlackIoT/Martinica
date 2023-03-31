  
/***************************************************************************
 *  Created on: Feb 14, 2023
 *      Author: umberto@blackiot.ch
 
 ***************************************************************************/


#include "ArduinoLowPower.h"



void setup() {
  
// Power-Down WiFi
  pinMode(WINC1501_RESET_PIN, OUTPUT);
  digitalWrite(WINC1501_RESET_PIN, LOW);
  pinMode(WINC1501_CHIP_EN_PIN, OUTPUT);
  digitalWrite(WINC1501_CHIP_EN_PIN, LOW);
// Disable FLASH memory
  pinMode(FLASH_EN, OUTPUT);
  digitalWrite(FLASH_EN, HIGH);

  delay(30000); // to enable programming the board _ DO NOT REMOVE

  LowPower.deepSleep();
}
  //*********************************************************************
  //*************NOW LET'S START MEASURING*******************************
void loop() 
{ 
}
