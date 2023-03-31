/*
  Blink all Leds

 *  Created on: Feb 14, 2023
 *      Author: umberto@blackiot.ch
*/


// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(USER_LED, OUTPUT);
  digitalWrite(USER_LED, HIGH);
  pinMode(LED_R, OUTPUT);
  digitalWrite(LED_R, HIGH);
  pinMode(LED_G, OUTPUT);
  digitalWrite(LED_G, HIGH);
  pinMode(LED_B, OUTPUT);
  digitalWrite(LED_B, HIGH);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(USER_LED, LOW);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(USER_LED, HIGH);   // turn the LED off by making the voltage LOW
  delay(1000);                      // wait for a second
  static int i = 0;
  i++;
  if(i & 0x01)
     digitalWrite(LED_R, LOW);
  else   
     digitalWrite(LED_R, HIGH);
  
  if(i & 0x02)
     digitalWrite(LED_G, LOW);
  else   
     digitalWrite(LED_G, HIGH);

  if(i & 0x04)
     digitalWrite(LED_B, LOW);
  else   
     digitalWrite(LED_B, HIGH);
}
