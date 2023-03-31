/*
  RGBFade

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
  static int i = 0;
  i++;
  if( i & 0x100)
     analogWrite(LED_R, 255 - (i & 0xff));
  if( i & 0x200)
     analogWrite(LED_R, (i & 0xff));
  if( i & 0x400)
     analogWrite(LED_G, 255 - (i & 0xff));
  if( i & 0x800)
     analogWrite(LED_G, (i & 0xff));
  if( i & 0x1000)
     analogWrite(LED_B, 255 - (i & 0xff));
  if( i & 0x2000)
     analogWrite(LED_B, (i & 0xff));

  delay(10); 
}
