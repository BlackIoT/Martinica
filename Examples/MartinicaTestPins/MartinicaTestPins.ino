/*
   Check voltage with mater starting with D0. 

 *  Created on: Feb 14, 2023
 *      Author: umberto@blackiot.ch

*/


#define PORT_LAST 21

void setup() {
  // put your setup code here, to run once:
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(USER_LED, OUTPUT);
  digitalWrite(USER_LED, USER_LED_OFF);

  pinMode(USER_BUTTON, INPUT);

  // st pin D0 ... D14 and A0 ... A6 to digital output
  for(int i = 0; i<=PORT_LAST; i++)
     pinMode(i, OUTPUT);

SerialUSB.println("starting all pin test");
}

bool buttonPressedEvent = false;
bool buttonPressedStatus = false;
int port = 0;

void loop() {
  
// BUTTON to generate events
if(digitalRead(USER_BUTTON) == USER_BUTTON_PRESSED)
{
    digitalWrite(USER_LED, USER_LED_ON);
    if(buttonPressedStatus == false)
    {
       buttonPressedStatus = true;
       buttonPressedEvent = true;
       SerialUSB.println("Button Pressed Event");
    }
}
else
{
    digitalWrite(USER_LED, USER_LED_OFF);
    buttonPressedStatus = false;
}

// get button pressed event
if(buttonPressedEvent)
{
  buttonPressedEvent = false;
  digitalWrite(port, HIGH);
  SerialUSB.print("Port ON : D");
  SerialUSB.println(port);

  if(port == 0)
     digitalWrite(PORT_LAST, LOW);    
  else
     digitalWrite(port-1, LOW);    

  port++;
  if(port > PORT_LAST) port = 0;
}


}
