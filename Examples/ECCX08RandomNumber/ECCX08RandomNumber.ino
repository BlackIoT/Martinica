/*
  ECCX08 Random Number

  This sketch uses the ECC508 or ECC608 to generate a random number 
  every second and print it to the Serial monitor

  Circuit:
   - MKR board with ECC508 or ECC608 on board

  created 19 July 2018
  by Sandeep Mistry
  
 *  Modified on: Feb 14, 2023
 *      Author: umberto@blackiot.ch
  
*/

#include <ArduinoECCX08.h>

uint16_t ECCX08ClassCrc16(const byte data[], size_t length)
{
  if (data == NULL || length == 0) {
    return 0;
  }

  uint16_t crc = 0;

  while (length) {
    byte b = *data;

    for (uint8_t shift = 0x01; shift > 0x00; shift <<= 1) {
      uint8_t dataBit = (b & shift) ? 1 : 0;
      uint8_t crcBit = crc >> 15;

      crc <<= 1;
      
      if (dataBit != crcBit) {
        crc ^= 0x8005;
      }
    }

    length--;
    data++;
  }

  return crc;
}


int ECCX08ClassReceiveResponse(void* response, size_t length)
{
  int retries = 20;
  size_t responseSize = length + 3; // 1 for length header, 2 for CRC
  byte responseBuffer[responseSize];

  while (Wire1.requestFrom((uint8_t)0x36, (size_t)responseSize, (bool)true) != responseSize && retries--);

  Serial.print("Retries : ");
  Serial.println(retries);
  responseBuffer[0] = Wire1.read();

  // make sure length matches
  if (responseBuffer[0] != responseSize) {
    Serial.println("No Len Match");
    return 0;
  }

  for (size_t i = 1; Wire1.available(); i++) {
    responseBuffer[i] = Wire1.read();
  }

  // verify CRC
  uint16_t responseCrc = responseBuffer[length + 1] | (responseBuffer[length + 2] << 8);
  if (responseCrc != ECCX08ClassCrc16(responseBuffer, responseSize - 2)) {
    Serial.println("CRC Fail");
    return 0;
  }
  
  memcpy(response, &responseBuffer[1], length);

  return 1;
}

int ECCX08ClassWakeup()
{
  Wire1.setClock(100000u);
  Wire1.beginTransmission(0x00);
  Wire1.endTransmission();

  delayMicroseconds(1500);

  byte response;

  if (!ECCX08ClassReceiveResponse(&response, sizeof(response)) || response != 0x11) {
    return 0;
  }

  Wire1.setClock(100000u);

  return 1;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

// ------------ test iniziale
/*
for(;;)
{

  Serial.println("Test Start");
  Wire1.begin();
  int rc = ECCX08ClassWakeup();
  if(rc)
    Serial.println("Wakeup successful");
  else   
    Serial.println("Wakeup error");
 
 

 delay(1000);
}
*/
// ------------ fine test  

  if (!ECCX08.begin()) {
    Serial.println("Failed to communicate with ECC508/ECC608!");
    while (1);
  }

  if (!ECCX08.locked()) {
    Serial.println("The ECC508/ECC608 is not locked!");
    while (1);
  }
}

void loop() {
  Serial.print("Random number = ");
  Serial.println(ECCX08.random(65535));

  delay(1000);
}

