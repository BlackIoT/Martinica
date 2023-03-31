/**
 ******************************************************************************
 * @file    WriteText.ino
 * @author  wi6labs
 * @version V1.0.0
 * @date    6 July 2017
 * @brief   Arduino test application for the STMicrolectronics
 *          STM32L475V-DISCOVERY-IOT. NFC tag based on M24SR device.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

 /**
 ******************************************************************************
* How to use this sketch
*
* This sketch uses the I2C interface for communicate with the NFC device.
* It writes an NFC tag type Text and reads this same tag.
* Check the pinout and the I2C interface used for communication.
* Choose the content of the text by changing the content of text_write.
*
* When the NFC module is started and ready, the message "Sytstem init done!" is
* displayed on the monitor window. Next, the tag is written, read and printed on
* the monitor window.
*
* You can also use your smartphone to read/write a tag.
* On Android, donwload a NFC Tools. Then start the app, check if NFC is activated
* on your smartphone. Put your smartphone near the tag, you can read it. You can
* write a tag with this app.
 ******************************************************************************
 
 *  Modified on: Feb 14, 2023
 *      Author: umberto@blackiot.ch
 
 */

#include <Wire.h>
#include <M24SR.h>

/*  This configuration is dedicated to the board Disco L475VG IoT.
    You should update those parameters according to the board tested.
*/
#define SerialPort      SerialUSB
#define I2C2_SCL        42
#define I2C2_SDA        41
#define M24SR_ADDR      0xAC
#define GPO_PIN         43
#define RF_DISABLE_PIN  -1

//TwoWire dev_i2c(I2C2_SDA, I2C2_SCL);
M24SR nfcTag(M24SR_ADDR, &Wire1, NULL, GPO_PIN, RF_DISABLE_PIN);

/**
 * Write a Ndef Txt message, wait and read the message from
 * the Nfc tag.
 *
 */
void setup() {
  const char text_write[] = "hello world welcome on earth"; // Text to write in the tag
  char text_read[255] = {'\0'};                             // Text read in the tag
  delay(10000);
  // Initialize serial for output.
  Serial.begin(115200);

  Serial.println("TestStart");
  // Initialize I2C bus.
  Wire1.begin();

  // Intialize NFC module
  if(nfcTag.begin(NULL) == 0) {
    Serial.println("System Init done!");
  } else {
    Serial.println("System Init failed!");
    while(1);
  }

  delay(100);

  if(nfcTag.writeTxt(text_write) == false) {
    Serial.println("Write failed!");
    while(1);
  }

  delay(100);

  //read the txt message and print it
  nfcTag.readTxt(text_read);
  Serial.print("Message content: ");
  Serial.println(text_read);

  if(strcmp(text_write, text_read) == 0) {
    Serial.println("Successfully written and read!");
  } else {
    Serial.println("Read failed!");
  }
}

void loop() {
  //empty loop
}
