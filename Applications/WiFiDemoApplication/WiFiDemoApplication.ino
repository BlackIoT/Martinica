/*
 * WiFiDemoApplication.h
 *
 *  Created on: Feb 14, 2023
 *      Author: umberto@blackiot.ch
 */
#include <SPI.h>
#include <WiFi101.h>
#include <PubSubClient.h>
#include "bme68xLibrary.h"
#include <M24SR.h>

char ssid[] = "XXXXXX";    // your network SSID (name)
char pass[] = "yyyyyy";    // your network password (use for WPA, or use as key for WEP)

int status = WL_IDLE_STATUS;

uint8_t buffer[500];
size_t  szBuffer = 0;
byte macAddress[6];
char strMacAddress[18] = "00:00:00:00:00:00";
char topicEnviro[] = "martinica/00:00:00:00:00:00/sensor/enviro";
char topicStatus[] = "martinica/00:00:00:00:00:00/status";

typedef struct
{
  float T;
  float H;
  float P;
  float IAQ;
} SensorData;

SensorData sensorData = {21.3,74.5,1010,34};
char sensorBuffer[100];

WiFiClient wificlient;
PubSubClient mqttclient(wificlient);

// BME688 initialization
#define NEW_GAS_MEAS (BME68X_GASM_VALID_MSK | BME68X_HEAT_STAB_MSK | BME68X_NEW_DATA_MSK)
Bme68x bme;

// NFC Tag initialization
#define M24SR_ADDR      0xAC
#define GPO_PIN         43
#define RF_DISABLE_PIN  -1
M24SR nfcTag(M24SR_ADDR, &Wire1, NULL, GPO_PIN, RF_DISABLE_PIN);


void reconnect(void);

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  memcpy(buffer,payload,length);
  szBuffer = length;
/*  
  if(!memcmp(topic,"martinica/sensors/setup",18))
    commAutoma.do_send_setup();
  else  
    commAutoma.do_send_calibration();
    */
}


void setup() {

  Serial.begin(115200);
  delay(5000);
  
  Serial.println("Setup Martinica WiFi Demo App 1.0");
  
// Turn On BME688
  pinMode(BME_OFF, OUTPUT);
  digitalWrite(BME_OFF, LOW);

	Wire1.begin();
	/* Initializes the sensor based on Wire library */
	bme.begin(BME68X_I2C_ADDR_LOW, Wire1);
	Serial.println("Sensor checkStatus");
	if(bme.checkStatus())
	{
		if (bme.checkStatus() == BME68X_ERROR)
		{
			Serial.println("Sensor error:" + bme.statusString());
			return;
		}
		else if (bme.checkStatus() == BME68X_WARNING)
		{
			Serial.println("Sensor Warning:" + bme.statusString());
		}
	}
	/* Set the default configuration for temperature, pressure and humidity */
	bme.setTPH();
	/* Heater temperature in degree Celsius */
	uint16_t tempProf[10] = { 100, 200, 320 };
	/* Heating duration in milliseconds */
	uint16_t durProf[10] = { 150, 150, 150 };
	bme.setSeqSleep(BME68X_ODR_250_MS);
	bme.setHeaterProf(tempProf, durProf, 3);
	bme.setOpMode(BME68X_SEQUENTIAL_MODE);

// NFC TAG
  // Intialize NFC module
  if(nfcTag.begin(NULL) == 0) {
    Serial.println("NFC System Init done!");
  } else {
    Serial.println("NFC System Init failed!");
    while(1);
  }


  mqttclient.setServer("192.168.1.128",1883);
  mqttclient.setCallback(callback);
  mqttclient.setBufferSize(500);

  
  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    
    WiFi.macAddress(macAddress);
    snprintf(strMacAddress,18,"%02x:%02x:%02x:%02x:%02x:%02x",macAddress[5],macAddress[4],macAddress[3],macAddress[2],macAddress[1],macAddress[0]);
    Serial.print("MAC Address : ");
    Serial.println(strMacAddress);

    snprintf(topicEnviro,sizeof(topicEnviro),"martinica/%02x:%02x:%02x:%02x:%02x:%02x/sensor/enviro",macAddress[5],macAddress[4],macAddress[3],macAddress[2],macAddress[1],macAddress[0]);
    Serial.print("Enviro Topic : ");
    Serial.println(topicEnviro);

    snprintf(topicStatus,sizeof(topicStatus),"martinica/%02x:%02x:%02x:%02x:%02x:%02x/status",macAddress[5],macAddress[4],macAddress[3],macAddress[2],macAddress[1],macAddress[0]);
    Serial.print("Status Topic : ");
    Serial.println(topicStatus);

    // wait 10 seconds for connection:
    delay(10000);
  }

  Serial.println("Connected to wifi");

//   if (mqttclient.connect("arduino", "uallievi", "UAllSIMqtt")) {
//  }

  reconnect();  

  //commAutoma.on_setup();
}

void loop() {

  // put your main code here, to run repeatedly:
  if (!mqttclient.loop()) {
    reconnect();
  }


{ // every 100 ms
static uint32_t oldMillis = millis();

//Serial.println(oldMillis); 

if((millis()-oldMillis) >= 100)
{
  oldMillis = millis();
/*  if(commAutoma.on_event(0))
  Serial.print(commAutoma.actual().get_name());
  */
}
}

{ // every second
static uint32_t oldMillis = millis();

if((millis()-oldMillis) >= 1000)
{
  oldMillis = millis();
}
}

{ // every 10 seconds
static uint32_t oldMillis = millis();

if((millis()-oldMillis) >= 10000)
{
  oldMillis = millis();
  //Serial.println("do_send"); 
  //commAutoma.do_send();

}
}

{ // every minute
static uint32_t oldMillis = millis();

if((millis()-oldMillis) >= 60000)
{
  oldMillis = millis();

	bme68xData data;
	uint8_t nFieldsLeft = 0;

	if (bme.fetchData())
	{
		do
		{
			nFieldsLeft = bme.getData(data);
			//if (data.status == NEW_GAS_MEAS)
			{
				Serial.print(String(millis()) + ", ");
				Serial.print(String(data.temperature) + ", ");
				Serial.print(String(data.pressure) + ", ");
				Serial.print(String(data.humidity) + ", ");
				Serial.print(String(data.gas_resistance) + ", ");
				Serial.print(String(data.status, HEX) + ", ");
				Serial.println(data.gas_index);
				if(data.gas_index == 2) /* Sequential mode sleeps after this measurement */
					delay(250);
			}
		} while (nFieldsLeft);

  sensorData.T = data.temperature-13.;
  sensorData.H = data.humidity;
  sensorData.P = data.pressure/100.;
  sensorData.IAQ = data.gas_index;
  }  
  //commAutoma.do_send();
  snprintf(sensorBuffer,100,"{\"T\":%.1f,\"H\":%.1f,\"P\":%.1f,\"IAQ\":%.1f}",sensorData.T,sensorData.H,sensorData.P,sensorData.IAQ);
  mqttclient.publish(topicEnviro,sensorBuffer);

  if(nfcTag.writeTxt(sensorBuffer) == false) {
    Serial.println("Write failed!");
    while(1);
  }

}
}

  //commAutoma.on_idle();    
}


void reconnect(void) {
  // Loop until we're reconnected
  while (!mqttclient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttclient.connect(strMacAddress, "uallievi", "UAllSIMqtt")) {
      Serial.println("connected");
      mqttclient.publish(topicStatus,"connected");
/*      
      if(mqttclient.subscribe("martinica/sensors/setup",0))
         Serial.println("Subscribed setup");
      else   
         Serial.println("Subscribe setup FAIL");
      if(mqttclient.subscribe("martinica/sensors/calibration",0))
         Serial.println("Subscribed calibration");
      else   
         Serial.println("Subscribe calibration FAIL");
         */
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttclient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
