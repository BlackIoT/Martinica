/**
 * Copyright (C) 2021 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 *  Modified on: Feb 14, 2023
 *      Author: umberto@blackiot.ch
 */

#include "Arduino.h"
#include "bme68xLibrary.h"

#define NEW_GAS_MEAS (BME68X_GASM_VALID_MSK | BME68X_HEAT_STAB_MSK | BME68X_NEW_DATA_MSK)

Bme68x bme;

/**
 * @brief Initializes the sensor and hardware settings
 */
void setup(void)
{
  pinMode(LED_R, OUTPUT);
  digitalWrite(LED_R, HIGH);
  pinMode(LED_G, OUTPUT);
  digitalWrite(LED_G, HIGH);
  pinMode(LED_B, OUTPUT);
  digitalWrite(LED_B, HIGH);

// Turn On BME688
  pinMode(BME_OFF, OUTPUT);
  digitalWrite(BME_OFF, LOW);


  digitalWrite(LED_R, LOW);
	Wire1.begin();
  digitalWrite(LED_R, HIGH);
	Serial.begin(115200);
//	while (!Serial)
//		delay(10);

	/* Initializes the sensor based on Wire library */
  digitalWrite(LED_G, LOW);
	bme.begin(BME68X_I2C_ADDR_LOW, Wire1);
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_B, LOW);

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

	Serial.println("TimeStamp(ms), Temperature(deg C), Pressure(Pa), Humidity(%), Gas resistance(ohm), Status, Gas index");

  digitalWrite(LED_B, HIGH);
}

void loop(void)
{
	bme68xData data;
	uint8_t nFieldsLeft = 0;

	delay(150);

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
	}
}
