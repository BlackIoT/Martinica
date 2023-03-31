# Martinica
The Martinica Ultra Low Power transmission module consists of ATSAMD21 microcontroller unit with ATWINC1500 Wi-Fi (802.11 b/g/n) module.

The board is in Arduino MKR form factor can be  programmed with Arduino IDE or with Microchip Studio 7 IDE enabling fast application development.

Onboard ATECC608B-TFLXTLSU-PROTO cryptochip from Microchip provides security.

Onboard BME688 Environmental Sensor with A.I. from Bosch Sensortec allows real world testing.

This repository contains sources for Arduino IDE integration and board testing.

A complete application with sources is supported. 

To run examples and Demo Application you must install the following libraries using the library manager:

- Arduino Low Power Library & RTCZero
- WiFi101
- BME68x
- STM32Duino M24SR64-Y
- PubSubClient
- ArduinoECCX08

Manually modify ArduinoECCX08 :

<b>WARNING: Address for ATECC608B-TFLXTLS-PROTO is  0x36 (7bit)</B>

Must be changed in library /libraries/ArduinoECCX08/src/ECCX08.cpp at end of file:

	#ifdef CRYPTO_WIRE
		ECCX08Class ECCX08(CRYPTO_WIRE, 0x36);
	#else
		ECCX08Class ECCX08(Wire, 0x36);
	#endif

Manually install sst26vf library:

	https://github.com/marshallcroes/arduino_sst26vf_fat_filesystem

