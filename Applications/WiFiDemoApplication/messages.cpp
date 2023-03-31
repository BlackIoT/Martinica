/*
 * messages.cpp
 *
 *  Created on: Feb 14, 2023
 *      Author: umberto@blackiot.ch
 */
#include "Arduino.h" 
#include "messages.h"
#include <PubSubClient.h>

extern PubSubClient mqttclient;

#define SOF_TX 0xA5
#define SOF_RX 0x5A


uint16_t SAMBA_CRC_Calculate(uint8_t *pBuffer, size_t BufferLength);
uint8_t message_receive_msg_void(const uint8_t *pBuffer, size_t szBuffer);
uint8_t message_receive_msg_sensor_environ(const uint8_t *pBuffer, size_t szBuffer);
uint8_t message_receive_msg_sensor_motion(const uint8_t *pBuffer, size_t szBuffer);
uint8_t message_receive_msg_actuator(const uint8_t *pBuffer, size_t szBuffer);
uint8_t message_receive_msg_command(const uint8_t *pBuffer, size_t szBuffer);
uint8_t message_receive_msg_answer(const uint8_t *pBuffer, size_t szBuffer);
uint8_t message_receive_msg_capability(const uint8_t *pBuffer, size_t szBuffer);
uint8_t message_receive_msg_status(const uint8_t *pBuffer, size_t szBuffer);
uint8_t message_receive_msg_setup(const uint8_t *pBuffer, size_t szBuffer);
uint8_t message_receive_msg_error(const uint8_t *pBuffer, size_t szBuffer);



#define TX_BUFFER_SIZE 500
static uint8_t txBuffer[TX_BUFFER_SIZE] = {0};
#define RX_BUFFER_SIZE 500
static uint8_t rxBuffer[RX_BUFFER_SIZE] = {0};
size_t szRxBuffer = 0;

static uint32_t counter = 0;

extern uint8_t buffer[];
extern size_t  szBuffer;


void message_send(int msgType)
{
//static const uint8_t msgConfigTemp[] = "{\"environ\":{\"ena\":1,\"window\":600,\"temp\":{\"mode\":1,\"offset\":-5,\"rules\":{\"fix\":{\"ena\":1,\"thres\":0,\"tout\":10},\"min\":{\"ena\":0,\"thres\":12.5,\"tout\":10},\"max\":{\"ena\":0,\"thres\":12.5,\"tout\":10},\"delta\":{\"ena\":0,\"thres\":12.5,\"tout\":10}}}}}";
	  //int32_t vector [] = {325,804,10200,104,73};
	  //size_t size = st_environment_encode(&(txBuffer[4]), TX_BUFFER_SIZE-4,counter++,1631789104UL,vector);
	  //size_t size = sizeof(msgConfigTemp);

	  txBuffer[0] = SOF_TX;
	  txBuffer[1] = msgType;
	  txBuffer[2] = (szBuffer >> 8) & 0xFF;
	  txBuffer[3] = szBuffer & 0xFF;

    memcpy(&(txBuffer[4]),buffer,szBuffer);
	  uint16_t crc = SAMBA_CRC_Calculate(txBuffer, szBuffer + 4);

	  txBuffer[szBuffer+4] = (crc >> 8) & 0xFF;
	  txBuffer[szBuffer+5] = crc & 0xFF;

	  Serial1.write(txBuffer,szBuffer+6);
}

void message_send_ack(void)
{

	  txBuffer[0] = SOF_RX;
	  txBuffer[1] = PACKET_TYPE_ANSWER;
	  txBuffer[2] = 00; // payload 0 size
	  txBuffer[3] = 00;

	  uint16_t crc = SAMBA_CRC_Calculate(txBuffer, 4);

	  txBuffer[4] = (crc >> 8) & 0xFF;
	  txBuffer[5] = crc & 0xFF;

	  Serial1.write(txBuffer,6);
}

void message_receive_start(void)
{
	// clears RX register
	//for(uint8_t tmp; commUart.HAL::UART::Receive(&tmp, 1, 0) == HAL_OK;);
  for(;Serial1.available();) Serial1.read();
	// clears buffer
	memset(rxBuffer,0,RX_BUFFER_SIZE);
	szRxBuffer = 0;
	// start receive
	//commUart.Receive(rxBuffer, RX_BUFFER_SIZE);
}

void message_receive_end(void)
{
	//commUart.AbortReceive();
  szRxBuffer = 0;
}

uint8_t message_receive_msg_ack(void)
{
	if(message_receive_msg() == MESSAGE_OK)
	{
       if(rxBuffer[0] == SOF_RX)
          return MESSAGE_OK;
	}
return MESSAGE_ERROR_ACK;
}

//TODO: does not receive correctly if dirt at start of receive
uint8_t message_receive_msg(void)
{
  {
	size_t rxSize = Serial1.available();
	if(rxSize)
       szRxBuffer += Serial1.readBytes(&rxBuffer[szRxBuffer], rxSize); 
  }

	if(szRxBuffer >= 6) // minimum answer len
	{
        if((rxBuffer[0] != SOF_TX) && (rxBuffer[0] != SOF_RX))
		   return MESSAGE_ERROR_SOF;
        // SOT is OK
        size_t size = 0;
        size |= (rxBuffer[2] << 8) & 0xFF00;
        size |= (rxBuffer[3]) & 0xFF;
        if(size > RX_BUFFER_SIZE)
        	return MESSAGE_ERROR_OVERFLOW;
        if(size > (szRxBuffer-6))
            return MESSAGE_UNDERSIZE;
        if(size < (szRxBuffer-6))
            return MESSAGE_ERROR_OVERSIZE;
        // size is in limits and equal to received size
        uint16_t crc = SAMBA_CRC_Calculate(rxBuffer,size+4);
        if(rxBuffer[size+4] != ((crc >> 8) & 0xFF))
        	return MESSAGE_ERROR_CRC;
        if(rxBuffer[size+5] != (crc & 0xFF))
        	return MESSAGE_ERROR_CRC;
        // CRC is OK
		switch(rxBuffer[1])
		{
		case PACKET_TYPE_VOID:
			return message_receive_msg_void(&rxBuffer[4], size);
		case PACKET_TYPE_SENSOR_ENVIRON:
			return message_receive_msg_sensor_environ(&rxBuffer[4], size);
    case PACKET_TYPE_SENSOR_MOTION:
      return message_receive_msg_sensor_motion(&rxBuffer[4], size);
		case PACKET_TYPE_ACTUATOR:
			return message_receive_msg_actuator(&rxBuffer[4], size);
		case PACKET_TYPE_COMMAND:
			return message_receive_msg_command(&rxBuffer[4], size);
		case PACKET_TYPE_ANSWER:
			return message_receive_msg_answer(&rxBuffer[4], size);
		case PACKET_TYPE_CAPABILITY:
			return message_receive_msg_capability(&rxBuffer[4], size);
		case PACKET_TYPE_STATUS:
			return message_receive_msg_status(&rxBuffer[4], size);
		case PACKET_TYPE_SETUP:
			return message_receive_msg_setup(&rxBuffer[4], size);
		case PACKET_TYPE_ERROR:
			return message_receive_msg_error(&rxBuffer[4], size);
		default:
			return MESSAGE_ERROR_TYPE;
		}
   return MESSAGE_RECEIVING;
	}
	return MESSAGE_NO_DATA;
}


uint8_t message_receive_msg_void(const uint8_t *pBuffer, size_t szBuffer)
{
  Serial.println("message_receive_msg_void");
  return MESSAGE_OK;
	//return MESSAGE_ERROR_VOID;
}

uint8_t message_receive_msg_sensor_environ(const uint8_t *pBuffer, size_t szBuffer)
{
   mqttclient.publish("enel/sensors/environment",pBuffer,szBuffer);
  
  Serial.println("message_receive_msg_sensor_environ");
  return MESSAGE_OK;
//	return MESSAGE_ERROR_SENSOR;
}

uint8_t message_receive_msg_sensor_motion(const uint8_t *pBuffer, size_t szBuffer)
{
   mqttclient.publish("enel/sensors/motion",pBuffer,szBuffer);
  
  Serial.println("message_receive_msg_sensor_motion");
  return MESSAGE_OK;
//  return MESSAGE_ERROR_SENSOR;
}

uint8_t message_receive_msg_actuator(const uint8_t *pBuffer, size_t szBuffer)
{
  Serial.println("message_receive_msg_actuator");
  return MESSAGE_OK;
	//return MESSAGE_ERROR_ACTUATOR;
}

uint8_t message_receive_msg_command(const uint8_t *pBuffer, size_t szBuffer)
{
  Serial.println("message_receive_msg_command");
  return MESSAGE_OK;
	//return MESSAGE_ERROR_COMMAND;
}

uint8_t message_receive_msg_answer(const uint8_t *pBuffer, size_t szBuffer)
{
  Serial.println("message_receive_msg_answer");
	return MESSAGE_OK;
	//return MESSAGE_ERROR_ANSWER;
}

uint8_t message_receive_msg_capability(const uint8_t *pBuffer, size_t szBuffer)
{
  Serial.println("message_receive_msg_capability");
  return MESSAGE_OK;
	//return MESSAGE_ERROR_CAPABILITY;
}

uint8_t message_receive_msg_status(const uint8_t *pBuffer, size_t szBuffer)
{
  Serial.println("message_receive_msg_status");
  return MESSAGE_OK;
	//return MESSAGE_ERROR_STATUS;
}

uint8_t message_receive_msg_setup(const uint8_t *pBuffer, size_t szBuffer)
{
  Serial.println("message_receive_msg_setup");
  return MESSAGE_OK;
	//return MESSAGE_ERROR_SETUP;
}

uint8_t message_receive_msg_error(const uint8_t *pBuffer, size_t szBuffer)
{
  Serial.println("message_receive_msg_error");
  return MESSAGE_OK;
	//return MESSAGE_ERROR_ERROR;
}


uint16_t SAMBA_CRC_Accumulate(uint16_t *crc, uint8_t *pBuffer, size_t BufferLength)
{
static const uint16_t crc_lut[256] = {
	    0x0000U, 0x1021U, 0x2042U, 0x3063U, 0x4084U, 0x50a5U, 0x60c6U, 0x70e7U,
	    0x8108U, 0x9129U, 0xa14aU, 0xb16bU, 0xc18cU, 0xd1adU, 0xe1ceU, 0xf1efU,
	    0x1231U, 0x0210U, 0x3273U, 0x2252U, 0x52b5U, 0x4294U, 0x72f7U, 0x62d6U,
	    0x9339U, 0x8318U, 0xb37bU, 0xa35aU, 0xd3bdU, 0xc39cU, 0xf3ffU, 0xe3deU,
	    0x2462U, 0x3443U, 0x0420U, 0x1401U, 0x64e6U, 0x74c7U, 0x44a4U, 0x5485U,
	    0xa56aU, 0xb54bU, 0x8528U, 0x9509U, 0xe5eeU, 0xf5cfU, 0xc5acU, 0xd58dU,
	    0x3653U, 0x2672U, 0x1611U, 0x0630U, 0x76d7U, 0x66f6U, 0x5695U, 0x46b4U,
	    0xb75bU, 0xa77aU, 0x9719U, 0x8738U, 0xf7dfU, 0xe7feU, 0xd79dU, 0xc7bcU,
	    0x48c4U, 0x58e5U, 0x6886U, 0x78a7U, 0x0840U, 0x1861U, 0x2802U, 0x3823U,
	    0xc9ccU, 0xd9edU, 0xe98eU, 0xf9afU, 0x8948U, 0x9969U, 0xa90aU, 0xb92bU,
	    0x5af5U, 0x4ad4U, 0x7ab7U, 0x6a96U, 0x1a71U, 0x0a50U, 0x3a33U, 0x2a12U,
	    0xdbfdU, 0xcbdcU, 0xfbbfU, 0xeb9eU, 0x9b79U, 0x8b58U, 0xbb3bU, 0xab1aU,
	    0x6ca6U, 0x7c87U, 0x4ce4U, 0x5cc5U, 0x2c22U, 0x3c03U, 0x0c60U, 0x1c41U,
	    0xedaeU, 0xfd8fU, 0xcdecU, 0xddcdU, 0xad2aU, 0xbd0bU, 0x8d68U, 0x9d49U,
	    0x7e97U, 0x6eb6U, 0x5ed5U, 0x4ef4U, 0x3e13U, 0x2e32U, 0x1e51U, 0x0e70U,
	    0xff9fU, 0xefbeU, 0xdfddU, 0xcffcU, 0xbf1bU, 0xaf3aU, 0x9f59U, 0x8f78U,
	    0x9188U, 0x81a9U, 0xb1caU, 0xa1ebU, 0xd10cU, 0xc12dU, 0xf14eU, 0xe16fU,
	    0x1080U, 0x00a1U, 0x30c2U, 0x20e3U, 0x5004U, 0x4025U, 0x7046U, 0x6067U,
	    0x83b9U, 0x9398U, 0xa3fbU, 0xb3daU, 0xc33dU, 0xd31cU, 0xe37fU, 0xf35eU,
	    0x02b1U, 0x1290U, 0x22f3U, 0x32d2U, 0x4235U, 0x5214U, 0x6277U, 0x7256U,
	    0xb5eaU, 0xa5cbU, 0x95a8U, 0x8589U, 0xf56eU, 0xe54fU, 0xd52cU, 0xc50dU,
	    0x34e2U, 0x24c3U, 0x14a0U, 0x0481U, 0x7466U, 0x6447U, 0x5424U, 0x4405U,
	    0xa7dbU, 0xb7faU, 0x8799U, 0x97b8U, 0xe75fU, 0xf77eU, 0xc71dU, 0xd73cU,
	    0x26d3U, 0x36f2U, 0x0691U, 0x16b0U, 0x6657U, 0x7676U, 0x4615U, 0x5634U,
	    0xd94cU, 0xc96dU, 0xf90eU, 0xe92fU, 0x99c8U, 0x89e9U, 0xb98aU, 0xa9abU,
	    0x5844U, 0x4865U, 0x7806U, 0x6827U, 0x18c0U, 0x08e1U, 0x3882U, 0x28a3U,
	    0xcb7dU, 0xdb5cU, 0xeb3fU, 0xfb1eU, 0x8bf9U, 0x9bd8U, 0xabbbU, 0xbb9aU,
	    0x4a75U, 0x5a54U, 0x6a37U, 0x7a16U, 0x0af1U, 0x1ad0U, 0x2ab3U, 0x3a92U,
	    0xfd2eU, 0xed0fU, 0xdd6cU, 0xcd4dU, 0xbdaaU, 0xad8bU, 0x9de8U, 0x8dc9U,
	    0x7c26U, 0x6c07U, 0x5c64U, 0x4c45U, 0x3ca2U, 0x2c83U, 0x1ce0U, 0x0cc1U,
	    0xef1fU, 0xff3eU, 0xcf5dU, 0xdf7cU, 0xaf9bU, 0xbfbaU, 0x8fd9U, 0x9ff8U,
	    0x6e17U, 0x7e36U, 0x4e55U, 0x5e74U, 0x2e93U, 0x3eb2U, 0x0ed1U, 0x1ef0U
	};

	uint16_t val1, val2;
    for (size_t i = 0; i < BufferLength; i++)
    {
        val2 = (*crc >> 8) ^ pBuffer[i];
        val1 = crc_lut[val2 & 0xFFU];
        val2 = *crc << 8;
        val1 = val1 ^ val2;
        *crc = val1;
    }
    return *crc;
}

uint16_t SAMBA_CRC_Calculate(uint8_t *pBuffer, size_t BufferLength)
{
	uint16_t crc = 0xFFFF;
	SAMBA_CRC_Accumulate(&crc, pBuffer, BufferLength);
	return crc;
}
