/*
 * messages.h
 *
 *  Created on: Feb 14, 2023
 *      Author: umberto@blackiot.ch
 */

#ifndef MESSAGES_H_
#define MESSAGES_H_

#include <cstdint>

#define MESSAGE_OK               0
#define MESSAGE_RECEIVING        1  
#define MESSAGE_NO_DATA          2
#define MESSAGE_UNDERSIZE        3

#define MESSAGE_ERROR_UNKNOWN    100
#define MESSAGE_ERROR_SOF        101
#define MESSAGE_ERROR_OVERFLOW   102
#define MESSAGE_ERROR_OVERSIZE   103
#define MESSAGE_ERROR_CRC        104
#define MESSAGE_ERROR_TYPE       105
#define MESSAGE_ERROR_ACK        106


#define MESSAGE_ERROR_VOID       200
#define MESSAGE_ERROR_SENSOR     201
#define MESSAGE_ERROR_ACTUATOR   202
#define MESSAGE_ERROR_COMMAND    203
#define MESSAGE_ERROR_ANSWER     204
#define MESSAGE_ERROR_CAPABILITY 205
#define MESSAGE_ERROR_STATUS     206
#define MESSAGE_ERROR_SETUP      207
#define MESSAGE_ERROR_ERROR      208

#define PACKET_TYPE_VOID       0x00 // Payload non presente (solo header)
#define PACKET_TYPE_SENSOR_ENVIRON     0x01 //  Sensori
#define PACKET_TYPE_SENSOR_MOTION      0x02 //  Sensori
#define PACKET_TYPE_ACTUATOR   0x03 // Azionamenti
#define PACKET_TYPE_COMMAND    0x04 // Comandi
#define PACKET_TYPE_ANSWER     0x05 // Risposte ai comandi
#define PACKET_TYPE_CAPABILITY 0x06 // Funzionalità del dispositivo
#define PACKET_TYPE_STATUS     0x07 // Stato del dispositivo
#define PACKET_TYPE_SETUP      0x08 // Setup iniziale del dispositivo
#define PACKET_TYPE_CALIBRATION  0x09 // calibrazione sensori del dispositivo
#define PACKET_TYPE_ERROR      0x0F // Errore



void message_send(int msgType);
void message_send_ack(void);
void message_receive_start(void);
void message_receive_end(void);
uint8_t message_receive_msg_ack(void);
uint8_t message_receive_msg(void);


#endif /* MESSAGES_H_ */
