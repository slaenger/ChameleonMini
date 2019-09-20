/*
 * ApplicationCom.h
 *
 *  Created on: 19.09.2019
 *      Author: slaengerich
 */

#ifndef FIRMWARE_CHAMELEON_MINI_TERMINAL_APPLICATIONCOM_H_
#define FIRMWARE_CHAMELEON_MINI_TERMINAL_APPLICATIONCOM_H_

#include "../Common.h"

typedef enum {
    COM_RECEIVE,
    COM_IDLE,
}CommunicationMode;

typedef struct {
    uint16_t (*ApplicationProcess)(uint8_t* Buffer, uint16_t BitCount);
    uint8_t* CommunicationBuffer;
    uint16_t MaxBufferSize;
    uint16_t BitCount;
    CommunicationMode Mode;

}APDUComInterface ;
APDUComInterface APDUInterface;

bool APDUProcessByte (uint8_t Byte);

#endif /* FIRMWARE_CHAMELEON_MINI_TERMINAL_APPLICATIONCOM_H_ */
