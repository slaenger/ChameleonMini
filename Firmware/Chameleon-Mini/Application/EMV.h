/*
 * EMV.h
 *
 *  Created on: 28.05.2019
 *      Author: slaengerich
 */

#ifndef CHAMELEON_MINI_APPLICATION_EMV_H_
#define CHAMELEON_MINI_APPLICATION_EMV_H_

#include "Application.h"
#include "ISO14443-3A.h"



void EMVAppTask(void);
void EMVAppinitAPDU (void);
void EMVAppResetAPDU (void);


uint16_t EMVAppProcess(uint8_t* Buffer, uint16_t BitCount);

void EMVGetUid(ConfigurationUidType Uid);
void EMVSetUid(ConfigurationUidType Uid);


#endif /* CHAMELEON_MINI_APPLICATION_EMV_H_ */
