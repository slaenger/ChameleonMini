/*
 * ApplicationCom.c
 *
 *  Created on: 19.09.2019
 *      Author: slaengerich
 */
#include "ApplicationCom.h"
#include "Terminal.h"


#define IS_CHARACTER(c) ( \
  ( ((c) >= 'A') && ((c) <= 'Z') ) || \
  ( ((c) >= 'a') && ((c) <= 'z') ) || \
  ( ((c) >= '0') && ((c) <= '9') ) || \
  ( ((c) == '_') ) \
)

#define IS_LOWERCASE(c) ( ((c) >= 'a') && ((c) <= 'z') )
#define TO_UPPERCASE(c) ( (c) - 'a' + 'A' )

#define IS_WHITESPACE(c) ( ((c) == ' ') || ((c) == '\t') )

static uint16_t ApplicationProcessDummy(uint8_t* ByteBuffer, uint16_t ByteCount) { return 0; }
const uint8_t EndSignal  [] PROGMEM = {0xFF,0xFF,0xFF,0xFF};

APDUComInterface APDUInterface = {
        ApplicationProcessDummy,
        TerminalBuffer,
        TERMINAL_BUFFER_SIZE,
        0,
        COM_IDLE
};



bool APDUProcessByte (uint8_t Byte)
{
    if(APDUInterface.Mode == COM_RECEIVE){
        if (IS_CHARACTER(Byte)) {
            /* Store uppercase character */
            if (IS_LOWERCASE(Byte)) {
                Byte = TO_UPPERCASE(Byte);
            }

            /* Prevent buffer overflow  and transform into Bytes*/
            if ((APDUInterface.BitCount/8) < APDUInterface.MaxBufferSize && VALID_HEXCHAR(Byte)) {
                uint8_t Nibble = HEXCHAR_TO_NIBBLE(Byte);
                if (APDUInterface.BitCount%8){
                    APDUInterface.CommunicationBuffer[(APDUInterface.BitCount/8)] |= Nibble;
                    APDUInterface.BitCount+=4;
                }else{
                    APDUInterface.CommunicationBuffer[(APDUInterface.BitCount/8)] = Nibble<<4;
                    APDUInterface.BitCount+=4;
                }
            }else TerminalSendStringP(PSTR("String too long"));
        } else if (Byte == '\r') {
            /* Process on \r. Terminate string and decode. */
            if ((APDUInterface.BitCount/8) == 4 && !memcmp_P(APDUInterface.CommunicationBuffer,EndSignal,4)){
                APDUInterface.BitCount=0;
                APDUInterface.Mode=COM_IDLE;
                return true;
            }
            // Call the application with data and respond with application response
            APDUInterface.BitCount = APDUInterface.ApplicationProcess(APDUInterface.CommunicationBuffer,APDUInterface.BitCount);
            if(APDUInterface.BitCount)
            {
                TerminalSendBlock(APDUInterface.CommunicationBuffer,(APDUInterface.BitCount/8));
                TerminalSendChar('\n');
                APDUInterface.BitCount=0;
            }else APDUInterface.Mode = COM_IDLE;
        } else if (Byte == '\b') {
            /* Backspace. Delete last character in buffer. */
            if (APDUInterface.BitCount > 0) {
                APDUInterface.BitCount -= 4;
            }
        } else if (Byte == 0x1B) {
            /* Drop buffer on escape */
            APDUInterface.BitCount = 0;
        } else {
            /* Ignore other chars */
        }
        return true;
    }
    else return false;

}
