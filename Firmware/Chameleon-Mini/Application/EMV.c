/*
 * EMV.c
 *
 *  Created on: 28.05.2019
 *      Author: slaengerich
 */
#include "EMV.h"

#include "../Random.h"
#include "../Codec/ISO14443-2A.h"
#include"../LEDHook.h"
#include "../Terminal/ApplicationCom.h"



#define UID_CL1_ADDRESS         0x00
#define UID_SINGLE_SIZE			4
#define UID_CL1_SIZE            3
#define UID_BCC1_ADDRESS        0x03
#define UID_CL2_ADDRESS         0x04
#define UID_CL2_SIZE            4
#define UID_BCC2_ADDRESS        0x08
#define EMV_CL_BCC_SIZE 		1
#define EMV_UID0_CT				0x00
#define BLOCK_LENGTH			16

#define ATQA_VALUE              0x0002
#define SAK_VALUE			0x38
#define SAK_CL1_VALUE           ISO14443A_SAK_INCOMPLETE
#define SAK_CL2_VALUE           ISO14443A_SAK_COMPLETE_NOT_COMPLIANT

#define ACK_VALUE               0x0A
#define ACK_FRAME_SIZE          4 /* Bits */
#define NAK_INVALID_ARG         0x00
#define NAK_CRC_ERROR           0x01
#define NAK_CTR_ERROR           0x04
#define NAK_EEPROM_ERROR        0x05
#define NAK_OTHER_ERROR         0x06
/* NOTE: the spec is not crystal clear which error is returned */
#define NAK_AUTH_REQUIRED       NAK_OTHER_ERROR
#define NAK_AUTH_FAILED         NAK_OTHER_ERROR
#define NAK_FRAME_SIZE          4
// Only ISO Command in use in ready State
#define CMD_HALT                0x50
// EMV rats Command
#define CMD_RATS				0xe0
#define DEFAULT_T0				0x78 // frame window of 8 = 256B and TA/TB/TC following
#define DEFAULT_TA				0x80 // default bit rate of 106 kbit/s in both directions
#define DEFAULT_TB				0xa1 // default FWT of  11= 618 ms and SFGI of 1
#define DEFAULT_TC				0x00 // CID and NAD not supported


static bool FromHalt = false;
static bool BlockNumber;
bool Communicating;
uint16_t ReceivedBitCount;
//uint8_t* AcBuffer = &CodecBuffer2[0];
static uint8_t RetryCounter = 0;
bool SkipAcGen;
static uint16_t RetransmissonLength;

bool ReceivedData;
static bool DataChaining;


static enum {
	STATE_HALT,
	STATE_IDLE,
	STATE_READY1,
	STATE_READY2,
	STATE_ACTIVE,
	STATE_PROTOCOL,
} State;



uint16_t TerminalInterface (uint8_t* Buffer,uint16_t BitCount)
{
    ReceivedData = true;
    ReceivedBitCount = BitCount;
    return 0;
}

void EMVAppinitAPDU ()
{
    ReceivedData= false;
	APDUInterface.Mode = COM_IDLE;
    APDUInterface.BitCount = 0;
	APDUInterface.ApplicationProcess = TerminalInterface;
	APDUInterface.CommunicationBuffer = TerminalBuffer;
	APDUInterface.MaxBufferSize = CODEC_BUFFER_SIZE;
    LEDHook(LED_SETTING_CHANGE,LED_OFF);
	State = STATE_IDLE;
	FromHalt = false;
	BlockNumber=1;
	RetransmissonLength = 0;
	ReceivedBitCount=0;
	Communicating = false;
}

void EMVAppResetAPDU(void)
{
    APDUInterface.Mode = COM_IDLE;
    APDUInterface.BitCount = 0;
    State = STATE_IDLE;
    ReceivedBitCount = 0;
    BlockNumber = 1;
    RetransmissonLength = 0;
    FromHalt = false;
    RetryCounter = 0;
	//	DataAmount = 0;
}

/* Is called on every Tick usefull if you want to do things in parallel with Codec */
void EMVAppTask(void)
{

}

/* Handles processing of ISO 14443 commands */
static uint16_t AppProcess(uint8_t* const Buffer, uint16_t ByteCount)
{
	uint8_t Cmd = Buffer[0];

	/* Handle commands (very short List of Commands since most work is done in higher layer Protocol) */
	switch (Cmd) {

	case CMD_HALT: {
		/* Halts the tag. According to the ISO14443, the second
		 * byte is supposed to be 0. */
		if (Buffer[1] == 0) {
			/* According to ISO14443, we must not send anything
			 * in order to acknowledge the HALT command. */
			State = STATE_HALT;
			return ISO14443A_APP_NO_RESPONSE;
		} else {
			Buffer[0] = NAK_INVALID_ARG;
			return NAK_FRAME_SIZE;
		}
	}
	case CMD_RATS :
	{

		State = STATE_PROTOCOL;
		Buffer[0] = 5; // specifying length of Standard Message to 5 bytes
		Buffer[1] = DEFAULT_T0;
		Buffer[2] = DEFAULT_TA;//Interfacebyte TA
		Buffer[3] = DEFAULT_TB;// SFWTG and FWT
		Buffer[4] = DEFAULT_TC;//Indicates whether NAD and CID are supported (No support)
		ISO14443AAppendCRCA(Buffer,5);
		BlockNumber=1;
		return 56; // 4 Bytes + 2 CRCA Bytes

	}
	default:
		break;
	}

	/* Command not handled. Switch to idle. */
	State = STATE_IDLE;
	return ISO14443A_APP_NO_RESPONSE;
}

/* Creates RBlock with Response true/false*/
static uint16_t SendRBlock (uint8_t* const Buffer, bool Response)
{
	Buffer[0] = 0xa2;
	Buffer[0] |= ((!Response) <<4);
	Buffer[0] |= BlockNumber;
	return 1;
}

/* Handles higher layer messages in Half Duplex Transmission Protocol */
static uint16_t ProtocolProcess(uint8_t* const Buffer, uint16_t ByteCount)
{
	uint16_t ByteCounts=0;
	if (ISO14443ACheckCRCA(Buffer, ByteCount - ISO14443A_CRCA_SIZE)&& ByteCount >=3) // Block should be Appended with CRCA check this
	{
		uint8_t BlockType = Buffer [0] >>6; // Block Type is given by first 2 most significant bits

		switch (BlockType) {
		case 0 : // I Block Contains Application Data
		{
		    RetryCounter = 0;
		    BlockNumber = !BlockNumber;// Trigger BlockNumber
		    //TODO Data Chaining if we receive large amounts of Data should be added around here

		    Buffer[0] = Buffer[0]&0x10;
		    APDUInterface.Mode = COM_RECEIVE;
		    BufferToHexString((char*)APDUInterface.CommunicationBuffer,TERMINAL_BUFFER_SIZE,Buffer,ByteCount-2);
		    TerminalSendString((char*)APDUInterface.CommunicationBuffer);
		    TerminalSendChar('\n');
		    Communicating = true;
		    return 0 ;

		    if(ByteCounts == 1) return ISO14443A_APP_NO_RESPONSE;// No response
		    Buffer[0] = 2|BlockNumber;
		    if(DataChaining) Buffer[0] |= 0x10;
		    ISO14443AAppendCRCA(Buffer,ByteCounts);
		    ByteCounts +=2;
		    return ByteCounts;
		}
		case 2 : // R Block
		{
			RetryCounter = 0;
			// We get an R Block indicating DataChaining indicate this in response to PC
			if (DataChaining)
			{
				if(BlockNumber == (Buffer[0]&1)) { // Blocknumber not as expected
					// Attempt Retransmission
					memcpy(Buffer,APDUInterface.CommunicationBuffer,RetransmissonLength);
					return RetransmissonLength;
				}
				    BlockNumber = !BlockNumber;
					APDUInterface.Mode = COM_RECEIVE;
				    TerminalSendString("10");
				    TerminalSendChar('\n');
					Communicating = true;
					return 0 ;
			}else{
				if(BlockNumber == (Buffer[0]&1)){ // Attempt Retransmission if Blocknumber not as expected
					memcpy(Buffer, APDUInterface.CommunicationBuffer, RetransmissonLength);
					return RetransmissonLength;
				}
				if( Buffer[0]&0x10){ // if R(NAK)
					ByteCounts = SendRBlock(Buffer, 1);// Acknowledge R Block Received
					ISO14443AAppendCRCA(Buffer,ByteCounts);
					return 3;
				} else { // R(AK)
					if(BlockNumber != (Buffer[0]&1)){
						BlockNumber = !BlockNumber; // toggle if Blocknumber != CurrentBlocknumber
						ByteCounts = SendRBlock(Buffer, 1);// Acknowledge R Block Received
						ISO14443AAppendCRCA(Buffer,ByteCounts);
						return 3;
					}
				}
			}
			break;
		}
		case 3 : //S Block
		{
			RetryCounter = 0;
			if (Buffer[0] & 0x02)
			{
				uint8_t SBlockType = (Buffer[0]>>4) & 0x03;
				if (SBlockType== 0x03) // S(WTX) Block
				{
					Buffer[0] = 0xb2;
					ISO14443AAppendCRCA(Buffer,1);
					return 24;
					// WTX Block response
				}else if (SBlockType == 0)//DESELECT
				{
					State = STATE_IDLE; // Go back to sleep
					Buffer [0] = 0xc0;// SBlockType
					Buffer [0] +=0x02;// sResponse Block DESELECT
					ISO14443AAppendCRCA(Buffer,1);
					return 3;
				}
			}else {
				//Protocol Error or s(Parameter)
			}

			break;
		}
		default:
			break;
		}
	}
	if (RetryCounter>3){
		RetryCounter = 0;
		State= STATE_IDLE;
	}
	else {
		RetryCounter++;
		// Transmission Error ask for Retransmission
		ByteCount = SendRBlock(Buffer,1);
		ISO14443AAppendCRCA(Buffer,ByteCount);
		ByteCount+=2;
		return ByteCount;
	}
	return 0;
}



//Standard Anticollision in Iso14443 with Protocol state for higher layer messages also catches Pc Output
uint16_t EMVAppProcess(uint8_t* const Buffer, uint16_t BitCount)
{
	if (Communicating) // In this mode we wait for Data to arrive from PC
	{

		if (ReceivedData) // Once Data has arrived we send response to Terminal
		{

				uint16_t ByteCount = (ReceivedBitCount +7) >>3;
				memcpy (Buffer, APDUInterface.CommunicationBuffer,ByteCount);
				DataChaining = APDUInterface.CommunicationBuffer[0] &0x10;
				if (BlockNumber) Buffer[0] |= 3;
				else Buffer[0] |= 2;
				if(ByteCount > 254) ByteCount = 254; // ByteCount shall not be grater 254 before adding on 2 CRCA Bytes
				ISO14443AAppendCRCA(Buffer,ByteCount);
				ByteCount +=2;
				CommandLinePendingTaskFinished(COMMAND_INFO_OK_ID,NULL);
				Communicating = false;
				ReceivedBitCount = 0 ;
				ReceivedData = false;
				RetransmissonLength = ByteCount;
				memcpy(APDUInterface.CommunicationBuffer,Buffer,RetransmissonLength);
				return 8*ByteCount;
		}
	}
	else
	{

		uint8_t Cmd = Buffer[0];
		uint16_t ByteCount;
		switch(State) {
		case STATE_IDLE:
		case STATE_HALT:
			FromHalt = State == STATE_HALT;
			if (ISO14443AWakeUp(Buffer, &BitCount, ATQA_VALUE, FromHalt)) {
				/* We received a REQA or WUPA command, so wake up. */
				State = STATE_READY1;
				return BitCount;
			}
			break;

		case STATE_READY1:
			if (ISO14443AWakeUp(Buffer, &BitCount, ATQA_VALUE, FromHalt)) {
				State = FromHalt ? STATE_HALT : STATE_IDLE;
				return ISO14443A_APP_NO_RESPONSE;
			} else if (Buffer[0] == ISO14443A_CMD_SELECT_CL1) {
				/* Load UID CL1 and perform anticollision */
				uint8_t UidCL1[ISO14443A_CL_UID_SIZE];
				/* For Longer UIDs indicate that more UID-Bytes follow (-> CL2) */
				if (ActiveConfiguration.UidSize == 7) {
					MemoryReadBlock(&UidCL1[1], UID_CL1_ADDRESS, UID_CL1_SIZE-1);
					UidCL1[0] = ISO14443A_UID0_CT;
					if (ISO14443ASelect(Buffer, &BitCount, UidCL1, SAK_CL1_VALUE))
						State = STATE_READY2;
				} else {
					MemoryReadBlock(UidCL1, UID_CL1_ADDRESS, UID_SINGLE_SIZE);

					if (ISO14443ASelect(Buffer, &BitCount, UidCL1, SAK_VALUE)) {
						/* invalid, force reload */
						State = STATE_ACTIVE;
					}
				}
				return BitCount;
			} else {
				/* Unknown command. Enter HALT state. */
				State = STATE_HALT;
			}
			break;

		case STATE_READY2:
			if (ISO14443AWakeUp(Buffer, &BitCount, ATQA_VALUE, FromHalt)) {
				State = FromHalt ? STATE_HALT : STATE_IDLE;
				return ISO14443A_APP_NO_RESPONSE;
			} else if (Cmd == ISO14443A_CMD_SELECT_CL2) {
				/* Load UID CL2 and perform anticollision */
				uint8_t UidCL2[ISO14443A_CL_UID_SIZE];

				MemoryReadBlock(UidCL2, UID_CL2_ADDRESS, UID_CL2_SIZE);

				if (ISO14443ASelect(Buffer, &BitCount, UidCL2, SAK_CL2_VALUE)) {
					/* CL2 stage has ended successfully. This means
					 * our complete UID has been sent to the reader. */
					State = STATE_ACTIVE;
				}

				return BitCount;
			} else {
				/* Unknown command. Enter halt state */
				State = STATE_IDLE;
			}
			break;

		case STATE_ACTIVE:
			/* Preserve incoming data length */
			ByteCount = (BitCount + 7) >> 3;
			if (ISO14443AWakeUp(Buffer, &BitCount, ATQA_VALUE, FromHalt)) {
				State = FromHalt ? STATE_HALT : STATE_IDLE;
				return ISO14443A_APP_NO_RESPONSE;
			}
			/* At the very least, there should be 3 bytes in the buffer. */
			if (ByteCount < (1 + ISO14443A_CRCA_SIZE)) {
				State = STATE_IDLE;
				return ISO14443A_APP_NO_RESPONSE;
			}
			/* All commands here have CRCA appended; verify it right away */
			ByteCount -= 2;
			if (!ISO14443ACheckCRCA(Buffer, ByteCount)) {
				Buffer[0] = NAK_CRC_ERROR;
				return NAK_FRAME_SIZE;
			}
			return AppProcess(Buffer, ByteCount);

		case STATE_PROTOCOL:
		{

			ByteCount = (BitCount + 7) >> 3;
			if (ByteCount <3 && ISO14443AWakeUp(Buffer, &BitCount, ATQA_VALUE, FromHalt)){
				// We received a frame that is too small to be handed over ( Transmission of ATS probably failed) so check if its a wakeup call if it is we pretend to be asleep
				// This does serve to reduce time to retry
				State = FromHalt ? STATE_HALT : STATE_IDLE;
				return BitCount;
			}
			else if (Buffer[0] == CMD_RATS && ISO14443ACheckCRCA(Buffer,ByteCount-2)){ // If we receive Rats in Protocol State we still return ATS
				Buffer[0] = 0x05; // specifying length of Standard Message to 5 bytes
				Buffer[1] = DEFAULT_T0;
				Buffer[2] = DEFAULT_TA;//Interfacebyte TA
				Buffer[3] = DEFAULT_TB;// SFWTG and FWT
				Buffer[4] = DEFAULT_TC;//Indicates whether NAD and CID are supported (No support)
				ISO14443AAppendCRCA(Buffer,5);
				return 7*8; // 5 Bytes + 2 CRCA Bytes
			}
			else {
				RetransmissonLength = ProtocolProcess(Buffer, ByteCount);
				if(!Communicating)memcpy(APDUInterface.CommunicationBuffer,Buffer,RetransmissonLength);
				return (8 * RetransmissonLength);
			}
		}

		default:
			/* Unknown state? Should never happen. */
			break;
		}


	}
	/* No response has been sent, when we reach here */
	return ISO14443A_APP_NO_RESPONSE;
}




void EMVGetUid(ConfigurationUidType Uid)
{
	if (ActiveConfiguration.UidSize == 7) {
		//Uid[0]=0x88;
		MemoryReadBlock(&Uid[0], UID_CL1_ADDRESS, UID_CL1_SIZE-1);
		MemoryReadBlock(&Uid[3], UID_CL2_ADDRESS, UID_CL2_SIZE);
	}
	else
		MemoryReadBlock(Uid, UID_CL1_ADDRESS, UID_SINGLE_SIZE);
}

void EMVSetUid(ConfigurationUidType Uid)
{
	if (ActiveConfiguration.UidSize == 7) {
		//Uid[0]=0x88;
		MemoryWriteBlock(Uid, UID_CL1_ADDRESS, ActiveConfiguration.UidSize);
	}
	else {
		uint8_t BCC =  Uid[0] ^ Uid[1] ^ Uid[2] ^ Uid[3];

		MemoryWriteBlock(Uid, UID_CL1_ADDRESS, UID_SINGLE_SIZE);
		MemoryWriteBlock(&BCC, 4, ISO14443A_CL_BCC_SIZE);
	}
}

