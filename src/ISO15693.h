/*
ISO15693 Library for the MFRC522
----------  PREREQUISITES  ----------

- Include the MFRC522 and SPI libraries
- Create at least a 1280-byte buffer.

----------  WARNING  ----------

The major problems are :
  - A 1280 byte buffer is requiered to store analog transmit/receive samples
  - Interrupts are DISABLED when transceiving.
  - Signals are NOT processed in realtime.
    -> Before transmitting, the first part of the buffer is set acccording to the data, and the rest is prepared for receiving
    -> The first part of the buffer is sent to the RC522's TxControlReg in order to transmit, and then samples from the TestADCReg fill the remaining part of he buffer in order to receive
    -> The remaining part of the buffer is processed in multiple steps to decode the data.
  - The sample rate is LOWER than the 423kHz tag's subcarrier. The subcarrier is thus NOT demodulated. But it still works!

----------  FEATURES  ----------

- Transceiving
  -> Reader to card
        Data rate : Only HIGH is supported (26.67kbps, 1 out of 4 coding)
        Modulation : Only 100% is supported, but 10% should be possible
  -> Card to reader
        Data rate : Only HIGH is supported (26.67kbps)
        Modulation : Only single subcarrier is supported
  -> Airtime : 217 bytes of DSP buffer gives 1ms of airtime. (Either transmitting or receiving).
  -> DSP buffer size : a 1280-byte DSP buffer allows to transceive 17 bytes. that's enough to send a 5-byte Inventory request and to receive the 12-byte tag response.

- Inventory.
  -> Anticollision is NOT implemented.
  -> Only 1 timeslot is supported
  -> If more than one tag is present in the field, the library will report that no tag is detected.

- Select

- Read single block
  -> Only 4-byte blocks are supported
  -> I will add support for block lock status
  -> ONLY selected mode is supported.

- Write single block
  -> Only a 4 byte block can be written
  -> Because of the very limited airtime I can not get the response from the card so I need to read the block after reading it.
  -> I will add support for the Option flag to get response after an EOF

- Reset to ready

----------  TIMING  ----------

I define an ETU (Elementary timing unit) as a 9.44us delay, even if no such ETU is defined in the ISO standard.
*/


#ifndef DEF_ISO15693
#define DEF_ISO15693

#include <SPI.h>
#include <MFRC522.h>

class ISO15693
{
	public:
	
	enum commandCodes : byte//Command codes supported by the library.
	{
		INVENTORY_CMD 			= 0x01,
		READ_SINGLE_BLOCK_CMD 	= 0x20,
		WRITE_SINGLE_BLOCK_CMD 	= 0x21,	
		SELECT_CMD 				= 0x25,
		RESET_TO_READY_CMD 		= 0x26,
	};
	
	enum flags : byte//List of the different flags than can be set when sending a command. To set multiple flags, do a bitwose OR between them. Exampls : DATA_RATE_FLAG | INVENTORY_FLAG
	{
		SUBCARRIER_FLAG 		= 0b00000001,//Do not set this flag, as only one subcarrier is supported
		DATA_RATE_FLAG 			= 0b00000010,//Don't forget to set this flag when manually sending a command, as only high data rate is supported !
		INVENTORY_FLAG 			= 0b00000100,//Shall be set whne sending an INVENTORY command
		PROTOCOL_EXTENSION_FLAG = 0b00001000,//RFU
		
		SELECT_FLAG 			= 0b00010000,//If set, command will only be executed by the tag in SELECTED state.
		ADDRESS_FLAG 			= 0b00100000,//If set, only tag whose UID is included after the command will execute the command
		OPTION_FLAG 			= 0b01000000,//Meaning is defined by the command
		RFU_FLAG 				= 0b10000000,//RFU (Reserved for Future use)
		
		AFI_FLAG				= 0b00010000,//When the INVENTORY flag is set, the AFI flag signals that anly tags whose AFI matches the specified AFI shall respond. (Not used in this library).
		NB_SLOTS_FLAG			= 0b00100000//When the inventory flag is set, this flag signals that can only respond within 1 time slot, that is, right after the command has been set.
	};
	
	ISO15693(MFRC522 *rfid, byte DSPBuffer [], int DSPBufferLength);//Constructor
	
	//Internally used.
	void generateRXBuffer(byte buf [], int len);
	
	void transferBuffer(byte buffer [], int len);
	
	int getBitPair(byte buf [], int idx);
	
	void addETUs(byte buf [], int &idx, int nbETUs);
	void addPause(byte buf [], int &idx);
	
	int generateTXBuffer(byte cmd[], int cmdLen, byte buf[], int buflen);
	
	void comp2amp(byte buf [], int len);
	
	void discretize(byte buf [], int len, int discrPeriod);
	
	bool lowToHigh(byte buf [], int idx);
	bool highToLow(byte buf [], int idx);
	bool detectPattern(byte buf [], int bufIdx, bool pattern [], int patternLen);
	bool detectSF(byte buf [], int len, int &SFidx);
	bool detectEF(byte buf [], int len, int &EFidx);
	void setBit(byte buf [], int bitIdx, bool bitValue);
	bool searchForTransition(byte buf [], int len, int &idx, bool &bitValue);
	
	int decodeBuffer(byte rxBuffer [], int rxlen, byte dataBuffer [], int nbDataBytes);
	int calculateCRC(byte data [], int len);
	
	bool verifyCRC(byte data [], int len);
	void appendCRC(byte data [], int len);
	
	int transceive15693(byte buffer [], int bufferLength, int bytesToTransmit, bool receiveLater = false);
	
	//Predefined commands.
	bool inventory(byte uid [8]);//Returns true and copies the UID in reversed order into UID if a tag is within the reader's range.
	bool readSingleBlock(byte blockNumber, byte data [4]);//Returns true if a block has successfully been read and copies 4 bytes into the data array. Only tags in SELECTED mode will respond to this command.
	bool writeSingleBlock(byte blockNumber, byte data [4]);//Writes 4 bytes into block blockNumber and returns true if successful. Only tags in SELECTED mode will respond to this command.
	bool select(byte uid [8]);//Returns true if a card whose UID matches the uid array has been selected. (UID is in reversed order as for the INVENTORY command).
	bool resetToReady();//returns true if a tag in the SELECTED state has successfully been unselected.
	
	bool tagError();//Returns true if a tag responded with an error to the last command.
	byte getErrorCode();//Get the last error code that has been returned by a tag.
	
	private:
	byte *_DSPBuffer;//Unified signal processing buffer, divided between the Transmit and Receive buffers.
	int _DSPBufferLength;//Length of the unified DSP buffer
	
	//These have been defined as member variables only to facilitate debug.
	int _txLength;//Length of the TX buffer
	int _rxLength;//Length of the RX buffer
	byte *_rxBuffer;//Pointer ot the beginning of the RX buffer.
	
	const int _INTBUFLEN = 32;//Internal command transceiving buffer.
	byte _internalBuffer [32];
	
	MFRC522 *_rfid;//RC522 object.
		
	bool _tagError = false;//Set to true if  a tag responded to the last request with an error
	byte _errorCode = 0x00;//Contains the last error code which has been received.
};

#endif