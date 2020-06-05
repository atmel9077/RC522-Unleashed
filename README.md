# RC522-Unleashed
Arduino library to detect/read/write to ISO15693 tags with the MFRC522. Very experimental and unreliable. Requires normal RC522 library.

> ## Note
>
> In this wiki the terms "card" and "tag" are used interchangeably.

## Requirements

- Include the MFRC522 library
- Create an 1280-byte array. It will be used to process the signals that will be sent to and received from the tags.

## Warning

There are yet major limitations to this library : 

- A 1280-byte buffer is requiered to store analog samples to be transmitted/that have been received.
- **Interrupts are disabled when transceiving**
- Signals are **not** processed in real-time
  - Before transmitting, the first part of the buffer is filled with samples representing the data to be sent.  
  - The precalculated samples are sent to the RC522's TxControlReg at a rate of 230.9Ksamples/s in order to periodically switch off the carrier according to the data to be transmitted.  
  - While receiving, the remaining part of the buffer which wasn't used for transmitting is filled with analog samples read from the RC522/s TestADCReg ar a rate of 230.9Ksamples/s.
  - When the buffer is full, the data is processed in several steps, namely :  
    -Complex samples are converted into real samples.  
    -The signal is lowpass-filtered and discretized to binary levels.  
    -The discretized signal is decoded into bits and bytes  
- **The sampling rate is lower than the tag's subcarrier frequency (230.9 vs 423kHz)** Thus, only the less robust DC component of the modulated data is recovered.
- The sample rate is a compromise between signal quality and RAM occupancy. **Note that the SPI bus speed is set to 2Mbps/250KBps when transceiving, but the lack of double-buffering in the ATMEGA328P results in the odd and ugly 230.9ksps sample rate**

## Communication parameters

- Reader to card
  - Only HIGH data rate is supported (26.67kbps, 1 out of 4 coding)  
  - Only 100% modulation index is supported (had no luck with 10%)  
- Card to reader
  - Only HIGH data rate is supported (26.67kbps)  
  - Only signle subcarrier is supported  
- Transceive length : The recommended 1280-byte DSP buffer allows to transmit/receive 17 bytes (enough for the reader to send an INVENTORY request, and for the card to respond.)

## Supported commands

- Inventory.
  - Anticollision is NOT implemented.
  - Only 1 timeslot is supported
  - If more than one tag is present in the field, the library will report that no tag is detected.

- Select

- Read single block
  - Only 4-byte blocks are supported
  - I will add support for block lock status
  - ONLY selected mode is supported.

- Write single block
  - Only a 4 byte block can be written
  - Only selected mode is supported.
- Reset to ready

## Timing definition

I defined an ETU or Elementary Timing Unit as 9.44 microseconds (2.18 samples at 230.9ksps), **despite there is no such definition in the ISO15693 standard**. It can make some explanations easier.

## Functions

### Constructor

`ISO15693(MFRC522 *rfid, byte DSPBuffer [], int DSPBufferLength)`

#### Description

Creates and initializes an ISO15693 object, which will be used to comminucate with ISO15693 tags through an atttached RC522.

#### Parameters

- MFRC522 *rfid : Pointer to an RC522 object
- byte DSPBuffer [] : Pointer to a byte array. It should be 1280-bytes long.
- int DSPBufferLength : Length of the aforementioned buffer. Should be 1280.

### Inventory

`bool inventory (byte uid)`

#### Description

Sends an Inventory request to any tag in the field of the reader. If **one and only one** tag is present in the reader's field, this function will return true and the received UID will copied into the UID array.

#### Parameters

- byte uid [8] : Pointer to an 8-byte array in which the UID will be stored in reverse order.

#### Return value

- bool : Returns true if a tag responded to the INVENTORY request. UID is changed only if a tag responded to the command.

### Read Single Block

`bool readSingleBlock(byte blockNumber, byte data [4])`

#### Description

Reads 4 bytes from block blockNumber. **Only cards in the Selected state will respond to this command. The card must have previously been put into the Selected state by sending a Select command.**. If a card responds, this function will return true and the received data will be copied into the data array.

#### Parameters

- byte blockNumber : The number of the block to be read (Number of available blocks depends on tag used).
- byte data [4] : Pointer to a 4-byte array into which the data read from the block will be stored

#### Return value

- bool : Returns true if reading successful. Data is changed only if reading sccessful.

### Write Single Block

`bool writeSingleBlock(byte blockNumber, byte data [4])`

#### Description

Writes the 4 bytes contained in data at block blockNumber. **Only cards in the Selected state will respond to this command**. If the card reports that the block has successfully been written, this function will return true.

#### Parameters

- byte blockNumber : The number of the block to be written to (Number of available blocks depends on tag used).
- byte data [4] : Pointer to a 4-byte array containing the data to be written into the block.

#### Return value

- bool : Returns true **if the tag reports that the block has successfully been written**

> ##### Warning
> 
> If this function returns false, **it does not necessarily means that writing the block failed**  
> The receiving part is still very unreliable. When the cards responds having successfully written the block,  
> the reader will sometimes fail to decode the message and returns false.  
> If this function returns false and tagError returns false, it usually means that the reader failed  
> to receive acknowledge from the card.

### Select

`bool select(byte uid [8])`

#### Description

Puts the card whose UID is specified into the **Selected** state. Note that any other card which happens to be in Selected sate and whose UID does not match the specified one will be reset to the Ready state. This function returns true if the card reports having successfully been selected.

> ##### Note
>
> Only one card shall be in the Selected state at a given time.

#### Parameters

- byte uid [8] : Pointer to an 8-byte array containing the specific card UID in reversed form. (Same format as returned by card to Inventory requests)

#### Return value

- bool : Returns true if the card reports being successfully selected.

### Reset to Ready

`bool resetToReady()`

#### Description

Resets any card which happens to be in the Selected state back to the Ready state. This function will return true if the card reports having successsfully been reset to the Ready state.

#### Return value

- bool : Returns true if the card reported being successfully reset to the Ready state.

### tagError

`bool tagError()`

#### Return value

- bool : Returns true if a tag responded to the last command with an error code.

### getErrorCode

`byte getErrorCode()`

#### Return value

- byte : The last error code which has been received from a tag.

## Error handling

Unlike most conventional approaches which uses status codes, I decided that the relative simplicity of this library requiered a simple error handling mechanism.  

### Types of errors

There can be two types of errors, namely:

#### Tag errors

These errors occur when a tag cannot process a specific request.  
This will happen, for example, if the user tries to read more block than there are in the card's memory.  
In this case, the tag will respond with an ISO15693 error code.  

#### Communication errors

These errors occur when the reader fails to send and/or receive messages to and from the card.  
This can occur from several reasons including :  

- The tag is too far from, or improperly coupled with the reader. This results in a weak signal, which will not be properly decoded (Bad CRC, Failure to detect SOF/EOF symbol)
- An improperly addressed command has been sent, such as a Select command with a wrong UID, or read/write commands if no tag in the Selected state are present in the reader's field. In this case, cards will not responds.
- Transmit buffer overflow. This happens if (70 * number of bytes to transmit + 80) exceeds the length of the DSP bufffer. This should not happen when using the predefined inventory/read/write/select/reset to ready commands.
- Receive buffer overflow. This happens if (70 * (number of bytes to transmit + number of bytes received) + 80) exceeds the length of the DSP bufffer. In this case the received signal does not fit inside the DSP buffer, gets truncated and canot get decoded. This should not happen when using predefined commands.

> Note that it is impossible to distinguish between these errors. It is up to the user to verify that the transmitted and received data fits into the DSP buffer. The recommended 1280-byte DSP buffer allows up to 17 bytes to be transmited or received. (e.g. 5 bytes transmitted and 12 received, **including both transmitted an received CRC**.

### Dealing with errors

- If either error occured, predefined functions (Inventory/Select/read/Write/Reset to Ready) will return false. The error type shall be determined through the **tagError** function.
- If the last error to have occured was a **tag** error, the tagError function will return true, false otherwise. The error can be further determined through the **getErrorCode** function.

Here is a list of the standardized ISO15693 error codes :

> ISO15693 error codes
>
> - 0x01 : The command is not supported, i.e. the request code is not recognized.
> - 0x02 : The command is not recognized, for example: a format error occurred.> 
> - 0x0F : Unknown error.
> - 0x10 : The specified block is not available (doesn’t exist).
> - 0x11 : The specified block is already -locked and thus cannot be locked again
> - 0x12 : The specified block is locked and its content cannot be changed.
> - 0x13 : The specified block was not successfully programmed.
> - 0x14 : The specified block was not successfully locked.
> - 0xA0 thru 0xDF : Custom command error codes
> - All others : Reserved for Future Use.

## TODO

- Add comments and documentation (coming very soon)!!!!
- Add more explanations about how the hell this can ever work (even if FAR from working well).
- Rebuild the demodulator with correlators and digital filters.
- **Improve error handling**
