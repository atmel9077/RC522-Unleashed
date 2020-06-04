# RC522-Unleashed
Arduino library to detect/read/write to ISO15693 tags with the MFRC522. Very experimental and unreliable. Requires normal RC522 library.

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
- **The sampling rate is lower than the tag's subcarrier frequency (230.9 vs 423kHz)** Thus, only the less robust DC component of the modulated data is recivered.
- The sample rate is a compromise between signal quality and RAM occupancy. **Note that the SPI bus speed is set to 2Mbps/250KBps when transceiving, but the lack of double-buffering in the ATMEGA328P results in the odd and ugly 230.9ksps sample rate**

## Communication parameters

- Reader to card
  - Only HIGH data rate is supported (26.27kbps, 1 out of 4 coding)  
  - Only 100% modulation index is supported (had no luck with 10%)  
- Card to reader
  - Only HIGH data rate is supported (26.67kbps)  
  - Only signle subcarrier is supported  
- Transceive length : The recommended 1280-byte DSP buffer allows to ttransmit/receive 17 bytes (enough for an INVENTORY cmd)

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
  - Because of the very limited airtime I can not get the response from the card so I need to read the block after reading it.
  - I will add support for the Option flag to get response after an EOF

- Reset to ready

## Timing definition

I defined an ETU or Elementary Timing Unit as 9.44 microseconds (2.18 samples at 230.9ksps), **despite there is no such definition in the ISO15693 standard**. It can make some explanations easier.

## TODO

- Add comments and documentation (coming very soon)!!!!
- Add more explanations about how the hell this can ever work (even if FAR from working well).
- Rebuild the demodulator with correlators and digital filters.
