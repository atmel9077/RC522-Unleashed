#include <ISO15693.h>

ISO15693::ISO15693(MFRC522 *rfid, byte DSPBuffer [], int DSPBufferLength)
{
	_rfid = rfid;//Copy pointer to mfrc522
	_DSPBuffer = DSPBuffer;//Copy pointer to DSP buffer
	_DSPBufferLength = DSPBufferLength;//Copy buffer length
}

//Prepare a buffer buf of length len for receiving
void ISO15693::generateRXBuffer(byte buf [], int len)
{
  for(int i=0; i<len; i++)//Fill the entire buffer with 0xF6. (Tells the RC522 to sand back the contents of the TestADCReg register).
  {
    buf[i] = 0xF6;//READ TestADCReg's address (digitized complex baseband signal)
  }
  buf[len - 1] = 0x00;//Terminate the buffer with 0x00 to tell the RC522 that reading is finished
}

//Transfer a buffer buf of length len to the RC522. When reading, the buffer will be filled by the data returned from the RC522
void ISO15693::transferBuffer(byte buffer [], int len)
{
  noInterrupts();//Disable interrupts
  SPI.setClockDivider(SPI_CLOCK_DIV8);//Set RAW data rate to 2Mbps = 250Kbytes/s
  digitalWrite(3, LOW);//Enable rc522 chip select
  SPI.transfer(buffer, len);//Transfer the buffer
  digitalWrite(3, HIGH);//Disable chip select
  SPI.setClockDivider(SPI_CLOCK_DIV4);//Set data rate back to 4Mbps
  interrupts();//Re-enable interrupts.
}

//Get a bit pair at index idx from a buffer. A n-byte buffer contains 4*n bit pairs and they are numbered LSB-first. Index 0 = 2 LSBs of byte 0, index 3 = 2 MSBs of byte 0
//This function returns the value of the bit pair.
int ISO15693::getBitPair(byte buf [], int idx)
{
  return (buf[idx >> 2] >> (2 * (idx & 0b11))) & 0b11;
}

//Add nbETUs * 2.18 0x00 samples to the buffer and update the index. The number of samples is rounded to the nearest integer. 1 etu = 9.44us = 2.18 samples
//0x00 means carrier on. (InvMod bit not set).
void ISO15693::addETUs(byte buf [], int &idx, int nbETUs)
{
  float nbSamples;
  int nbSamplesInteger;
  nbSamples = nbETUs * 2.18;
  nbSamplesInteger = roundf(nbSamples);
  for(int cnt = 0; cnt < nbSamplesInteger; cnt++)
  {
    buf[idx] = 0x00;
    idx++;
  }
}

//Add 2 0x08 samples to the buffer and update the index. This creates a pause which duration is close enough to 9.44us.
//0x08 means carrier off. (InvMod bit set).
void ISO15693::addPause(byte buf [], int &idx)
{
  buf[idx] = 0x08;
  buf[idx + 1] = 0x08;
  idx += 2;
}

//Prepare a transmit buffer buf of length len, from the data in the command buffer cmd of length cmdLen.
//This function returns the number of bytes written to the buffer.
//TODO : LENGTH CHECKING
int ISO15693::generateTXBuffer(byte cmd[], int cmdLen, byte buf[], int buflen)
{
  int numBitPairs = 4 * cmdLen;
  int idx = 0;//Reset idx
  if(cmdLen * 70 < buflen - 80)
  {
	  buf[0] = 0x24;//Write to TxModeReg register
	  idx = 1;
	  addPause(buf, idx);//Add the SOF
	  addETUs(buf, idx, 4);
	  addPause(buf, idx);
	  addETUs(buf, idx, 3 + (2 * getBitPair(cmd, 0)));//Add the time between the last pulse of the EOF and the pulse of the first bit pair.
	  addPause(buf, idx);
	  for(int bitPairIdx = 1; bitPairIdx < numBitPairs; bitPairIdx++)//For each bit pair:
	  {
		addETUs(buf, idx, 7 + (2 * (getBitPair(cmd, bitPairIdx) - getBitPair(cmd, bitPairIdx - 1))));//Add a delay equal to 7 ETUs + 2 times the difference between the current and the last bit pair
		addPause(buf, idx);//Pulse the field
	  }
	  addETUs(buf, idx, 8 - (2 * getBitPair(cmd, numBitPairs   - 1)));//Add the delay between the last bit pair's pulse and the End of frame's pulse
	  addPause(buf, idx);
	  addETUs(buf, idx, 1);//Add an 1 ETU delay to turn the carrier back on
  }
  return idx;
}

void ISO15693::comp2amp(byte buf [], int len)//Convert the complex I/Q samples that have been returned by the RC522 into a real amplitude
{
  int i;
  int q;
  for(int idx = 0; idx < len; idx++)//Read the entire buffer
  {
    i = (buf[idx] >> 4) & 0b1111;//Get the Inphase & Quadrature samples. Inphase is in the 4 MSBs, Quadrature in the 4 LSBs.
    q = buf[idx] & 0b1111;
    if(i > 7)//Convert I and Q from 4-bit two-s complement signed to a standard int.
      i -= 16;
    if(q > 7)
      q -= 16;
    buf[idx] = sqrt((i * i) + (q * q));//Calculate the resulting amplitude.
  }
}

void ISO15693::discretize(byte buf [], int len, int discrPeriod)//Convert the variable amplitude into a binary signal. discrPeriod is the number of samples on which the signal is averaged.
{
  int rollingAverage = 0;
  int ampFactor = discrPeriod / 3;
  
  int usefulLen = 0;
  
  int averagedSample = 0;
  bool discretizedSample = 0;
  
  if(len >= discrPeriod && len >= 3)//If the buffer is shorter than discrPerod (discretization rolling average period) or 3 samples (filtering rolling average length), we exit.
  {  
	  for(int idx = discrPeriod; idx < len - 4; idx++)//We set usefulLen to the index of 4 samples after the LAST amplitude to be greater than 1.(Squelch )
	  {
		  if(buf[idx] > 1)
		  {
			  usefulLen = idx + 4;
		  }
	  }
	  for(int idx = 0; idx < discrPeriod; idx++)//We initialize the rolling average over discrPeriod samples.
	  {
		rollingAverage += buf[idx];
	  }
	  for(int idx = 0; idx < usefulLen; idx++)//For each sample until usefulLen is reached
	  {
		averagedSample = buf[idx] + buf[idx + 1] + buf[idx + 2];//We average the signal over 3 samples to remove noise.
		discretizedSample = averagedSample * ampFactor > rollingAverage;//We multiply the signal by ampFactor to scale it with the rolling average. If the scaled and filtered signal is higher than the rolling average, the sample is discretized as one.
		if(idx < usefulLen - discrPeriod)//If we can still "advance" the rolling average, we advance it by one sample.
		{
		  rollingAverage -= buf[idx];
		  rollingAverage += buf[idx + discrPeriod];
		}
		buf[idx] = discretizedSample;//We replace the sample by its discretized version.
	  }
  }
}

bool ISO15693::lowToHigh(byte buf [], int idx)//Returns true if a low to high transition occurs after index idx in the buffer.
{
  return buf[idx] == 0 && buf[idx + 1] == 1;
}

bool ISO15693::highToLow(byte buf [], int idx)//Same but with a high to low transition.
{
  return buf[idx] == 1 && buf[idx + 1] == 0;
}

bool ISO15693::detectPattern(byte buf [], int bufIdx, bool pattern [], int patternLen)//Detect a pattern  of length patternLen at index idx. idx is where the first ETU starts. This function will compare at the middle of each ETU.
{
  int nbDetected = 0;
  for(int patternIdx = 0; patternIdx < patternLen; patternIdx++)
  {
    if(buf[roundf(bufIdx + (2 * patternIdx + 1) * 2.18)] == pattern[patternIdx])
      nbDetected++;
  }
  return nbDetected == patternLen;
}

bool ISO15693::detectSF(byte buf [], int len, int &SFidx)//Detect a start of frame at index idx. Idx is where the first ETU starts
{
  const bool SFPattern [5] = {1, 1, 1, 0, 1};
  bool detected = false;
  int idx = 0;
  do
  {
    if(lowToHigh(buf, idx))//If a low to high transition is found
    {
      if(detectPattern(buf, idx, SFPattern, 5))//We check whether we can find a start of frame.
      {
        detected = true;
        SFidx = idx;
      }
    }
    idx++;
  }
  while(!detected  && idx < len);
  return detected;
}

bool ISO15693::detectEF(byte buf [], int len, int &EFidx)//Same but with an end of frame.
{
  const bool EFPattern [5] = {0, 1, 1, 1, 0};
  bool detected = false;
  int idx = 0;
  do
  {
    if(highToLow(buf, idx))//If a high to low transition is found
    {
      if(detectPattern(buf, idx, EFPattern, 5))//We check whether we can find an end of frame.
      {
        detected = true;
        EFidx = idx;
      }
    }
    idx++;
  }
  while(!detected  && idx < len);
  return detected;
}

void ISO15693::setBit(byte buf [], int bitIdx, bool bitValue)//Set a bit at index bitIdx if bitValue is set.  THIS CANNOT RESET BITS. THE BUFFER MUST BE RESET FIRST!
{
  buf[bitIdx >> 3] |= bitValue << (bitIdx & 0b111);//The byte index is given by the index's MSBs (starting from bit 3), the 3 LSBs indicate the bit index.
}

bool ISO15693::searchForTransition(byte buf [], int len, int &idx, bool &bitValue)//Start searching for a transition at index idx and stop when a transition occurs between buf[idx] and buf[idx + 1] or if the end of the buffer is reached
																					//Return true if a transition has been found otherwise false. bitValue is set to one if a low to high transition is detected.
{
  bool transitionDetected = false;
  do
  {
    if(lowToHigh(buf, idx))
    {
      transitionDetected = true;
      bitValue = true;
    }
    else if(highToLow(buf, idx))
    {
      transitionDetected = true;
      bitValue = false;
    }
    else
      idx++;
  }
  while(!transitionDetected && idx < len);
  return transitionDetected;
}

int ISO15693::decodeBuffer(byte rxBuffer [], int rxlen, byte dataBuffer [], int nbDataBytes)//Try decoding a discretised buffer rxBuffen of length rxlen and put the data in dataBuffer of length nbDataBytes
																							//Returns the number of received bytes if start and end of frame are found, number of bits is multiple of 8 and data fits into dataBuffer, otherwise returns 0
{
  int bufIdx = 0;
  int dataIdx = 0;
  int SFidx;
  int EFidx;
  bool transitionDetected = true;
  bool bitValue;
  int nbDataBits = 8 * nbDataBytes;
  for(int i = 0; i < nbDataBytes; i++)
    dataBuffer[i] = 0;
  if(detectSF(rxBuffer, rxlen, SFidx))//Try detecting a start of frame
  {
    if(detectEF(rxBuffer, rxlen, EFidx))//Then try detecting an end of frame
    {
      bufIdx = SFidx + 24;//Go 24 samples after the start of frame
      while(bufIdx < EFidx - 4 && dataIdx < nbDataBits && transitionDetected)//While we haven't reached the end of the discretizd buffer and the end of the data buffer and we found a transition last time we searched for one
      {
        transitionDetected = searchForTransition(rxBuffer, rxlen, bufIdx, bitValue);//We search for a transition and get bitBalue
        if(transitionDetected)//If one is detected
        {
          setBit(dataBuffer, dataIdx, bitValue);//We set a bit in the data buffer in case of a low to high transiton
          dataIdx++;//We go to the next data bit
          bufIdx += 7;//We go 7 samples further to end up in the first part of the next sample
        }
      }
    }
  }
  if(!transitionDetected || dataIdx > nbDataBits || dataIdx % 8 != 0)//Of no transition detected at all or received too many bits or not a multiple of 8 bits
    dataIdx = 0;//We return 0 (0 valid bytes=
  else
    dataIdx /= 8;//If everything is ok we divide the number of bits by 8 to get a number of bytes
  return dataIdx;//We return the number of bytes
}

/*int ISO15693::decodeBuffer(byte rxBuffer [], int rxlen, byte dataBuffer [], int nbDataBytes)//Try decoding a discretised buffer rxBuffen of length rxlen and put the data in dataBuffer of length nbDataBytes
																							//Returns the number of received bytes if start and end of frame are found, number of bits is multiple of 8 and data fits into dataBuffer, otherwise returns 0
{
  int bufIdx = 0;
  int dataIdx = 0;
  int SFidx;
  int EFidx;
  int difference;
  bool bitValue;
  int nbDataBits = 8 * nbDataBytes;
  for(int i = 0; i < nbDataBytes; i++)
    dataBuffer[i] = 0;
  if(detectSF(rxBuffer, rxlen, SFidx))//Try detecting a start of frame
  {
    if(detectEF(rxBuffer, rxlen, EFidx))//Then try detecting an end of frame
    {
	  bufIdx = SFidx + 22;
      while(bufIdx < EFidx - 12 && dataIdx < nbDataBits)//While we haven't reached the end of the discretizd buffer and the end of the data buffer and we found a transition last time we searched for one
      {
		difference = rxBuffer[bufIdx + 4] + rxBuffer[bufIdx + 5] + rxBuffer[bufIdx + 6] + rxBuffer[bufIdx + 7] - rxBuffer[bufIdx + 0] - rxBuffer[bufIdx + 1] - rxBuffer[bufIdx + 2] - rxBuffer[bufIdx + 3];
        setBit(dataBuffer, dataIdx, difference >= 0);
		dataIdx++;
		bufIdx = 22 + roundf(8.72 * dataIdx);
      }
    }
  }
  if(dataIdx > nbDataBits)//Of no transition detected at all or received too many bits or not a multiple of 8 bits
    dataIdx = 0;//We return 0 (0 valid bytes=
  else
    dataIdx /= 8;//If everything is ok we divide the number of bits by 8 to get a number of bytes
  return dataIdx;//We return the number of bytes
}*/

int ISO15693::calculateCRC(byte data [], int len)
{
  const unsigned int POLYNOMIAL = 0x8408;
  unsigned int current_crc_value = 0xFFFF;
  for (int i = 0; i < len; i++)
  {
    current_crc_value = current_crc_value ^ ((unsigned int)data[i]);
    for (int j = 0; j < 8; j++)
    {
      if (current_crc_value & 0x0001)
      {
        current_crc_value = (current_crc_value >> 1) ^ POLYNOMIAL;
      }
      else
      {
        current_crc_value = (current_crc_value >> 1);
      }
    }
  }
  return current_crc_value;
}

bool ISO15693::verifyCRC(byte data [], int len)
{
  return calculateCRC(data, len) == 0xF0B8;
}

void ISO15693::appendCRC(byte data [], int len)
{
  if(len > 2)
  {
    unsigned int calculatedCRC = calculateCRC(data, len - 2);
    data[len - 2] = ~calculatedCRC & 0xFF;
    data[len - 1] = ~(calculatedCRC >> 8) & 0xFF;
  }
}

int ISO15693::transceive15693(byte buffer [], int bufferLength, int bytesToTransmit, bool receiveLater = false)
{
	int bytesReceived = 0;
	if(bytesToTransmit < bufferLength - 2)
	{
		appendCRC(buffer, bytesToTransmit + 2);
		int _txLength = generateTXBuffer(buffer, bytesToTransmit + 2, _DSPBuffer, _DSPBufferLength);
		_rxBuffer = _DSPBuffer + _txLength;
		_rxLength = _DSPBufferLength - _txLength;
		generateRXBuffer(_rxBuffer, _rxLength);
		transferBuffer(_DSPBuffer, _txLength);
		if(receiveLater)
		{
			_DSPBuffer[0] = 0x24;
			_DSPBuffer[1] = 0x08;
			_DSPBuffer[2] = 0x08;
			_DSPBuffer[3] = 0x00;
			delay(20);
			transferBuffer(_DSPBuffer, 4);
		}
		_rfid->PCD_WriteRegister(MFRC522::RFCfgReg, 0x70);
		_rfid->PCD_WriteRegister(MFRC522::CommandReg, MFRC522::PCD_Receive);
		transferBuffer(_rxBuffer, _rxLength);
		comp2amp(_rxBuffer, _rxLength);
		discretize(_rxBuffer, _rxLength, 24);
		bytesReceived = decodeBuffer(_rxBuffer, _rxLength, buffer, bufferLength);
		if(verifyCRC(buffer, bytesReceived))
			bytesReceived -= 2;
		else
			bytesReceived = 0;
	}
	return bytesReceived;
}

bool ISO15693::inventory(byte uid [8])
{
	bool success = false;
	
	_tagError = false;
	
	_internalBuffer[0] = DATA_RATE_FLAG | INVENTORY_FLAG | NB_SLOTS_FLAG;
	_internalBuffer[1] = INVENTORY_CMD;
	_internalBuffer[3] = 0x00;
	
	if(transceive15693(_internalBuffer, _INTBUFLEN, 3))
	{
		if((_internalBuffer[0] & 0x01) == 0x00)
		{
			memcpy(uid, &_internalBuffer[2], 8);
			success = true;
		}
		else
		{
			_tagError = true;
			_errorCode = _internalBuffer[1];
		}
	}
	
	return success;
}

bool ISO15693::readSingleBlock(byte blockNumber, byte data [4])
{
	bool success = false;
	
	_tagError = false;
	
	_internalBuffer[0] = DATA_RATE_FLAG | SELECT_FLAG;
	_internalBuffer[1] = READ_SINGLE_BLOCK_CMD;
	_internalBuffer[2] = blockNumber;
	
	if(transceive15693(_internalBuffer, _INTBUFLEN, 3))
	{
		if((_internalBuffer[0] & 0x01) == 0x00)
		{
			memcpy(data, &_internalBuffer[1], 4);
			success = true;
		}
		else
		{
			_tagError = true;
			_errorCode = _internalBuffer[1];
		}
	}
	
	return success;
}

bool ISO15693::writeSingleBlock(byte blockNumber, byte data [4])
{
	bool success = false;
	
	_tagError = false;
	
	_internalBuffer[0] = DATA_RATE_FLAG | SELECT_FLAG | OPTION_FLAG;
	_internalBuffer[1] = WRITE_SINGLE_BLOCK_CMD;
	_internalBuffer[2] = blockNumber;
	
	memcpy(&_internalBuffer[3], data, 4);
	
	/*transceive15693(_internalBuffer, _INTBUFLEN, 7);
	
	delay(10);
	
	_internalBuffer[0] = DATA_RATE_FLAG | SELECT_FLAG;
	_internalBuffer[1] = READ_SINGLE_BLOCK_CMD;
	_internalBuffer[2] = blockNumber;
	
	if(transceive15693(_internalBuffer, _INTBUFLEN, 3) == 5)
	{
		if((_internalBuffer[0] & 0x01) == 0x00)
		{
			if(_internalBuffer[1] == data[0] && _internalBuffer[2] == data[1] && _internalBuffer[3] == data[2] && _internalBuffer[4] == data[3])
				success = true;
		}
	}*/
	
	if(transceive15693(_internalBuffer, _INTBUFLEN, 7, true))
	{
		if((_internalBuffer[0] & 0x01) == 0x00)
			success = true;
		else
		{
			_tagError = true;
			_errorCode = _internalBuffer[1];
		}
	}
	
	return success;
}

bool ISO15693::select(byte uid [8])
{
	bool success = false;
	
	_tagError = false;
	
	_internalBuffer[0] = DATA_RATE_FLAG | ADDRESS_FLAG;
	_internalBuffer[1] = SELECT_CMD;
	
	memcpy(&_internalBuffer[2], uid, 8);
	
	if(transceive15693(_internalBuffer, _INTBUFLEN, 10) == 1)
	{
		if((_internalBuffer[0] & 0x01) == 0x00)
			success = true;
		else
		{
			_tagError = true;
			_errorCode = _internalBuffer[1];
		}
	}
	
	return success;
}

bool ISO15693::resetToReady()
{
	bool success = false;
	
	_tagError = false;
	
	_internalBuffer[0] = DATA_RATE_FLAG | SELECT_FLAG;
	_internalBuffer[1] = RESET_TO_READY_CMD;
	
	if(transceive15693(_internalBuffer, _INTBUFLEN, 2) == 1)
	{
		if((_internalBuffer[0] & 0x01) == 0x00)
			success = true;
		else
		{
			_tagError = true;
			_errorCode = _internalBuffer[1];
		}
	}
	
	return success;
}

bool ISO15693::tagError()
{
	return _tagError;
}

byte ISO15693::getErrorCode()
{
	return _errorCode;
}