#include <SPI.h>
#include <MFRC522.h>
#include <ISO15693.h>

MFRC522 rfid(3, 2);//RST and CS pins, change accordingly to yur circuit

byte DSPBuffer [1280];//Array that will store transmitted/received samples

byte uid [8];//UID (stored in reverse, because it is transmitted/received that way in the 15693 standard)

bool detectedLastTime = false;
bool detected = false;

ISO15693 iso15693(&rfid, DSPBuffer, sizeof(DSPBuffer));//Initialize the 15693 transceiver with specified RC522 module and buffer.

void setup() {
  SPI.begin();
  Serial.begin(115200);
  Serial.println("Reader started");
  rfid.PCD_Init();//Init RC522 module (must be done manually)
}

void loop() {
  detectedLastTime = detected;
  detected = iso15693.inventory(uid);//Check if a tag is in the field. If yes, its UID is saved.
  if(detected && !detectedLastTime)//If we detected a tag this time and not the previous time, print the UID on the serial monitor.
  {
    Serial.print("Tag detected UID : ");
    for(int idx = 7; idx >= 0; idx--)
    {
      if(uid[idx] < 0xF)
        Serial.print('0');
      Serial.print(uid[idx], HEX);
      Serial.print(' ');
    }
    Serial.println();
    delay(1000);
  }
}
