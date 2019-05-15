/*
@author Tadeusz Studnik https://rasmatic.pl

MIT License

Copyright (c) [2019] [Tadeusz Studnik https://rasmatic.pl]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
An example of using the RMRFM69 library. The program takes the message using the RFM69 module.
Based on the sample program from HopeRF: 
 * file       rfm69_rx.ino
 * hardware   HopeRF's RFduino TRx
 * software   get message via rfm69, & send message to uart
 * note       can talk to HopeRF's EVB or DK demo           
 *
 * version    1.0
 * date       Jun 10 2014
 * author     QY Ruan

tested on ESP32 
SCLK = 18, MISO = 19, MOSI = 23, SS = 5
RST = 17, DIO0 = 16
*/

#include <Arduino.h>
#include <SPI.h>
#include <RMRFM69.h>

SPIClass rfspi = SPIClass(VSPI);

RMRFM69 radio(rfspi, 5, 16, 17);

byte getstr[21];

void setup()
{
  Serial.begin(115200);
 // vspi = SPIClass(VSPI);

  radio.Modulation     = FSK;
  radio.COB            = RFM69;
  radio.Frequency      = 868000;
  radio.OutputPower    = 10+18;          //10dBm OutputPower
  radio.PreambleLength = 16;             //16Byte preamble
  radio.FixedPktLength = true;           //packet length didn't in message which need to be send
  radio.PayloadLength  = 21;
  radio.CrcDisable     = false;          //CRC On
  radio.AesOn          = false;

  radio.SymbolTime     = 416000;         //2.4Kbps
  radio.Devation       = 35;             //35KHz for devation
  radio.BandWidth      = 100;            //100KHz for bandwidth
  radio.SyncLength     = 3;              //
  radio.SyncWord[0]    = 0xAA;
  radio.SyncWord[1]    = 0x2D;
  radio.SyncWord[2]    = 0xD4;
  radio.vInitialize();
  radio.vGoRx();
  Serial.println("Start RX...");
}

void loop()
{
if(radio.bGetMessage(getstr)!=0)
    {
    
    Serial.printf("%s\n\r",getstr);

    }  	
}
