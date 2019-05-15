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

This library is a port of HopeRF's library:
https://www.hoperf.com/data/upload/back/20181122/HoepRF_HSP_V1.3.rar
 
*/

#ifndef RMRFM69_h
	#define RMRFM69_h
	
	#include <Arduino.h>
	#include <spi.h>
	
	typedef union  
	{
	 struct
	 	{
	  	byte FreqL: 8;
	  	byte FreqM: 8;
	  	byte FreqH: 8;
	  	byte FreqX: 8;
	 	}freq;
	 unsigned long Freq;
	}FreqStruct;

	enum modulationType {OOK, FSK, GFSK};
	
	enum moduleType {RFM65, RFM65C, RFM69, RFM69C, RFM69H, RFM69HC};
	
	class RMRFM69
	{
	 public:
		 RMRFM69(SPIClass &spiPort, byte csPin, byte dio0Pin, byte rstPin);
		 modulationType Modulation; //OOK/FSK/GFSK
		 moduleType COB;				//Chip on board
		 uint32_t Frequency;			//unit: KHz
		 uint32_t SymbolTime;			//unit: ns
		 uint32_t Devation;				//unit: KHz
		 word BandWidth;				//unit: KHz
		 byte OutputPower;				//unit: dBm   range: 0-31 [-18dBm~+13dBm] for RFM69/RFM69C
										//            range: 0-31 [-11dBm~+20dBm] for RFM69H/RFM69HC
		 word PreambleLength;			//unit: byte

		 bool CrcDisable; //fasle: CRC enable�� & use CCITT 16bit
						  //true : CRC disable
		 bool CrcMode;	//false: CCITT

		 bool FixedPktLength; //false: for contain packet length in Tx message, the same mean with variable lenth
							  //true : for doesn't include packet length in Tx message, the same mean with fixed length
		 bool AesOn;		  //false:
							  //true:
		 bool AfcOn;		  //false:
							  //true:
		 byte SyncLength;	 //unit: none, range: 1-8[Byte], value '0' is not allowed!
		 byte SyncWord[8];
		 byte PayloadLength; //PayloadLength is need to be set a value, when FixedPktLength is true.
		 byte AesKey[16];	//AES Key block, note [0]->[15] == MSB->LSB

		 void vInitialize(void);
		 void vConfig(void);
		 void vGoRx(void);
		 void vGoStandby(void);
		 void vGoSleep(void);
		 bool bSendMessage(byte msg[], byte length);
		 byte bGetMessage(byte msg[]);
		 void vRF69SetAesKey(void);
		 void vTrigAfc(void);

		 void vDirectRx(void);			  //go continuous rx mode, with init. inside
		 void vChangeFreq(uint32_t freq); //change frequency
		 byte bReadRssi(void);			  //read rssi value
				
	 private:
		 SPIClass *_spiPort = NULL;
		 byte _rstPin;
		 byte _dio0Pin;
		 byte _csPin;
		 FreqStruct FrequencyValue;
		 word BitRateValue;
		 word DevationValue;
		 byte BandWidthValue;

		 void vRF69Reset(void);
		 byte bSelectBandwidth(byte rx_bw);
		 byte bSelectRamping(uint32_t symbol);
		 void vSpiWrite(word data);
		 byte bSpiRead(byte reg);
		 void vSpiBurstRead(byte reg, byte * rbuffer, int rlen);
		 void vSpiBurstWrite(byte reg, byte * wbuffer, int wlen);
		 void vSpiInit();
	};
#else
	#warning "RMRFM69.h have been defined!"

#endif