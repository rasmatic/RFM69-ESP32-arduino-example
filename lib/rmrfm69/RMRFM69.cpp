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

#include "RMRFM69.h"

/********************************************************** 
**RF69 Regsister define                                      
**********************************************************/ 
#define 	RegFifo 			0x00 
#define 	RegOpMode 			0x01 
#define 	RegDataModul 		0x02 
#define 	RegBitrateMsb 		0x03 
#define 	RegBitrateLsb 		0x04 
#define 	RegFdevMsb 		    0x05 
#define 	RegFdevLsb 		    0x06 
#define 	RegFrMsb 			0x07 
#define 	RegFrMid 			0x08 
#define 	RegFrLsb 			0x09 
#define 	RegOsc1 			0x0A 
#define 	RegAfcCtrl	 		0x0B 
#define 	RegListen1 		    0x0D 
#define 	RegListen2 		    0x0E 
#define 	RegListen3 		    0x0F 
#define 	RegVersion 		    0x10 
#define 	RegPaLevel 		    0x11 
#define 	RegPaRamp 			0x12 
#define 	RegOcp 			    0x13 
#define 	RegLna 			    0x18 
#define 	RegRxBw 			0x19 
#define 	RegAfcBw 			0x1A 
#define 	RegOokPeak 		    0x1B 
#define 	RegOokAvg 			0x1C 
#define 	RegOokFix 			0x1D 
#define 	RegAfcFei 			0x1E 
#define 	RegAfcMsb 			0x1F 
#define 	RegAfcLsb 			0x20 
#define 	RegFeiMsb 			0x21 
#define 	RegFeiLsb 			0x22 
#define 	RegRssiConfig 		0x23 
#define 	RegRssiValue 		0x24 
#define 	RegDioMapping1 	    0x25 
#define 	RegDioMapping2 	    0x26 
#define 	RegIrqFlags1 		0x27 
#define 	RegIrqFlags2 		0x28 
#define 	RegRssiThresh 		0x29 
#define 	RegRxTimeout1 		0x2A 
#define 	RegRxTimeout2 		0x2B 
#define 	RegPreambleMsb 	    0x2C 
#define 	RegPreambleLsb 	    0x2D 
#define 	RegSyncConfig 		0x2E 
#define 	RegSyncValue1		0x2F 
#define 	RegSyncValue2       0x30 
#define 	RegSyncValue3       0x31 
#define 	RegSyncValue4       0x32 
#define 	RegSyncValue5       0x33 
#define 	RegSyncValue6       0x34 
#define 	RegSyncValue7       0x35 
#define 	RegSyncValue8       0x36 
#define 	RegPacketConfig1 	0x37 
#define 	RegPayloadLength 	0x38 
#define 	RegNodeAdrs 		0x39 
#define 	RegBroadcastAdrs 	0x3A 
#define 	RegAutoModes 		0x3B 
#define 	RegFifoThresh 		0x3C 
#define 	RegPacketConfig2 	0x3D 
#define 	RegAesKey1		    0x3E 
#define 	RegAesKey2          0x3F 
#define 	RegAesKey3          0x40 
#define 	RegAesKey4          0x41 
#define 	RegAesKey5          0x42 
#define 	RegAesKey6          0x43 
#define 	RegAesKey7          0x44 
#define 	RegAesKey8          0x45 
#define 	RegAesKey9          0x46 
#define 	RegAesKey10         0x47 
#define 	RegAesKey11         0x48 
#define 	RegAesKey12         0x49 
#define 	RegAesKey13         0x4A 
#define 	RegAesKey14         0x4B 
#define 	RegAesKey15         0x4C 
#define 	RegAesKey16         0x4D 
#define 	RegTemp1 			0x4E 
#define 	RegTemp2 			0x4F 
#define 	RegTestLna 		    0x58 
#define 	RegTestPa1 		    0x5A 
#define 	RegTestPa2 		    0x5C 
#define 	RegTestDagc 		0x6F 
#define 	RegTestAfc 		    0x71 

/**********************************************************      
**RF69 mode status                                          
**********************************************************/      
#define		RADIO_SLEEP			(0x00<<2)
#define		RADIO_STANDBY		(0x01<<2)
#define		RADIO_TX			(0x03<<2)
#define		RADIO_RX			(0x04<<2)

#define 	MODE_MASK			0xE3

#define		PacketMode			(0<<5)
#define		ConWithClkMode		(2<<5)
#define		ConWithoutClkMode	(3<<5)

#define		FskMode				(0<<3)
#define		OokMode				(1<<3)

#define		Shaping				2

//for PacketConfig
#define		VariablePacket		(1<<7)
#define		DcFree_NRZ			(0<<5)
#define		DcFree_MANCHESTER	(1<<5)
#define		DcFree_WHITENING	(2<<5)
#define		CrcOn				(1<<4)
#define		CrcDisAutoClear		(1<<3)
#define		AddrFilter_NONE		(0<<1)
#define		AddrFilter_NODE		(1<<1)
#define		AddrFilter_ALL		(2<<1)
#define		CrcCalc_CCITT		0x00
#define		CrcCalc_IBM			0x01

//Constructor
RMRFM69::RMRFM69(SPIClass &spiPort, byte csPin, byte dio0Pin, byte rstPin)
{
	_spiPort = &spiPort;
	_csPin = csPin;
	_dio0Pin = dio0Pin;
	_rstPin = rstPin;

}

/**********************************************************
**Name:     vInitialize
**Function: initialize rfm69 or rfm69c
**Input:    none
**Output:   none
**********************************************************/
void RMRFM69::vInitialize(void)
{
	pinMode (_csPin, OUTPUT);
	pinMode (_rstPin, OUTPUT);
	pinMode (_dio0Pin, INPUT_PULLDOWN);

	digitalWrite(_csPin, HIGH);
	digitalWrite(_rstPin, LOW);
	
	vSpiInit();
	
	//�˿ڳ�ʼ�� for 32MHz
	FrequencyValue.Freq = (Frequency << 11) / 125; //Calc. Freq
	BitRateValue = (SymbolTime << 5) / 1000;	   //Calc. BitRate
	DevationValue = (Devation << 11) / 125;		   //Calc. Fdev
	BandWidthValue = bSelectBandwidth(BandWidth);

	vConfig();
	vGoStandby();
}

/**********************************************************
**Name:     vConfig
**Function: config RF69 
**Input:    none
**Output:   none
**********************************************************/
void RMRFM69::vConfig(void)
{
 byte i;
 byte sync;
 vRF69Reset();
 vGoStandby();

 //Frequency
 vSpiWrite(((word)RegFrMsb<<8)+FrequencyValue.freq.FreqH);
 vSpiWrite(((word)RegFrMid<<8)+FrequencyValue.freq.FreqM);
 vSpiWrite(((word)RegFrLsb<<8)+FrequencyValue.freq.FreqL);
 
 // Mode
 switch(Modulation)
 	{
 	case OOK:
 		vSpiWrite(((word)RegDataModul<<8)+PacketMode+OokMode+Shaping);
 		break;
 	case GFSK:
 		vSpiWrite(((word)RegDataModul<<8)+PacketMode+FskMode+Shaping);
 		break;
 	case FSK:
 	default:
 		vSpiWrite(((word)RegDataModul<<8)+PacketMode+FskMode);
 		break;
 	}

 //BitRate 
 vSpiWrite(((word)RegBitrateMsb<<8)+(byte)(BitRateValue>>8));
 vSpiWrite(((word)RegBitrateLsb<<8)+(byte)BitRateValue);
 
 //Devation
 vSpiWrite(((word)RegFdevMsb<<8)+(((byte)(DevationValue>>8))&0x3F));
 vSpiWrite(((word)RegFdevLsb<<8)+(byte)(DevationValue&0xFF));
 
 //AFC
 vSpiWrite(((word)RegAfcCtrl<<8)+0x00);		//Standard
 
 //PA
 switch(COB)
 	{
 	case RFM69H:
 	case RFM69HC:
 		vSpiWrite(((word)RegPaLevel<<8)+0x60+(OutputPower&0x1F));	
 		break;
 	default:		
 		vSpiWrite(((word)RegPaLevel<<8)+0x80+(OutputPower&0x1F));
 		break;
 	}
 vSpiWrite(((word)RegPaRamp<<8)+bSelectRamping(SymbolTime));			
 
 //Ocp
 vSpiWrite(((word)RegOcp<<8)+0x0F);			//Disable Ocp
 //LNA
 vSpiWrite(((word)RegLna<<8)+0x88);			//High & LNA Enable
 //RxBw
 vSpiWrite(((word)RegRxBw<<8)+0x20+BandWidthValue);	//CutOff for 8%
 //AfcBw
 vSpiWrite(((word)RegAfcBw<<8)+0x20+BandWidthValue);
 //OOK
 vSpiWrite(((word)RegOokPeak<<8)+0x40+(0x07<<3)+0x00);
 
 //AFC
 vSpiWrite(((word)RegAfcFei<<8)+0x00);		//Disable
 
 vSpiWrite(((word)RegPreambleMsb<<8)+(byte)(PreambleLength>>8));
 vSpiWrite(((word)RegPreambleLsb<<8)+(byte)PreambleLength);
 if(SyncLength==0)
 	sync = 0;
 else
 	sync = SyncLength-1;
 vSpiWrite(((word)RegSyncConfig<<8)+0x80+((sync&0x07)<<3));	//SyncOn Nbyte 
 for(i=0;i<8;i++)								//SyncWordSetting
 	vSpiWrite(((word)(RegSyncValue1+i)<<8)+SyncWord[i]);	

 vSpiWrite(((word)RegDioMapping2<<8)+0xF7);	//DIO4 PllLock / DIO5 ModeRdy / CLK Off  
 
 i = DcFree_NRZ + AddrFilter_NONE;
 if(!FixedPktLength)
 	i += VariablePacket;
 if(!CrcDisable)
 	{
 	i += CrcOn;
	if(CrcMode)
		i += CrcCalc_IBM;
	else
		i += CrcCalc_CCITT;
	}
 vSpiWrite(((word)RegPacketConfig1<<8)+i);		
 if(AesOn)
 	vSpiWrite(((word)RegPacketConfig2<<8)+0x01);//
 else
 	vSpiWrite(((word)RegPacketConfig2<<8)+0);	//
 
 if(FixedPktLength)								//Set Packet length
 	vSpiWrite(((word)RegPayloadLength<<8)+PayloadLength);
 else	
 	vSpiWrite(((word)RegPayloadLength<<8)+0x40);

 vSpiWrite(((word)RegFifoThresh<<8)+0x01);
}

/**********************************************************
**Name:     vGoRx
**Function: set rf69 to receive mode
**Input:    none
**Output:   none
**********************************************************/
void RMRFM69::vGoRx(void)
{
 byte tmp;
 
 vSpiWrite(((word)RegTestPa1<<8)+0x55);			//for NormalMode or RxMode
 vSpiWrite(((word)RegTestPa2<<8)+0x70);

 if(CrcDisable)	
 	vSpiWrite(((word)RegDioMapping1<<8)+0x44);	//DIO0 PayloadReady / DIO1 FiflLevel / DIO2 Data /DIO3 FifoFull
 else
 	vSpiWrite(((word)RegDioMapping1<<8)+0x04);	//DIO0 CrcOk  / DIO1 FiflLevel / DIO2 Data /DIO3 FifoFull
 
 tmp = bSpiRead(RegOpMode);
 tmp&= MODE_MASK;
 tmp |= RADIO_RX;
 vSpiWrite(((word)RegOpMode<<8)+tmp);
}

/**********************************************************
**Name:     vGoStandby
**Function: set rf69 to standby mode
**Input:    none
**Output:   none
**********************************************************/
void RMRFM69::vGoStandby(void)
{
 byte tmp;
 tmp = bSpiRead(RegOpMode);
 tmp&= MODE_MASK;
 tmp |= RADIO_STANDBY;
 vSpiWrite(((word)RegOpMode<<8)+tmp);
}	

/**********************************************************
**Name:     vGoSleep
**Function: set rf69 to sleep mode
**Input:    none
**Output:   none
**********************************************************/
void RMRFM69::vGoSleep(void)
{
 byte tmp;
 tmp = bSpiRead(RegOpMode);
 tmp&= MODE_MASK;
 tmp |= RADIO_SLEEP;
 vSpiWrite(((word)RegOpMode<<8)+tmp);
}	

/**********************************************************
**Name:     bSendMessage
**Function: set rf69 to sleep mode
**Input:    msg------for which message to send
            length---message length
**Output:   true-----send ok
            false----send error/over time
**********************************************************/
bool RMRFM69::bSendMessage(byte msg[], byte length)
{
 byte tmp;
 uint32_t overtime;
 word bittime;

 switch(COB)
	{
	case RFM65:									//only for Rx
	case RFM65C:
		return(false);
	case RFM69H:
	case RFM69HC:
 		vSpiWrite(((word)RegTestPa1<<8)+0x5D);		//for HighPower
	 	vSpiWrite(((word)RegTestPa2<<8)+0x7C);
		break;
	default:
	case RFM69:
	case RFM69C:
	 	vSpiWrite(((word)RegTestPa1<<8)+0x55);		//for NormalMode or RxMode
 		vSpiWrite(((word)RegTestPa2<<8)+0x70);
		break;
	}
	
 vSpiWrite(((word)RegDioMapping1<<8)+0x04);	//DIO0 PacketSend  / DIO1 FiflLevel / DIO2 Data /DIO3 FifoFull
 
 if(!FixedPktLength)
 	vSpiWrite(((word)RegFifo<<8)+length);
 	vSpiBurstWrite(RegFifo, msg, length);
 
 tmp = bSpiRead(RegOpMode);
 tmp&= MODE_MASK;
 tmp |= RADIO_TX;
 vSpiWrite(((word)RegOpMode<<8)+tmp);
  
 //�ȴ��������
 bittime  = SymbolTime/1000;		//unit: us
 overtime = SyncLength+PreambleLength+length;
 if(!FixedPktLength)				//SyncWord & PktLength & 2ByteCRC
    overtime += 1;
 if(!CrcDisable)
 	overtime += 2;
 overtime<<=3;					//8bit == 1byte
 overtime*= bittime;
 overtime/= 1000;				//unit: ms
 if(overtime==0) 
 	overtime = 1;
 overtime += (overtime>>3);		//add 12.5% for ensure
 delay(overtime);			//
 for(tmp=0;tmp<100;tmp++)		//about 50ms for overtime
 	{
 	if(digitalRead(_dio0Pin))
 		break; 	
 	delayMicroseconds(500);
 	}
 vGoStandby();	
 if(tmp>=100)
 	return(false);
 else
 	return(true);
}

/**********************************************************
**Name:     bGetMessage
**Function: check receive packet
**Input:    msg------for which message to read
**Output:   packet length
			0--------have no packet
**********************************************************/
byte RMRFM69::bGetMessage(byte msg[])
{
 byte length;
 if (digitalRead(_dio0Pin)) //�յ�CrcOk or PayloadReady
 {
	 if (FixedPktLength)
		 length = PayloadLength;
	 else
		 length = bSpiRead(RegFifo);
	 vSpiBurstRead(RegFifo, msg, length);
	 vGoStandby();
	 vGoRx();
	 return (length);
 	}
 return(0);
}

/**********************************************************
**Name:     vRF69Reset
**Function: hardware reset rf69 chipset
**Input:    none
**Output:   none
**********************************************************/
void RMRFM69::vRF69Reset(void)
{
	digitalWrite(_rstPin, HIGH);
 	delayMicroseconds(300);					//at least 100us for reset
	digitalWrite(_rstPin, LOW);
	delay(10);						//wait for ready
}	

/**********************************************************
**Name:     bSelectBandwidth
**Function: 
**Input:    BandWidth
**Output:   BandWidthValue
**********************************************************/
byte RMRFM69::bSelectBandwidth(byte rx_bw)
{
 if(rx_bw<3)	
 	return 0x0F;						//2.6KHz
 else if(rx_bw<4)
 	return 0x07;						//3.91KHz
 else if(rx_bw<6)
 	return 0x16;						//5.21KHz 
 else if(rx_bw<7)
 	return 0x0E;						//6.25KHz
 else if(rx_bw<8)
 	return 0x06;						//7.81KHz
 else if(rx_bw<11)
	return 0x15;						//10.4KHz 	Min
 else if(rx_bw<13)
 	return 0x0D;						//12.5KHz	
 else if(rx_bw<16)	
 	return 0x05;						//15.6KHz
 else if(rx_bw<21)		
 	return 0x14;						//20.8KHz
 else if(rx_bw<=25)	
 	return 0x0C;						//25.0KHz
 else if(rx_bw<32)
 	return 0x04;						//31.3KHz
 else if(rx_bw<42)
 	return 0x13;						//41.7KHz
 else if(rx_bw<=50)
 	return 0x0B;						//50.0KHz
 else if(rx_bw<63)
 	return 0x03;						//62.5KHz
 else if(rx_bw<84)
 	return 0x12;						//83.3KHz
 else if(rx_bw<=100)
 	return 0x0A;						//100KHz
 else if(rx_bw<=125)
 	return 0x02;						//125KHz
 else if(rx_bw<167)
 	return 0x11;						//167KHz
 else if(rx_bw<=200)
 	return 0x09;						//200KHz
 else if(rx_bw<=250)
 	return 0x01;						//250KHz
 else if(rx_bw<=333)
 	return 0x10;						//333KHz
 else if(rx_bw<=400)
 	return 0x08;						//400KHz
 else
 	return 0x00;						//500KHz Max

}	

/**********************************************************
**Name:     bSelectRamping
**Function: 
**Input:    symbol time
**Output:   ramping value
**********************************************************/
byte RMRFM69::bSelectRamping(uint32_t symbol)
{
 uint32_t SymbolRate;
 
 SymbolRate = symbol/1000;			//ns->us
 SymbolRate = SymbolRate/4;			// 1/4 ramping
 
 if(SymbolRate<=10)		
 	return 0x0F;					//10us
 else if(SymbolRate<=12)			
 	return 0x0E;					//12us
 else if(SymbolRate<=15)			
 	return 0x0D;					//15us
 else if(SymbolRate<=20)
 	return 0x0C;					//20us
 else if(SymbolRate<=25)
 	return 0x0B;					//25us
 else if(SymbolRate<=31)
 	return 0x0A;					//31us
 else if(SymbolRate<=40)
 	return 0x09;					//40us
 else if(SymbolRate<=50)
 	return 0x08;					//50us
 else if(SymbolRate<=62)
 	return 0x07;					//62us
 else if(SymbolRate<=100)
 	return 0x06;					//100us
 else if(SymbolRate<=125)
 	return 0x05;					//125us
 else if(SymbolRate<=250)
 	return 0x04;					//250us
 else if(SymbolRate<=500)
 	return 0x03;					//500us
 else if(SymbolRate<=1000)
	return 0x02;					//1000us
 else if(SymbolRate<=2000)
 	return 0x01;					//2000us
 else 
 	return 0x00;
}	
	
/**********************************************************
**Name:     vRF69SetAesKey
**Function: Set AES128 Key block
**Input:    AesKey
**Output:   none
**********************************************************/
void RMRFM69::vRF69SetAesKey(void)
{
 byte i;
 for(i=0;i<16;i++)
 	vSpiWrite(((word)(RegAesKey1+i)<<8)+AesKey[i]);
}

/**********************************************************
**Name:     vTrigAfc
**Function: Manual Trigger AFC
**Input:    AesKey
**Output:   none
**********************************************************/
void RMRFM69::vTrigAfc(void)
{
 byte tmp, i;
 tmp = bSpiRead(RegAfcFei);	
 
 if(tmp&0x10)			//AFC has finished
 	{
 	vSpiWrite(((word)RegAfcFei<<8)+tmp+0x01);	//TrigAfc	
 	
 	for(i=0; i<100; i++)		//
 		{
 		delayMicroseconds(100);
 		tmp = bSpiRead(RegAfcFei);	
 		if(tmp&0x10)	//AFC has finished
 			break;
 		}
 	}
}	


/**********************************************************
**Name:     vDirectRx
**Function: Set RF69 to continuous rx mode(with init.)
**Input:    none
**Output:   none
**********************************************************/
void RMRFM69::vDirectRx(void)
{
	digitalWrite(_rstPin, LOW);
 	vSpiInit();		
 
 //�˿ڳ�ʼ�� for 32MHz
 FrequencyValue.Freq = (Frequency<<11)/125;		//Calc. Freq
 BitRateValue  = (SymbolTime<<5)/1000;			//Calc. BitRate
 DevationValue = (Devation<<11)/125;			//Calc. Fdev 
 BandWidthValue = bSelectBandwidth(BandWidth);
 
 vRF69Reset();
 vGoStandby();

 //Frequency
 vSpiWrite(((word)RegFrMsb<<8)+FrequencyValue.freq.FreqH);
 vSpiWrite(((word)RegFrMid<<8)+FrequencyValue.freq.FreqM);
 vSpiWrite(((word)RegFrLsb<<8)+FrequencyValue.freq.FreqL);
 
 // Mode
 switch(Modulation)
 	{
 	case OOK:
 		vSpiWrite(((word)RegDataModul<<8)+ConWithClkMode+OokMode+Shaping);
 		break;
 	case GFSK:
 		vSpiWrite(((word)RegDataModul<<8)+ConWithClkMode+FskMode+Shaping);
 		break;
 	case FSK:
 	default:
 		vSpiWrite(((word)RegDataModul<<8)+ConWithClkMode+FskMode);
 		break;
 	}

 //BitRate 
 vSpiWrite(((word)RegBitrateMsb<<8)+(byte)(BitRateValue>>8));
 vSpiWrite(((word)RegBitrateLsb<<8)+(byte)BitRateValue);
 
 //Devation
 vSpiWrite(((word)RegFdevMsb<<8)+(((byte)(DevationValue>>8))&0x3F));
 vSpiWrite(((word)RegFdevLsb<<8)+(byte)(DevationValue&0xFF));
 
 //AFC
 vSpiWrite(((word)RegAfcCtrl<<8)+0x00);		//Standard
 
 //PA
 switch(COB)
 	{
 	case RFM69H:
 	case RFM69HC:
 		vSpiWrite(((word)RegPaLevel<<8)+0x60+(OutputPower&0x1F));	
 		break;
 	default:		
 		vSpiWrite(((word)RegPaLevel<<8)+0x80+(OutputPower&0x1F));
 		break;
 	}
 vSpiWrite(((word)RegPaRamp<<8)+bSelectRamping(SymbolTime));			
 
 //Ocp
 vSpiWrite(((word)RegOcp<<8)+0x0F);			//Disable Ocp
 //LNA
 vSpiWrite(((word)RegLna<<8)+0x88);			//High & LNA Enable
 //RxBw
 vSpiWrite(((word)RegRxBw<<8)+0x20+BandWidthValue);	//CutOff for 8%
 //AfcBw
 vSpiWrite(((word)RegAfcBw<<8)+0x20+BandWidthValue);
 //OOK
 vSpiWrite(((word)RegOokPeak<<8)+0x40+(0x07<<3)+0x00);
 
 //AFC
 vSpiWrite(((word)RegAfcFei<<8)+0x00);		//Disable
 
 vSpiWrite(((word)RegPreambleMsb<<8)+(byte)(PreambleLength>>8));
 vSpiWrite(((word)RegPreambleLsb<<8)+(byte)PreambleLength);

 vSpiWrite(((word)RegSyncConfig<<8)+0x80+0x00);	//SyncOff Nbyte 
 vSpiWrite(((word)RegDioMapping2<<8)+0xF7);		//DIO4 PllLock / DIO5 ModeRdy / CLK Off  
 
 vSpiWrite(((word)RegPacketConfig1<<8)+0x00);		
 vSpiWrite(((word)RegPacketConfig2<<8)+0);	
 
 vGoStandby();
}

/**********************************************************
**Name:     vChangeFreq
**Function: Change Frequency
**Input:    none
**Output:   none
**********************************************************/
void RMRFM69::vChangeFreq(uint32_t freq)	
{
 FreqStruct ChangeFreq;
 byte tmp;
 
 vGoStandby();
 ChangeFreq.Freq = (freq<<11)/125;		//Calc. Freq
 
 //Frequency
 vSpiWrite(((word)RegFrMsb<<8)+ChangeFreq.freq.FreqH);
 vSpiWrite(((word)RegFrMid<<8)+ChangeFreq.freq.FreqM);
 vSpiWrite(((word)RegFrLsb<<8)+ChangeFreq.freq.FreqL); 

 vGoRx();
 do{
 	tmp = bSpiRead(RegIrqFlags1);
 	}
	while((tmp&0x40)!=0x40);
 
}

/**********************************************************
**Name:     vReadRssi
**Function: 
**Input:    none
**Output:   none
**********************************************************/
byte RMRFM69::bReadRssi(void)
{
 byte tmp;	
 vSpiWrite(((word)RegRssiConfig<<8)+0x01);		//TrigRssiADC
 
 do{
 	tmp = bSpiRead(RegRssiConfig);	
 	delayMicroseconds(10);
 	}while((tmp&0x02)!=0x02);
 
 return(bSpiRead(RegRssiValue));
}
/**********************************************************
**Name:     bSpiRead
**Function: Read RFM69 register
**Input:    register
**Output:   the contents of the register
**********************************************************/
byte RMRFM69::bSpiRead(byte reg)
{
	byte rBuffer;

	digitalWrite (_csPin, LOW);
	_spiPort->transfer(reg);
	rBuffer = _spiPort->transfer(0x00);
	digitalWrite(_csPin, HIGH);
	return rBuffer;
}
/**********************************************************
**Name:     vSpiWrite
**Function: write value to RFM69 register
**Input:    register + value (word)
**Output:   
**********************************************************/
void RMRFM69::vSpiWrite(word data)
{
	uint8_t reg, wValue;

	reg = (uint8_t)(data >> 8) | 0x80;
	wValue = (uint8_t)data;

	digitalWrite(_csPin, LOW);
	_spiPort->transfer(reg);
	_spiPort->transfer(wValue);
	digitalWrite(_csPin, HIGH);
}
/**********************************************************
**Name:     vSpiBurstRead
**Function: read multiple values
**Input:    register, buffer, buffer length
**Output:   none
**********************************************************/
void RMRFM69::vSpiBurstRead(byte reg, byte *rbuffer, int rlen)
{
	digitalWrite(_csPin, LOW);
	_spiPort->transfer(reg);
	_spiPort->transfer(rbuffer, rlen);
	digitalWrite(_csPin, HIGH);
}
/**********************************************************
**Name:     vSpiBurstWrite
**Function: write multiple values
**Input:    register, buffer, buffer length
**Output:   none
**********************************************************/
void RMRFM69::vSpiBurstWrite(byte reg, byte *wbuffer, int wlen)
{
	reg = reg | 0x80;

	digitalWrite(_csPin, LOW);
	_spiPort->transfer(reg);
	_spiPort->transfer(wbuffer, wlen);
	digitalWrite(_csPin, HIGH);
}
/**********************************************************
**Name:     vSpiInit
**Function: init SPI
**Input:    none
**Output:   none
**********************************************************/
void RMRFM69::vSpiInit()
{
	digitalWrite(_csPin, HIGH);
	_spiPort->setFrequency(1000000);
	_spiPort->setBitOrder(MSBFIRST);
	_spiPort->setDataMode(SPI_MODE0);
	_spiPort->begin();

}