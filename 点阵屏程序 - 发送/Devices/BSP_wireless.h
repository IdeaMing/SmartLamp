/**
  ******************************************************************************
  * @file    BSP_wireless.h
  * @author  Liming
  * @version V1.0.0
  * @date    11-November-2017
  * @brief   This file contains all the functions prototypes for the
  *          bsp_wirless.c driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2017 IdeaMing</center></h2>
  *
  * @Licensed under 
  * @Revision 
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CC2500_H
#define __CC2500_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Module---------------------------------------------------------------------*/	 
#ifdef	_WIRELESS_MODULE_
#define	WIRELESS_EXT
#else
#define	WIRELESS_EXT	extern
#endif
	 
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h" 
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/   
//Config Register
#define IOCFG2       0x00        // GDO2 output pin configuration
#define IOCFG1       0x01        // GDO1 output pin configuration
#define IOCFG0       0x02        // GDO0 output pin configuration
#define FIFOTHR      0x03        // RX FIFO and TX FIFO thresholds
#define SYNC1        0x04        // Sync word, high byte
#define SYNC0        0x05        // Sync word, low byte
#define PKTLEN       0x06        // Packet length
#define PKTCTRL1     0x07        // Packet automation control
#define PKTCTRL0     0x08        // Packet automation control
#define ADDR         0x09        // Device address
#define CHANNR       0x0A        // Channel number
#define FSCTRL1      0x0B        // Frequency synthesizer control
#define FSCTRL0      0x0C        // Frequency synthesizer control
#define FREQ2        0x0D        // Frequency control word, high byte
#define FREQ1        0x0E        // Frequency control word, middle byte
#define FREQ0        0x0F        // Frequency control word, low byte
#define MDMCFG4      0x10        // Modem configuration
#define MDMCFG3      0x11        // Modem configuration
#define MDMCFG2      0x12        // Modem configuration
#define MDMCFG1      0x13        // Modem configuration
#define MDMCFG0      0x14        // Modem configuration
#define DEVIATN      0x15        // Modem deviation setting
#define MCSM2        0x16        // Main Radio Control State Machine configuration
#define MCSM1        0x17        // Main Radio Control State Machine configuration
#define MCSM0        0x18        // Main Radio Control State Machine configuration
#define FOCCFG       0x19        // Frequency Offset Compensation configuration
#define BSCFG        0x1A        // Bit Synchronization configuration
#define AGCCTRL2     0x1B        // AGC control
#define AGCCTRL1     0x1C        // AGC control
#define AGCCTRL0     0x1D        // AGC control
#define WOREVT1      0x1E        // High byte Event 0 timeout
#define WOREVT0      0x1F        // Low byte Event 0 timeout
#define WORCTRL      0x20        // Wake On Radio control
#define FREND1       0x21        // Front end RX configuration
#define FREND0       0x22        // Front end TX configuration
#define FSCAL3       0x23        // Frequency synthesizer calibration
#define FSCAL2       0x24        // Frequency synthesizer calibration
#define FSCAL1       0x25        // Frequency synthesizer calibration
#define FSCAL0       0x26        // Frequency synthesizer calibration
#define RCCTRL1      0x27        // RC oscillator configuration
#define RCCTRL0      0x28        // RC oscillator configuration
#define FSTEST       0x29        // Frequency synthesizer calibration control
#define PTEST        0x2A        // Production test
#define AGCTEST      0x2B        // AGC test
#define TEST2        0x2C        // Various test settings
#define TEST1        0x2D        // Various test settings
#define TEST0        0x2E        // Various test settings

//Command
#define SRES         0x30        // Reset chip.
#define SFSTXON      0x31        // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
                                 // If in RX/TX: Go to a wait state where only the synthesizer is
                                 // running (for quick RX / TX turnaround).
#define SXOFF        0x32        // Turn off crystal oscillator.
#define SCAL         0x33        // Calibrate frequency synthesizer and turn it off
                                 // (enables quick start).
#define SRX          0x34        // Enable RX. Perform calibration first if coming from IDLE and
                                 // MCSM0.FS_AUTOCAL=1.
#define STX          0x35        // In IDLE state: Enable TX. Perform calibration first if
                                 // MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
                                 // Only go to TX if channel is clear.
#define SIDLE        0x36        // Exit RX / TX, turn off frequency synthesizer and exit
                                 // Wake-On-Radio mode if applicable.
#define SAFC         0x37        // Perform AFC adjustment of the frequency synthesizer
#define SWOR         0x38        // Start automatic RX polling sequence (Wake-on-Radio)
#define SPWD         0x39        // Enter power down mode when CSn goes high.
#define SFRX         0x3A        // Flush the RX FIFO buffer.
#define SFTX         0x3B        // Flush the TX FIFO buffer.
#define SWORRST      0x3C        // Reset real time clock.
#define SNOP         0x3D        // No operation. May be used to pad strobe commands to two
                                 // bytes for simpler software.

//Status Register
#define PARTNUM         0x30
#define VERSION         0x31
#define FREQEST         0x32
#define LQI             0x33
#define RSSI            0x34
#define MARCSTATE       0x35
#define WORTIME1        0x36
#define WORTIME0        0x37
#define PKTSTATUS       0x38
#define VCO_VC_DAC      0x39
#define TXBYTES         0x3A
#define RXBYTES         0x3B
#define RCCTRL1_STATUS	0x3C
#define RCCTRL0_STATUS	0x3D
#define PATABLE         0x3E
#define FIFO            0x3F


#define RSSIOFFSET      0x48
#define CHANNELCOUNT    0x20
#define PROHEAD           0x5353
#define SENDPACKETLEN     0x0040
#define RECVPACKETLEN     0x0020
/* Exported macro ------------------------------------------------------------*/


enum PRO_PACKET_VALUE
{
  PACKET_VALUE_IDLE=0x00,PACKET_VALUE_SIGNIN=0x01,PACKET_VALUE_VOTE=0x02,PACKET_VALUE_CHOICES=0x03,PACKET_VALUE_ELECTION=0x04,
  PACKET_VALUE_NUMBER=0x05,PACKET_VALUE_RUSHANSWER=0x06,PACKET_VALUE_ELECTIONNAMELIST=0x07,PACKET_VALUE_NUMBERNAMELIST=0x08,
  PACKET_VALUE_FREEKEY=0x09,PACKET_VALUE_JUDGEMENT=0x0A,PACKET_VALUE_ELECTIONMULTILIST=0x0B,PACKET_VALUE_BALLOT=0x0C,
  PACKET_VALUE_LOGIN=0x14,PACKET_VALUE_RUNPROPOSAL=0x15,PACKET_VALUE_POWERDOWN=0x20,PACKET_VALUE_SETKEYID=0x21,
  PACKET_VALUE_SETKEYPARAM=0x22,PACKET_VALUE_READVERSION=0x23,PACKET_VALUE_READPOWERSIGNAL=0x24,PACKET_VALUE_SETBASEMENTIP=0x25,
  PACKET_VALUE_SETBASEMENTMATCH=0x26,PACKET_VALUE_REPLACEKEYID=0x27,PACKET_VALUE_DISPLAYRESULT=0x30,PACKET_VALUE_DOWNLOADNAMES=0x40,
  PACKET_VALUE_DOWNLOADFONTS=0x41,PACKET_VALUE_DOWNLOADPROPOSAL=0x42,PACKET_VALUE_GETERRORCODE=0xE0,PACKET_VALUE_TEST=0xE1,
  PACKET_VALUE_LOGINCONFIRM=0xF9,PACKET_VALUE_HEART=0xFA,PACKET_VALUE_PACKETCONFIRM=0xFB,PACKET_VALUE_SETBASEMENTMATCHMODE=0xFC,
  PACKET_VALUE_MATCH=0xFD,PACKET_VALUE_PAUSECONTINUE=0xFE
};


enum  MARCStates
{
//State name  Value       Stae
SLEEP       = 0x00,   // SLEEP
IDLE        = 0x01,   // IDLE
XOFF        = 0x02,   // XOFF
VCOON_MC    = 0x03,   // MANCAL
REGON_MC    = 0x04,   // MANCAL
MANCAL      = 0x05,   // MANCAL
VCOON       = 0x06,   // FS_WAKEUP
REGON       = 0x07,   // FS_WAKEUP
STARTCAL    = 0x08,   // CALIBRATE
BWBOOST     = 0x09,   // SETTLING
FS_LOCK     = 0x0A,   // SETTLING
IFADCON     = 0x0B,   // SETTLING
ENDCAL      = 0x0C,   // CALIBRATE
RX          = 0x0D,   // RX
RX_END      = 0x0E,   // RX
RX_RST      = 0x0F,   // RX
TXRX_SWITCH = 0x10,   // TXRX_SETTLING
RX_OVERFLOW = 0x11,   // RX_OVERFLOW
FSTXON      = 0x12,   // FSTXON
TX          = 0x13,   // TX
TX_END      = 0x14,   // TX
RXTX_SWITCH = 0x15,   // RXTX_SETTLING
TX_UNDERFLOW= 0x16    // TX_UNDERFLOW
};

/* Exported variables --------------------------------------------------------*/
//WIRELESS_EXT  uint8_t CC2500_RXBUFF[65];


/* Exported functions ------------------------------------------------------- */
unsigned char CC2500_Init(unsigned char Channel,unsigned char State);
unsigned char CC2500_Reset(unsigned char Channel,unsigned char State);
unsigned char CC2500_Send(unsigned char *Buffer);
unsigned char CC2500_Recv(unsigned char *Buffer,unsigned char Mode);
void CC2500_Channel(unsigned char Channel);
void CC2500_SetState(unsigned char State);
unsigned char ProPacketUpload(unsigned short LocalID,unsigned short TargetID,unsigned char PacketIndex,
                              unsigned char PacketValue,unsigned char *Param,unsigned char *TargetBuffer);








#undef	_WIRELESS_MODULE_
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __CC2500_H */
/**
  * @}
  */ 

/*********************** (C) COPYRIGHT 2017 IdeaMing ******END OF FILE*********/
