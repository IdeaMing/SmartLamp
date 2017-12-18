/**
  ******************************************************************************
  * @file    BSP_wireless.c
  * @author  Liming
  * @version V1.0.0
  * @date    11-November-2017
  * @brief   This file provides set of firmware functions to xxxx
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2017 IdeaMing</center></h2>
  *
  * Licensed under 
  *
  ******************************************************************************
  */  

/* Module---------------------------------------------------------------------*/
#define	_WIRELESS_MODULE_

/* Includes ------------------------------------------------------------------*/
#include "BSP_wireless.h"
#include "matrix_display.h"
#include "main.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//ʹ��Ƶ��ֵ
const unsigned char CHANNEL[CHANNELCOUNT]=
{
  0x52,0x56,0x5A,0x5E,0x62,0x66,0x6A,0x72,0x76,0x7A,0x7E,0x82,0x86,0x8A,0x92,0x96,
  0x9A,0x01,0x05,0x36,0x3A,0x3E,0x42,0x46,0x4A,0xB6,0xBA,0xBE,0xC2,0xC6,0xCA,0xCE
};
unsigned short SYSTEM_MATCH_CODE=0x0000;
//״̬:00:����,01:1��������,02:2������,03:3������
unsigned char CC2500State=0x00;

/* Private function prototypes -----------------------------------------------*/
unsigned char CC2500_SPI_WriteRead(unsigned char Buffer);
void SPIWriteStrobe(unsigned char Reg);
unsigned char SPIReadStatus(unsigned char Reg);
unsigned char SPIWriteReg(unsigned char Reg,unsigned char Buffer);
unsigned char SPIReadReg(unsigned char Reg);
void SPIWriteRegBurst(unsigned char Reg,unsigned char *Buffer,unsigned char BufferLen);
void SPIReadRegBurst(unsigned char Reg,unsigned char *Buffer,unsigned char BufferLen);
/* Private functions ---------------------------------------------------------*/

/************************
# Data rate = 76.767 
# Preamble count = 4 
# Deviation = 38.085938 
# Data format = Normal mode 
# Modulated = true 
# Sync word qualifier mode = 30/32 sync word bits detected 
# Manchester enable = false 
# Modulation format = GFSK 
# Address config = No address check 
# Packet length = 255 
# Channel number = 114 
# Packet length mode = Variable packet length mode. Packet length configured by the first byte after sync word 
# TX power = 1 
# RX filter BW = 232.142857 
# Base frequency = 2400.207916 
# Channel spacing = 399.902344 
# CRC enable = true
# Carrier frequency = 2403.007233 
# CRC autoflush = false 
# Device address = 0 
# Whitening = true 
************************/

/*******************************************************************************
  * @brief  None
  * @param  None
  * @retval None
****************************************************************Author:Liming**/
unsigned char CC2500_Init(unsigned char Channel,unsigned char State)
{
  State=CC2500_Reset(Channel,State);
  SPIReadReg(IOCFG0);
  SPIReadReg(VERSION);
  return State;
}


/*******************************************************************************
  * @brief  ģ������
  * @param  Channel �ŵ�  State ģ��״̬
  * @retval 1 �ɹ�  0 ʧ��
****************************************************************Author:Liming**/
unsigned char CC2500_Reset(unsigned char Channel,unsigned char State)
{
  unsigned short Loop=0x00,time;
  GPIO_SetBits(SPI2_SCK_GPIO_PORT,SPI2_SCK_PIN);  // SCLK=1;
  GPIO_ResetBits(SPI2_MOSI_GPIO_PORT,SPI2_MOSI_PIN);   // ģ��SI����Ƭ��MOSI=0
  CC2500_NSS_HIGH();
  for(time=50;time>1;time--);// 6us �߼������ǲ���
  CC2500_NSS_LOW();
  for(time=50;time>1;time--);// 6us �˲���/��  �߼������ǲ���
  CC2500_NSS_HIGH();
  for(time=500;time>1;time--);// 62.75us >40us  �߼������ǲ���
  CC2500_NSS_LOW();
  Loop=0xFFFF;
  while((GPIO_ReadInputDataBit(SPI2_MISO_GPIO_PORT,SPI2_MISO_PIN))&&(--Loop));
  if(Loop)
  {
    CC2500_SPI_WriteRead(SRES);
    Loop=0xFFFF;
    while((GPIO_ReadInputDataBit(SPI2_MISO_GPIO_PORT,SPI2_MISO_PIN))&&(--Loop));
    CC2500_NSS_HIGH();
    if((Loop)&&(SPIReadStatus(PARTNUM)==0x80))
    {
      SPIWriteReg(IOCFG2,0x01);   // GDO2 �������õ��յ�����ʱ����
      SPIWriteReg(IOCFG0,0x06);   // GDO0 ��������
      SPIWriteReg(FIFOTHR,0x0E);  // �趨TX FIFO���� (0x03+1)*4 Bytes
      SPIWriteReg(PKTCTRL1,0x04);
      SPIWriteReg(PKTCTRL0,0x45);
      SPIWriteReg(PKTLEN,0xFF);   // �������ݰ�����
      SPIWriteReg(CHANNR,CHANNEL[Channel]); // �����ŵ�
      SPIWriteReg(FSCTRL1,0x06);  // ����Ƶ�� 0x06*26Mhz/1024
      SPIWriteReg(FREQ2,0x5C);    // Ƶ�ʷ�Χ 2412MHZ - 2463MHZ CHANNAL 0-255 
      SPIWriteReg(FREQ1,0x50);    // F0
      SPIWriteReg(FREQ0,0xD1);    // 9D
      SPIWriteReg(MDMCFG4,0x7B);  // ���ƽ��������
      SPIWriteReg(MDMCFG3,0x83);
      SPIWriteReg(MDMCFG2,0x13);
      SPIWriteReg(MDMCFG1,0x23);
      SPIWriteReg(DEVIATN,0x44);  // ���ƽ����ƫ������
      SPIWriteReg(MCSM1,0x33);    // ��ͨ�ſ���״̬������
      SPIWriteReg(MCSM0,0x18);
      SPIWriteReg(FOCCFG,0x16);   // Ƶ��ƫ�Ʋ���
      SPIWriteReg(AGCCTRL2,0x43); // AGC������SmartRF Studio�������
      SPIWriteReg(WOREVT1,0xE3);  // �¼�0��ʱ���ֽ�
      SPIWriteReg(WOREVT0,0x80);  // �¼�0��ʱ���ֽ�
      SPIWriteReg(FSCAL1,0x00);   // Ƶ�ʺϳ���У׼
      SPIWriteReg(FSCAL0,0x11);
      SPIWriteReg(TEST2,0x81);    // ���ԼĴ��� 
      SPIWriteReg(TEST1,0x35);    // ���ԼĴ���
      SPIWriteReg(PATABLE,0xFF);  // ���ʷŴ����
      CC2500State=0xFF;
      CC2500_SetState(State);     // ����ģʽ
      return 0x01;
    }
  }
  return 0x00;
}


/************************************/

/*************CC2500��������*********/
unsigned char CC2500_Send(unsigned char *Buffer)
{
  unsigned short Loop=0x00;
  SPIWriteStrobe(SIDLE);  // ���ý���IDLE
  Loop=0x000A;
  while((SPIReadStatus(MARCSTATE)!=0x01)&&(--Loop));// �ȴ�����IDLE 
  SPIWriteRegBurst(FIFO,Buffer,Buffer[0x00]+0x01);  // ���TXFIFO
  SPIWriteStrobe(STX);  // ��ʼ����
//  Loop=0x008D;
//  while((SPIReadStatus(MARCSTATE)!=0x13)&&(--Loop));// �ȴ����뷢��ģʽ
//  if(Loop)
//  {
//    Loop=0x0AD4;
//    while((!(GPIOA->IDR&0x0800))&&(--Loop));// �ȷ�����
//    if(Loop)
//    {
//      Loop=0x511E;
//      while((GPIOA->IDR&0x0800)&&(--Loop));
//      if(Loop)
//      {
//        Loop=0x000A;
//        while((SPIReadStatus(MARCSTATE)!=0x0D)&&(--Loop)); // �Ƚ������ģʽ
//        if(!Loop) Buffer[0x00]=0x00;  // �������޸�buffer[0]=0;
//      }
//    }
//  }

//  if(!Buffer[0]) CC2500_SetState(0x00); // ������ͳ���Ϊ0������Ϊ����ģʽ
//  else CC2500State=0x00;
  return Buffer[0]; // ������ɷ���buffer������ݳ��� ���û�������򷵻ز���0
} 
/************************************/

/************CC2500��������*********/
unsigned char CC2500_Recv(unsigned char *Buffer,unsigned char Mode)
{
    unsigned char Result=0x00,Status=0x00;
    unsigned short  time;
    switch(CC2500State)
    {
    case 0x00:
    case 0x01:
    if(CC2500State) Delay(0x07);
    else 
    {
        for(time=7200;time>1;time--);
    }
    Status=SPIReadStatus(RXBYTES);
    if(Status&0x80)
    {
      SPIReadRegBurst(FIFO,Buffer,0x40);
      Result=0x40;
    }
    else if(Status&0x7F)
    {
      SPIReadRegBurst(FIFO,Buffer,Status&0x7F);
      Result=Status&0x7F;
    }
    CC2500_SetState(Mode);
    break;
    }
    return Result;
}
/************************************/

/************CC2500�����ŵ�*********/
void CC2500_Channel(unsigned char Channel)
{
  unsigned char Loop=0x00;
  if(Channel<CHANNELCOUNT)
  {
    SPIWriteStrobe(SIDLE);
    Loop=0x0A;
    while((SPIReadStatus(MARCSTATE)!=0x01)&&(--Loop));
    SPIWriteReg(CHANNR,CHANNEL[Channel]);
    CC2500_SetState(CC2500State);
  }
}
/************************************/

/**************ģʽ����**************/
void CC2500_SetState(unsigned char State)
{
  unsigned char Loop=0x00;
  SPIWriteStrobe(SIDLE);
  Loop=0x0A;
  while((SPIReadStatus(MARCSTATE)!=0x01)&&(--Loop));
  switch(State)
  {
  case 0x00:
    //Rx
    SPIWriteReg(IOCFG2,0x01);
    SPIWriteReg(MCSM2,0x07);
    SPIWriteReg(MCSM0,0x18);
    SPIWriteReg(WORCTRL,0xF8);
    SPIWriteStrobe(SFTX);
    SPIWriteStrobe(SFRX);
    SPIWriteStrobe(SRX);
    Loop=0x8D;
    while((SPIReadStatus(MARCSTATE)!=0x0D)&&(--Loop));
    break;
  case 0x01:
    //Wor
    SPIWriteReg(IOCFG2,0x06);
    SPIWriteReg(MCSM2,0x00);
    SPIWriteReg(MCSM0,0x38);
    SPIWriteReg(WORCTRL,0x78);
    SPIWriteStrobe(SWORRST);
    SPIWriteStrobe(SWOR);
    break;
  case 0x02:
    //Sleep
    SPIWriteStrobe(SPWD);
    break;
  }
  CC2500State=State;
}

/**********************************/

/*************д��Command Strobe********/
void SPIWriteStrobe(unsigned char Reg)
{
  CC2500_NSS_LOW();
  while(GPIO_ReadInputDataBit(SPI2_MISO_GPIO_PORT,SPI2_MISO_PIN));
  CC2500_SPI_WriteRead(Reg);
  CC2500_NSS_HIGH();
}
/**************************/

/*************��ȡStatus Register*******/
unsigned char SPIReadStatus(unsigned char Reg)
{
  unsigned char Buffer=0x00;
  CC2500_NSS_LOW();
  while(GPIO_ReadInputDataBit(SPI2_MISO_GPIO_PORT,SPI2_MISO_PIN));
  CC2500_SPI_WriteRead(Reg|0xC0);
  Buffer=CC2500_SPI_WriteRead(0x00);
  CC2500_NSS_HIGH();
  return Buffer;
}
/**************************/

/*********д��Configuration Register*****/
unsigned char SPIWriteReg(unsigned char Reg,unsigned char Buffer)
{
  CC2500_NSS_LOW();
  while(GPIO_ReadInputDataBit(SPI2_MISO_GPIO_PORT,SPI2_MISO_PIN));
  CC2500_SPI_WriteRead(Reg);
  Buffer=CC2500_SPI_WriteRead(Buffer);
  CC2500_NSS_HIGH();
  return Buffer;  
}
/**************************/

/*******��ȡConfiguration Register***/
unsigned char SPIReadReg(unsigned char Reg)
{
  unsigned char Buffer=0x00;
  CC2500_NSS_LOW();  
  while(GPIO_ReadInputDataBit(SPI2_MISO_GPIO_PORT,SPI2_MISO_PIN));
  CC2500_SPI_WriteRead(Reg|0x80);
  Buffer=CC2500_SPI_WriteRead(0x00);
  CC2500_NSS_HIGH();
  return Buffer;
}
/**************************/

/******��Burst��ʽд��Configuration Register***/
void SPIWriteRegBurst(unsigned char Reg,unsigned char *Buffer,unsigned char BufferLen)
{
  CC2500_NSS_LOW();
  while(GPIO_ReadInputDataBit(SPI2_MISO_GPIO_PORT,SPI2_MISO_PIN));
  CC2500_SPI_WriteRead(Reg|0x40);
  while(BufferLen--) CC2500_SPI_WriteRead(*Buffer++);
  CC2500_NSS_HIGH();
}
/**************************/

/*******��Burst��ʽ��ȡConfiguration Register***/
void SPIReadRegBurst(unsigned char Reg,unsigned char *Buffer,unsigned char BufferLen)
{
  CC2500_NSS_LOW();
  while(GPIO_ReadInputDataBit(SPI2_MISO_GPIO_PORT,SPI2_MISO_PIN));
  CC2500_SPI_WriteRead(Reg|0xC0);
  while(BufferLen--) *Buffer++=CC2500_SPI_WriteRead(0x00);
  CC2500_NSS_HIGH();
}

//�����������ݰ�
unsigned char ProPacketUpload(unsigned short LocalID,unsigned short TargetID,unsigned char PacketIndex,
                              unsigned char PacketValue,unsigned char *Param,unsigned char *TargetBuffer)
{
  TargetBuffer[0x00]=SENDPACKETLEN-0x01;
  TargetBuffer[0x01]=PROHEAD&0x00FF;
  TargetBuffer[0x02]=(PROHEAD&0xFF00)>>0x08;
  TargetBuffer[0x03]=SYSTEM_MATCH_CODE;
  TargetBuffer[0x04]=SYSTEM_MATCH_CODE>>0x08;
  TargetBuffer[0x05]=LocalID;
  TargetBuffer[0x06]=LocalID>>0x08;
  TargetBuffer[0x07]=TargetID;
  TargetBuffer[0x08]=TargetID>>0x08;
  TargetBuffer[0x09]=PacketIndex;
  TargetBuffer[0x0A]=PacketValue;
  memcpy(&TargetBuffer[0x0B],&Param[0x00],SENDPACKETLEN-0x0A);      //
  return SENDPACKETLEN;
}


/*********************** (C) COPYRIGHT 2017 IdeaMing ******END OF FILE*********/
