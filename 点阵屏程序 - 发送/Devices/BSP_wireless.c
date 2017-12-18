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
//使用频道值
const unsigned char CHANNEL[CHANNELCOUNT]=
{
  0x52,0x56,0x5A,0x5E,0x62,0x66,0x6A,0x72,0x76,0x7A,0x7E,0x82,0x86,0x8A,0x92,0x96,
  0x9A,0x01,0x05,0x36,0x3A,0x3E,0x42,0x46,0x4A,0xB6,0xBA,0xBE,0xC2,0xC6,0xCA,0xCE
};
unsigned short SYSTEM_MATCH_CODE=0x0000;
//状态:00:运行,01:1级别休眠,02:2级休眠,03:3级休眠
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
  * @brief  模块重启
  * @param  Channel 信道  State 模块状态
  * @retval 1 成功  0 失败
****************************************************************Author:Liming**/
unsigned char CC2500_Reset(unsigned char Channel,unsigned char State)
{
  unsigned short Loop=0x00,time;
  GPIO_SetBits(SPI2_SCK_GPIO_PORT,SPI2_SCK_PIN);  // SCLK=1;
  GPIO_ResetBits(SPI2_MOSI_GPIO_PORT,SPI2_MOSI_PIN);   // 模块SI即单片机MOSI=0
  CC2500_NSS_HIGH();
  for(time=50;time>1;time--);// 6us 逻辑分析仪采样
  CC2500_NSS_LOW();
  for(time=50;time>1;time--);// 6us 滤波低/高  逻辑分析仪采样
  CC2500_NSS_HIGH();
  for(time=500;time>1;time--);// 62.75us >40us  逻辑分析仪采样
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
      SPIWriteReg(IOCFG2,0x01);   // GDO2 功能配置当收到数据时声明
      SPIWriteReg(IOCFG0,0x06);   // GDO0 功能配置
      SPIWriteReg(FIFOTHR,0x0E);  // 设定TX FIFO门限 (0x03+1)*4 Bytes
      SPIWriteReg(PKTCTRL1,0x04);
      SPIWriteReg(PKTCTRL0,0x45);
      SPIWriteReg(PKTLEN,0xFF);   // 设置数据包长度
      SPIWriteReg(CHANNR,CHANNEL[Channel]); // 设置信道
      SPIWriteReg(FSCTRL1,0x06);  // 设置频率 0x06*26Mhz/1024
      SPIWriteReg(FREQ2,0x5C);    // 频率范围 2412MHZ - 2463MHZ CHANNAL 0-255 
      SPIWriteReg(FREQ1,0x50);    // F0
      SPIWriteReg(FREQ0,0xD1);    // 9D
      SPIWriteReg(MDMCFG4,0x7B);  // 调制解调器设置
      SPIWriteReg(MDMCFG3,0x83);
      SPIWriteReg(MDMCFG2,0x13);
      SPIWriteReg(MDMCFG1,0x23);
      SPIWriteReg(DEVIATN,0x44);  // 调制解调器偏差设置
      SPIWriteReg(MCSM1,0x33);    // 主通信控制状态机配置
      SPIWriteReg(MCSM0,0x18);
      SPIWriteReg(FOCCFG,0x16);   // 频率偏移补偿
      SPIWriteReg(AGCCTRL2,0x43); // AGC控制由SmartRF Studio软件给出
      SPIWriteReg(WOREVT1,0xE3);  // 事件0超时高字节
      SPIWriteReg(WOREVT0,0x80);  // 事件0超时低字节
      SPIWriteReg(FSCAL1,0x00);   // 频率合成器校准
      SPIWriteReg(FSCAL0,0x11);
      SPIWriteReg(TEST2,0x81);    // 测试寄存器 
      SPIWriteReg(TEST1,0x35);    // 测试寄存器
      SPIWriteReg(PATABLE,0xFF);  // 功率放大控制
      CC2500State=0xFF;
      CC2500_SetState(State);     // 设置模式
      return 0x01;
    }
  }
  return 0x00;
}


/************************************/

/*************CC2500发送数据*********/
unsigned char CC2500_Send(unsigned char *Buffer)
{
  unsigned short Loop=0x00;
  SPIWriteStrobe(SIDLE);  // 配置进入IDLE
  Loop=0x000A;
  while((SPIReadStatus(MARCSTATE)!=0x01)&&(--Loop));// 等待进入IDLE 
  SPIWriteRegBurst(FIFO,Buffer,Buffer[0x00]+0x01);  // 填充TXFIFO
  SPIWriteStrobe(STX);  // 开始发送
//  Loop=0x008D;
//  while((SPIReadStatus(MARCSTATE)!=0x13)&&(--Loop));// 等待进入发送模式
//  if(Loop)
//  {
//    Loop=0x0AD4;
//    while((!(GPIOA->IDR&0x0800))&&(--Loop));// 等发送完
//    if(Loop)
//    {
//      Loop=0x511E;
//      while((GPIOA->IDR&0x0800)&&(--Loop));
//      if(Loop)
//      {
//        Loop=0x000A;
//        while((SPIReadStatus(MARCSTATE)!=0x0D)&&(--Loop)); // 等进入接收模式
//        if(!Loop) Buffer[0x00]=0x00;  // 发送完修改buffer[0]=0;
//      }
//    }
//  }

//  if(!Buffer[0]) CC2500_SetState(0x00); // 如果发送长度为0，设置为接收模式
//  else CC2500State=0x00;
  return Buffer[0]; // 发送完成返回buffer里的数据长度 如果没发送完则返回不是0
} 
/************************************/

/************CC2500接收数据*********/
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

/************CC2500更改信道*********/
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

/**************模式设置**************/
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

/*************写入Command Strobe********/
void SPIWriteStrobe(unsigned char Reg)
{
  CC2500_NSS_LOW();
  while(GPIO_ReadInputDataBit(SPI2_MISO_GPIO_PORT,SPI2_MISO_PIN));
  CC2500_SPI_WriteRead(Reg);
  CC2500_NSS_HIGH();
}
/**************************/

/*************读取Status Register*******/
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

/*********写入Configuration Register*****/
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

/*******读取Configuration Register***/
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

/******以Burst方式写入Configuration Register***/
void SPIWriteRegBurst(unsigned char Reg,unsigned char *Buffer,unsigned char BufferLen)
{
  CC2500_NSS_LOW();
  while(GPIO_ReadInputDataBit(SPI2_MISO_GPIO_PORT,SPI2_MISO_PIN));
  CC2500_SPI_WriteRead(Reg|0x40);
  while(BufferLen--) CC2500_SPI_WriteRead(*Buffer++);
  CC2500_NSS_HIGH();
}
/**************************/

/*******以Burst方式读取Configuration Register***/
void SPIReadRegBurst(unsigned char Reg,unsigned char *Buffer,unsigned char BufferLen)
{
  CC2500_NSS_LOW();
  while(GPIO_ReadInputDataBit(SPI2_MISO_GPIO_PORT,SPI2_MISO_PIN));
  CC2500_SPI_WriteRead(Reg|0xC0);
  while(BufferLen--) *Buffer++=CC2500_SPI_WriteRead(0x00);
  CC2500_NSS_HIGH();
}

//构建发送数据包
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
