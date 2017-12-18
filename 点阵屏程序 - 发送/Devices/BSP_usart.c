/**
  ******************************************************************************
  * @file    Bsp_usart.c
  * @author  Liming
  * @version V1.0.0
  * @date    11-November-2017
  * @brief   This file provides set of firmware functions to xxxx
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2017 IdeaMing</center></h2>
  *
  * @Licensed under 
  * @Revision 
  ******************************************************************************
  */  

/* Module---------------------------------------------------------------------*/
#define	_USART_MODULE_

/* Includes ------------------------------------------------------------------*/
#include "stdio.h"
#include "BSP_usart.h"


/* Private typedef -----------------------------------------------------------*/
#define Dummy_Byte 0xA5
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/ 
#ifdef __GNUC__  
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf 
     set to 'Yes') calls __io_putchar() */  
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)  
#else  
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)  
#endif /* __GNUC__ */  
/* Private functions ---------------------------------------------------------*/



/*******************************************************************************
  * @brief  printf函数重映射
  * @note   使用printf函数要勾选Use MicroLib
  * @param  None
  * @retval None
*******************************************************************************/
void USART1_SendBuf(uint8_t *pBuf, uint32_t u32Len)
{
  while(u32Len--)
  {
    /* 发送区是否为空 */
    while(!USART_GetFlagStatus(USART1,USART_FLAG_TXE));
    USART_SendData(USART1,*pBuf++);
  }
}

/** 
  * @brief  Retargets the C library printf function to the USART. 
  * @param  None 
  * @retval None 
  */  
PUTCHAR_PROTOTYPE  
{  
  /* Place your implementation of fputc here */  
  /* e.g. write a character to the USART */  
  USART_SendData(USART1, (uint8_t) ch);  
  
  /* Loop until the end of transmission */  
  while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)  
  {      
  }  
  return ch;  
} 

uint8_t USART1_ReciverBuf(void)
{
   /* 接收缓冲是否非空 */
  while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE));
  return USART_ReceiveData(USART1);
}

/*******************************************************************************
  * @brief  串口1初始化
  * @param  None
  * @retval None
****************************************************************Author:Liming**/
void USART1_Init(uint32_t BaudRate)
{
  GPIO_InitTypeDef    GPIO_Initstructure;
  USART_InitTypeDef   USART_Initstructure;
//  NVIC_InitTypeDef    NVIC_InitStructure; 
  
  RCC_APB2PeriphClockCmd(USART1_CLK,ENABLE);
  RCC_AHBPeriphClockCmd(USART1_GPIO_CLK,ENABLE);   
  
  /* Connect pin to Periph */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1); 
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);
  
  GPIO_Initstructure.GPIO_Pin=USART1_TX_PIN;
  GPIO_Initstructure.GPIO_Mode=GPIO_Mode_AF;
  GPIO_Initstructure.GPIO_OType=GPIO_OType_PP;  // 推挽输出
  GPIO_Initstructure.GPIO_PuPd=GPIO_PuPd_UP;
  GPIO_Initstructure.GPIO_Speed=GPIO_Speed_50MHz;    
  GPIO_Init(USART1_GPIO_PORT,&GPIO_Initstructure);
   
  GPIO_Initstructure.GPIO_Pin = USART1_RX_PIN;  // 浮空输入  
  GPIO_Init(USART1_GPIO_PORT,&GPIO_Initstructure);
     
  USART_Initstructure.USART_BaudRate = BaudRate;
  USART_Initstructure.USART_Parity   =USART_Parity_No;
  USART_Initstructure.USART_WordLength =USART_WordLength_8b;  
  USART_Initstructure.USART_StopBits  =USART_StopBits_1;
  USART_Initstructure.USART_Mode     = USART_Mode_Rx|USART_Mode_Tx;
  USART_Initstructure.USART_HardwareFlowControl =USART_HardwareFlowControl_None;
  USART_Init(USART1,&USART_Initstructure); 
  
  USART_ClearFlag(USART1,USART_FLAG_TC);
  USART_ITConfig(USART1,USART_IT_TC,ENABLE);
  USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
  USART_Cmd(USART1,ENABLE);      // 使能串口
  
  // 根据手册指导要先使能串口再使能中断，不然会卡在中断出不来 
//  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);     // 使能串口中断  
}








/*********************** (C) COPYRIGHT 2017 IdeaMing ******END OF FILE*********/
