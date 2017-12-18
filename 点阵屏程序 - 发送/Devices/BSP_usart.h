/**
  ******************************************************************************
  * @file    Bsp_usart.h
  * @author  Liming
  * @version V1.0.0
  * @date    11-November-2017
  * @brief   This file contains all the functions prototypes for the
  *          bsp_usart.c driver.
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
#ifndef __USART_H
#define __USART_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Module---------------------------------------------------------------------*/	 
#ifdef	_USART_MODULE_
#define	USART_EXT
#else
#define	USART_EXT	extern
#endif
	 
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include "stdio.h"
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/ 

/** @addtogroup MATRIX_DISPLAY_LOW_LEVEL_UART
  * @{
  */
#define USART1_TX_PIN            GPIO_Pin_9
#define USART1_RX_PIN            GPIO_Pin_10
#define USART1_GPIO_PORT         GPIOA
#define USART1_GPIO_CLK          RCC_AHBPeriph_GPIOA
#define USART1_CLK               RCC_APB2Periph_USART1

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void USART1_Init(uint32_t BaudRate);



#undef	_USART_MODULE_


#ifdef __cplusplus
}
#endif

#endif /* __USART_H */


/*********************** (C) COPYRIGHT 2017 IdeaMing ******END OF FILE*********/
